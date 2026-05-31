#!/usr/bin/env python3
"""
frontier_extractor – ROS 2 node

Inputs  (from octomap_server, requires publish_free_space:=true):
  /free_cells_vis_array          visualization_msgs/MarkerArray  (latched)
  /octomap_point_cloud_centers   sensor_msgs/PointCloud2         (latched)

Outputs:
  ~/frontier_cloud       sensor_msgs/PointCloud2        – all frontier voxels
  ~/cluster_centroids    geometry_msgs/PoseArray         – cluster centroids (NBV input)
  ~/cluster_markers      visualization_msgs/MarkerArray  – RViz spheres
  ~/status               std_msgs/String                 – JSON stats
"""

import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointField
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

from frontier_explorer_py._utils import (
    extract_frontiers,
    cluster_frontiers,
    positions_to_keys,
)

_PALETTE = [
    (1.0, 0.2, 0.2), (0.2, 1.0, 0.2), (0.2, 0.2, 1.0), (1.0, 1.0, 0.0),
    (1.0, 0.5, 0.0), (0.5, 0.0, 1.0), (0.0, 1.0, 1.0), (1.0, 0.0, 1.0),
]


class FrontierExtractor(Node):
    def __init__(self):
        super().__init__('frontier_extractor')

        self.declare_parameter('cluster_radius', 1.5)
        self.declare_parameter('min_cluster_size', 5)
        self.declare_parameter('free_cells_topic', '/free_cells_vis_array')
        self.declare_parameter('occ_cells_topic', '/octomap_point_cloud_centers')

        self._cluster_radius    = self.get_parameter('cluster_radius').value
        self._min_cluster_size  = self.get_parameter('min_cluster_size').value
        free_topic              = self.get_parameter('free_cells_topic').value
        occ_topic               = self.get_parameter('occ_cells_topic').value

        # Match octomap_server's transient_local (latched) QoS.
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.create_subscription(MarkerArray, free_topic, self._on_free_cells, latched)
        self.create_subscription(PointCloud2, occ_topic,  self._on_occ_cells,  latched)

        self._frontier_cloud_pub   = self.create_publisher(PointCloud2,  '~/frontier_cloud',    10)
        self._cluster_centroids_pub = self.create_publisher(PoseArray,    '~/cluster_centroids', 10)
        self._cluster_markers_pub  = self.create_publisher(MarkerArray,   '~/cluster_markers',   10)
        self._status_pub           = self.create_publisher(String,        '~/status',            10)

        self._free_pts:  np.ndarray | None = None
        self._occ_pts:   np.ndarray | None = None
        self._resolution: float | None = None
        self._frame_id = 'map'

        self.get_logger().info(
            f'frontier_extractor ready. '
            f'free={free_topic} occ={occ_topic} '
            f'cluster_radius={self._cluster_radius} '
            f'min_cluster_size={self._min_cluster_size}'
        )

    # ── subscribers ──────────────────────────────────────────────────────────

    def _on_free_cells(self, msg: MarkerArray):
        pts = []
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            pts.append((m.pose.position.x, m.pose.position.y, m.pose.position.z))
            if self._resolution is None and m.scale.x > 0:
                self._resolution = float(m.scale.x)
                self._frame_id   = m.header.frame_id
        self._free_pts = np.array(pts, dtype=np.float32) if pts else np.empty((0, 3), dtype=np.float32)
        self._try_update()

    def _on_occ_cells(self, msg: PointCloud2):
        self._frame_id = msg.header.frame_id
        try:
            from sensor_msgs_py.point_cloud2 import read_points_numpy
            arr = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            self._occ_pts = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
        except Exception:
            gen = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
            lst = list(gen)
            self._occ_pts = (np.array(lst, dtype=np.float32).reshape(-1, 3)
                             if lst else np.empty((0, 3), dtype=np.float32))
        self._try_update()

    # ── core update ──────────────────────────────────────────────────────────

    def _try_update(self):
        if self._free_pts is None or self._occ_pts is None or self._resolution is None:
            return
        if len(self._free_pts) == 0:
            self._publish_status(0, 0)
            return

        res = self._resolution

        free_keys = positions_to_keys(self._free_pts, res)
        occ_keys  = positions_to_keys(self._occ_pts,  res) if len(self._occ_pts) else np.empty((0, 3), dtype=np.int32)

        known = set(map(tuple, free_keys))
        for k in occ_keys:
            known.add((int(k[0]), int(k[1]), int(k[2])))

        frontier_key_list = extract_frontiers(
            [tuple(k) for k in free_keys], known)

        if not frontier_key_list:
            self._publish_status(0, 0)
            return

        # Map frontier keys back to metric positions.
        frontier_pts = np.array(frontier_key_list, dtype=np.float32) * res

        centroids, labels = cluster_frontiers(
            frontier_pts, self._cluster_radius, self._min_cluster_size)

        hdr = Header()
        hdr.frame_id = self._frame_id
        hdr.stamp    = self.get_clock().now().to_msg()

        self._publish_frontier_cloud(frontier_pts, labels, hdr)
        self._publish_cluster_markers(centroids, hdr)
        self._publish_cluster_centroids(centroids, hdr)
        self._publish_status(len(frontier_pts), len(centroids))

        self.get_logger().info(
            f'Frontiers: {len(frontier_pts)} voxels / {len(centroids)} clusters')

    # ── publishers ───────────────────────────────────────────────────────────

    def _publish_frontier_cloud(self, pts, labels, hdr):
        if self._frontier_cloud_pub.get_subscription_count() == 0:
            return
        fields = [
            PointField(name='x',          offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',          offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',          offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='cluster_id', offset=12, datatype=PointField.INT32,   count=1),
        ]
        data = np.column_stack([pts, labels.reshape(-1, 1).view(np.float32)])
        msg = point_cloud2.create_cloud(hdr, fields, data.tolist())
        self._frontier_cloud_pub.publish(msg)

    def _publish_cluster_markers(self, centroids, hdr):
        if self._cluster_markers_pub.get_subscription_count() == 0:
            return
        ma = MarkerArray()
        del_m = Marker()
        del_m.action = Marker.DELETEALL
        ma.markers.append(del_m)
        for i, c in enumerate(centroids):
            m = Marker()
            m.header = hdr
            m.ns     = 'frontier_clusters'
            m.id     = i
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(c[0])
            m.pose.position.y = float(c[1])
            m.pose.position.z = float(c[2])
            m.pose.orientation.w = 1.0
            r = max(0.3, self._cluster_radius * 0.4) * 2.0
            m.scale.x = m.scale.y = m.scale.z = r
            col = _PALETTE[i % len(_PALETTE)]
            m.color.r, m.color.g, m.color.b, m.color.a = col[0], col[1], col[2], 0.7
            ma.markers.append(m)
        self._cluster_markers_pub.publish(ma)

    def _publish_cluster_centroids(self, centroids, hdr):
        pa = PoseArray()
        pa.header = hdr
        for c in centroids:
            p = Pose()
            p.position.x, p.position.y, p.position.z = float(c[0]), float(c[1]), float(c[2])
            p.orientation.w = 1.0
            pa.poses.append(p)
        self._cluster_centroids_pub.publish(pa)

    def _publish_status(self, n_frontiers, n_clusters):
        msg = String()
        msg.data = json.dumps({'num_frontiers': n_frontiers, 'num_clusters': n_clusters})
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
