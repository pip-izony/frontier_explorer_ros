#!/usr/bin/env python3
"""NBV selector using frontier_extractor's cluster centroids as anchors.

Implements proposal eqs. (3), (4) with sparse raycast (§4.3):
    I(x_c, M) = | Vis(x_c, M) ∩ F |
    x*        = argmax_{x_c}  I(x_c, M) / d(x_t, x_c)
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


def quantize(pts, res):
    return np.floor(np.asarray(pts) / res).astype(np.int32)


def sample_directions(n):
    """Golden-spiral unit vectors on the sphere — sparse raycast (proposal §4.3)."""
    idx = np.arange(0, n, dtype=float) + 0.5
    phi = np.arccos(1 - 2 * idx / n)
    theta = np.pi * (1 + 5 ** 0.5) * idx
    return np.column_stack([
        np.cos(theta) * np.sin(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(phi),
    ])


class NBVSelector(Node):
    def __init__(self):
        super().__init__('nbv_selector')

        self.declare_parameter('voxel_size',       0.1)
        self.declare_parameter('r_max',            8.0)
        self.declare_parameter('n_ray_dirs',       48)
        self.declare_parameter('k_per_cluster',    5)
        self.declare_parameter('sampling_radius',  2.0)
        self.declare_parameter('min_z',            0.6)
        self.declare_parameter('max_z',            2.3)
        self.declare_parameter('replan_period',    1.0)
        self.declare_parameter('min_gain_threshold', 2)

        g = lambda n: self.get_parameter(n).value
        self.res         = float(g('voxel_size'))
        self.r_max       = float(g('r_max'))
        self.n_rays      = int(g('n_ray_dirs'))
        self.k_per_c     = int(g('k_per_cluster'))
        self.sample_r    = float(g('sampling_radius'))
        self.min_z       = float(g('min_z'))
        self.max_z       = float(g('max_z'))
        self.replan_T    = float(g('replan_period'))
        self.min_gain    = int(g('min_gain_threshold'))

        self.occupied = set()
        self.free     = set()
        self.frontier = set()
        self.centroids = []
        self.current_pos = None
        self.frame_id = 'simple_drone/odom'

        self.directions = sample_directions(self.n_rays)
        self.n_steps = int(self.r_max / self.res)

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(PoseArray,
            '/frontier_extractor/cluster_centroids', self.on_centroids, 10)
        self.create_subscription(PointCloud2,
            '/frontier_extractor/frontier_cloud', self.on_frontier, 10)
        self.create_subscription(PointCloud2,
            '/octomap_point_cloud_centers', self.on_occ, latched)
        self.create_subscription(MarkerArray,
            '/free_cells_vis_array', self.on_free, latched)
        self.create_subscription(PoseStamped,
            '/simple_drone/gt_pose', self.on_pose, 10)

        self.pub_nbv  = self.create_publisher(PoseStamped, '/next_viewpoint', 10)
        self.pub_cand = self.create_publisher(MarkerArray, '/nbv_candidates', 10)

        self.create_timer(self.replan_T, self.tick)
        self.get_logger().info(
            f"NBV up. res={self.res}m r_max={self.r_max}m rays={self.n_rays}"
        )

    def on_pose(self, msg):
        self.current_pos = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ], dtype=np.float64)

    def on_centroids(self, msg):
        self.frame_id = msg.header.frame_id or self.frame_id
        self.centroids = [
            np.array([p.position.x, p.position.y, p.position.z]) for p in msg.poses
        ]

    def on_frontier(self, msg):
        try:
            from sensor_msgs_py.point_cloud2 import read_points_numpy
            pts = read_points_numpy(msg, field_names=('x','y','z'), skip_nans=True)
            pts = np.asarray(pts).reshape(-1, 3)
        except Exception:
            return
        if pts.size:
            self.frontier = set(map(tuple, quantize(pts, self.res).tolist()))

    def on_occ(self, msg):
        try:
            from sensor_msgs_py.point_cloud2 import read_points_numpy
            pts = read_points_numpy(msg, field_names=('x','y','z'), skip_nans=True)
            pts = np.asarray(pts).reshape(-1, 3)
        except Exception:
            return
        self.occupied = set(map(tuple, quantize(pts, self.res).tolist())) if pts.size else set()

    def on_free(self, msg):
        pts = []
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            for p in m.points:
                pts.append((p.x, p.y, p.z))
        if pts:
            self.free = set(map(tuple, quantize(np.array(pts), self.res).tolist()))
        else:
            self.free = set()

    def tick(self):
        if self.current_pos is None or not self.centroids or not self.frontier:
            return

        candidates = []
        for c in self.centroids:
            for _ in range(self.k_per_c):
                r = np.random.uniform(0.8, self.sample_r)
                theta = np.random.uniform(0, 2*np.pi)
                phi   = np.random.uniform(np.pi/3, 2*np.pi/3)
                off = r * np.array([
                    np.sin(phi)*np.cos(theta),
                    np.sin(phi)*np.sin(theta),
                    np.cos(phi),
                ])
                cand = c + off
                cand[2] = float(np.clip(cand[2], self.min_z, self.max_z))
                key = tuple(quantize(cand.reshape(1, 3), self.res)[0].tolist())
                if key in self.occupied:
                    continue
                candidates.append(cand)

        if not candidates:
            return

        scores = []
        gains  = []
        for c in candidates:
            gain = self.compute_gain(c)
            gains.append(gain)
            cost = max(float(np.linalg.norm(c - self.current_pos)), 0.5)
            scores.append(gain / cost)

        best_idx = int(np.argmax(scores))
        if gains[best_idx] < self.min_gain:
            return

        self.publish_nbv(candidates[best_idx])
        self.publish_candidates(candidates, scores, best_idx)
        self.get_logger().info(
            f"NBV: ({candidates[best_idx][0]:.2f},{candidates[best_idx][1]:.2f},"
            f"{candidates[best_idx][2]:.2f}) gain={gains[best_idx]} "
            f"clusters={len(self.centroids)}",
            throttle_duration_sec=2.0,
        )

    def compute_gain(self, vp):
        """I(x_c) = |Vis ∩ F| via sparse raycast (§4.3)."""
        gain = 0
        vp = np.asarray(vp, dtype=np.float64)
        for d in self.directions:
            for i in range(1, self.n_steps + 1):
                p = vp + d * (i * self.res)
                key = tuple(quantize(p.reshape(1, 3), self.res)[0].tolist())
                if key in self.occupied:
                    break
                if key in self.frontier:
                    gain += 1
                    break  # §4.3: 1 frontier hit per ray
        return gain

    def publish_nbv(self, p):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        dx = p[0] - self.current_pos[0]
        dy = p[1] - self.current_pos[1]
        yaw = math.atan2(dy, dx)
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)
        self.pub_nbv.publish(msg)

    def publish_candidates(self, cands, scores, best_idx):
        ma = MarkerArray()
        clr = Marker()
        clr.header.frame_id = self.frame_id
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)
        smin, smax = min(scores), max(scores)
        srng = (smax - smin) if smax > smin else 1.0
        for i, (c, s) in enumerate(zip(cands, scores)):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns, m.id = 'nbv', i
            m.type, m.action = Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = \
                float(c[0]), float(c[1]), float(c[2])
            m.pose.orientation.w = 1.0
            if i == best_idx:
                m.scale.x = m.scale.y = m.scale.z = 0.4
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                m.scale.x = m.scale.y = m.scale.z = 0.15
                n = (s - smin) / srng
                m.color.r, m.color.g = float(1.0 - n), float(n)
                m.color.b, m.color.a = 0.2, 0.6
            ma.markers.append(m)
        self.pub_cand.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = NBVSelector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
