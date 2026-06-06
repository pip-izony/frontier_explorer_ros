#!/usr/bin/env python3
import csv, time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray


class ExploreLogger(Node):
    def __init__(self):
        super().__init__('explore_logger')
        self.declare_parameter('out', 'run.csv')
        self.declare_parameter('period', 1.0)
        self.out = self.get_parameter('out').value
        period = float(self.get_parameter('period').value)

        self.occ = 0
        self.free = 0
        self.path_len = 0.0
        self.prev_pos = None
        self.t0 = time.time()

        self.create_subscription(PointCloud2, '/octomap_point_cloud_centers', self.on_occ, 10)
        self.create_subscription(MarkerArray, '/free_cells_vis_array', self.on_free, 10)
        self.create_subscription(Pose, '/simple_drone/gt_pose', self.on_pose, 10)

        self.f = open(self.out, 'w', newline='')
        self.w = csv.writer(self.f)
        self.w.writerow(['t', 'occupied', 'free', 'known', 'path_len'])
        self.create_timer(period, self.tick)
        self.get_logger().info(f"Logging to {self.out}")

    def on_occ(self, msg):
        self.occ = msg.width * msg.height                    # occupied voxel count
    def on_free(self, msg):
        self.free = sum(len(m.points) for m in msg.markers if m.action == Marker.ADD)
    def on_pose(self, msg):
        p = np.array([msg.position.x, msg.position.y, msg.position.z])
        if self.prev_pos is not None:
            self.path_len += float(np.linalg.norm(p - self.prev_pos))
        self.prev_pos = p

    def tick(self):
        t = time.time() - self.t0
        self.w.writerow([f"{t:.2f}", self.occ, self.free,
                         self.occ + self.free, f"{self.path_len:.2f}"])
        self.f.flush()


def main(args=None):
    rclpy.init(args=args)
    node = ExploreLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.f.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
