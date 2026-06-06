#!/usr/bin/env python3
"""Pick the nearest unvisited frontier centroid and publish it as /next_viewpoint.
Simple frontier exploration: 'go to the nearest unexplored boundary',
without NBV's raycast/information-gain computation."""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose


class GoalPicker(Node):
    def __init__(self):
        super().__init__('goal_picker')
        self.declare_parameter('pick_period', 1.0)
        self.declare_parameter('reach_tol', 0.8)        # distance to count goal as reached
        self.declare_parameter('min_dist', 1.0)         # ignore centroids closer than this
        self.declare_parameter('cruise_z', 0.8)         # flight altitude for goals
        self.declare_parameter('revisit_radius', 1.5)   # avoid re-picking near visited goals
        self.declare_parameter('bbox', [-6.5, 6.5, -6.5, 6.5])
        self.declare_parameter('stuck_timeout', 15.0)   # give up on a goal after this

        g = lambda n: self.get_parameter(n).value
        self.period    = float(g('pick_period'))
        self.reach_tol = float(g('reach_tol'))
        self.min_dist  = float(g('min_dist'))
        self.cruise_z  = float(g('cruise_z'))
        self.revisit_r = float(g('revisit_radius'))
        self.bbox      = list(g('bbox'))
        self.stuck_to  = float(g('stuck_timeout'))

        self.current_pos = None
        self.centroids   = []
        self.frame_id    = 'simple_drone/odom'
        self.goal        = None
        self.goal_t      = None
        self.visited     = []

        self.create_subscription(Pose, '/simple_drone/gt_pose', self.on_pose, 10)
        self.create_subscription(PoseArray,
            '/frontier_extractor/cluster_centroids', self.on_centroids, 10)
        self.pub = self.create_publisher(PoseStamped, '/next_viewpoint', 10)
        self.create_timer(self.period, self.tick)
        self.get_logger().info("GoalPicker up.")

    def on_pose(self, msg):
        self.current_pos = np.array([msg.position.x, msg.position.y, msg.position.z])

    def on_centroids(self, msg):
        self.frame_id = msg.header.frame_id or self.frame_id
        self.centroids = [np.array([p.position.x, p.position.y, p.position.z])
                          for p in msg.poses]

    def in_bbox_xy(self, p):
        return (self.bbox[0] <= p[0] <= self.bbox[1] and
                self.bbox[2] <= p[1] <= self.bbox[3])

    def visited_recently(self, p):
        return any(np.linalg.norm(p[:2] - v[:2]) < self.revisit_r for v in self.visited)

    def tick(self):
        if self.current_pos is None or not self.centroids:
            return
        now = self.get_clock().now()

        # If we already have a goal, check whether it's reached or timed out
        if self.goal is not None:
            d = float(np.linalg.norm(self.goal[:2] - self.current_pos[:2]))
            elapsed = (now - self.goal_t).nanoseconds * 1e-9
            if d < self.reach_tol:
                self.get_logger().info("Goal reached -> next.")
                self.visited.append(self.goal.copy()); self.goal = None
            elif elapsed > self.stuck_to:
                self.get_logger().warn("Goal timeout -> skip.")
                self.visited.append(self.goal.copy()); self.goal = None
            else:
                return   # still heading to the current goal

        # Pick the nearest valid frontier centroid
        best, best_d = None, float('inf')
        for c in self.centroids:
            if not self.in_bbox_xy(c) or self.visited_recently(c):
                continue
            d = float(np.linalg.norm(c[:2] - self.current_pos[:2]))
            if d < self.min_dist:
                continue
            if d < best_d:
                best, best_d = c, d

        if best is None:
            self.get_logger().info("No frontier left.", throttle_duration_sec=5.0)
            return

        goal = best.copy(); goal[2] = self.cruise_z   # fly at cruise altitude
        self.goal, self.goal_t = goal, now
        m = PoseStamped()
        m.header.frame_id = self.frame_id
        m.header.stamp = now.to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(goal[0]), float(goal[1]), float(goal[2])
        m.pose.orientation.w = 1.0
        self.pub.publish(m)
        self.get_logger().info(
            f"New goal ({goal[0]:.2f},{goal[1]:.2f},{goal[2]:.2f}) d={best_d:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = GoalPicker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
