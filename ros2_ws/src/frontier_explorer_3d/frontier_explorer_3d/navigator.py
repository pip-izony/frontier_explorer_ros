#!/usr/bin/env python3
"""Navigator: auto-takeoff + A* + path follower.

Listens on /next_viewpoint (from NBV).
Publishes cmd_vel_raw so collision_avoidance is in the safety loop.

State machine: TAKEOFF -> HOVER -> PLAN -> FOLLOW -> HOVER -> ...
"""
import math
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty
from nav_msgs.msg import Path


def quantize(p, res):
    return tuple(int(np.floor(c / res)) for c in p)


class Navigator(Node):
    TAKEOFF = 'TAKEOFF'
    HOVER   = 'HOVER'
    PLAN    = 'PLAN'
    FOLLOW  = 'FOLLOW'

    def __init__(self):
        super().__init__('navigator')

        self.declare_parameter('cmd_vel_topic',     '/simple_drone/cmd_vel_raw')
        self.declare_parameter('takeoff_altitude',  1.2)
        self.declare_parameter('bbox',              [-7.0, 7.0, -7.0, 7.0, 0.4, 2.5])
        self.declare_parameter('astar_voxel_size',  0.4)
        self.declare_parameter('safety_inflation',  1)
        self.declare_parameter('astar_max_iter',    30000)
        self.declare_parameter('cruise_speed',      0.4)
        self.declare_parameter('reach_tol',         0.6)
        self.declare_parameter('lookahead',         1.5)
        self.declare_parameter('replan_period',     5.0)

        g = lambda n: self.get_parameter(n).value
        self.cmd_topic   = g('cmd_vel_topic')
        self.takeoff_alt = float(g('takeoff_altitude'))
        self.bbox        = list(g('bbox'))
        self.voxel       = float(g('astar_voxel_size'))
        self.inflation   = int(g('safety_inflation'))
        self.max_iter    = int(g('astar_max_iter'))
        self.cruise      = float(g('cruise_speed'))
        self.reach_tol   = float(g('reach_tol'))
        self.lookahead   = float(g('lookahead'))
        self.replan_T    = float(g('replan_period'))

        self.state          = self.TAKEOFF
        self.current_pos    = None
        self.frame_id       = 'simple_drone/odom'
        self.occupied       = set()
        self.current_goal   = None
        self.path           = []
        self.path_idx       = 0
        self.takeoff_sent_t = None
        self.last_replan_t  = None

        self.nbrs = [(dx, dy, dz)
                     for dx in (-1, 0, 1) for dy in (-1, 0, 1) for dz in (-1, 0, 1)
                     if (dx, dy, dz) != (0, 0, 0)]

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(PoseStamped, '/simple_drone/gt_pose', self.on_pose, 10)
        self.create_subscription(PoseStamped, '/next_viewpoint', self.on_goal, 10)
        self.create_subscription(PointCloud2,
            '/octomap_point_cloud_centers', self.on_occ, latched)

        self.pub_takeoff = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.pub_cmd     = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_path    = self.create_publisher(Path, '/planned_path', 10)

        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"Navigator up. Publishing cmd_vel to '{self.cmd_topic}'."
        )

    def on_pose(self, msg):
        self.current_pos = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ], dtype=np.float64)
        self.frame_id = msg.header.frame_id or self.frame_id

    def on_goal(self, msg):
        self.current_goal = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ], dtype=np.float64)
        if self.state in (self.HOVER, self.FOLLOW):
            self.state = self.PLAN

    def on_occ(self, msg):
        try:
            from sensor_msgs_py.point_cloud2 import read_points_numpy
            pts = read_points_numpy(msg, field_names=('x','y','z'), skip_nans=True)
            pts = np.asarray(pts).reshape(-1, 3)
        except Exception:
            return
        if pts.size == 0:
            self.occupied = set()
            return
        keys = np.floor(pts / self.voxel).astype(np.int32)
        self.occupied = set(map(tuple, keys.tolist()))

    def tick(self):
        if self.current_pos is None:
            return
        getattr(self, f'do_{self.state.lower()}')()

    def do_takeoff(self):
        now = self.get_clock().now()
        if self.takeoff_sent_t is None:
            self.get_logger().info("Sending takeoff...")
            self.pub_takeoff.publish(Empty())
            self.takeoff_sent_t = now
        elapsed = (now - self.takeoff_sent_t).nanoseconds * 1e-9
        if self.current_pos[2] >= self.takeoff_alt or elapsed > 8.0:
            self.get_logger().info(f"Hovering at z={self.current_pos[2]:.2f}m.")
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return
        if self.current_pos[2] >= self.takeoff_alt * 0.9:
            self.pub_cmd.publish(Twist())

    def do_hover(self):
        self.pub_cmd.publish(Twist())
        if self.current_goal is not None:
            if float(np.linalg.norm(self.current_goal - self.current_pos)) > self.reach_tol:
                self.state = self.PLAN

    def do_plan(self):
        if self.current_goal is None:
            self.state = self.HOVER
            return
        path = self.astar(self.current_pos, self.current_goal)
        if not path or len(path) < 2:
            self.get_logger().warn("A* failed. Direct flight fallback.")
            path = [self.current_pos.copy(), self.current_goal.copy()]
        self.path = path
        self.path_idx = 0
        self.publish_path(path)
        self.last_replan_t = self.get_clock().now()
        self.state = self.FOLLOW
        self.get_logger().info(f"A* plan: {len(path)} waypoints.")

    def do_follow(self):
        now = self.get_clock().now()
        if self.last_replan_t and (now - self.last_replan_t).nanoseconds * 1e-9 > self.replan_T:
            self.pub_cmd.publish(Twist())
            self.state = self.PLAN
            return

        if float(np.linalg.norm(self.current_goal - self.current_pos)) < self.reach_tol:
            self.get_logger().info("Goal reached.")
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return

        if self.path_idx >= len(self.path):
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return

        while self.path_idx < len(self.path) - 1:
            wp = self.path[self.path_idx]
            if float(np.linalg.norm(wp - self.current_pos)) < self.reach_tol:
                self.path_idx += 1
            else:
                break

        target = self.path[self.path_idx]
        for i in range(self.path_idx, len(self.path)):
            if float(np.linalg.norm(self.path[i] - self.current_pos)) <= self.lookahead:
                target = self.path[i]
            else:
                break

        direction = target - self.current_pos
        dist = float(np.linalg.norm(direction))
        if dist < 0.05:
            self.path_idx += 1
            return
        direction = direction / dist
        final_d = float(np.linalg.norm(self.path[-1] - self.current_pos))
        speed = min(self.cruise, max(final_d * 0.7, 0.1))

        cmd = Twist()
        cmd.linear.x = float(direction[0] * speed)
        cmd.linear.y = float(direction[1] * speed)
        cmd.linear.z = float(direction[2] * speed)
        self.pub_cmd.publish(cmd)

    def astar(self, start_w, goal_w):
        start = quantize(start_w, self.voxel)
        goal  = quantize(goal_w, self.voxel)
        if self.is_occ(start) or self.is_occ(goal):
            for off in self.nbrs:
                gnb = (goal[0]+off[0], goal[1]+off[1], goal[2]+off[2])
                if not self.is_occ(gnb) and self.in_bbox(self.center(gnb)):
                    goal = gnb
                    break
            else:
                return None

        def h(v):
            return np.linalg.norm(np.array(v) - np.array(goal)) * self.voxel

        open_set = []
        heapq.heappush(open_set, (h(start), 0, start))
        came_from = {}
        g_score = {start: 0.0}
        counter = 0

        while open_set and counter < self.max_iter:
            counter += 1
            _, _, cur = heapq.heappop(open_set)
            if np.linalg.norm(np.array(cur) - np.array(goal)) <= 1.5:
                pv = [cur]
                while cur in came_from:
                    cur = came_from[cur]
                    pv.append(cur)
                pv.reverse()
                return [self.center(v) for v in pv]
            for off in self.nbrs:
                nb = (cur[0]+off[0], cur[1]+off[1], cur[2]+off[2])
                if not self.in_bbox(self.center(nb)) or self.is_occ(nb):
                    continue
                edge = math.sqrt(off[0]**2 + off[1]**2 + off[2]**2) * self.voxel
                tentative = g_score[cur] + edge
                if tentative < g_score.get(nb, float('inf')):
                    came_from[nb] = cur
                    g_score[nb] = tentative
                    heapq.heappush(open_set, (tentative + h(nb), counter, nb))
        return None

    def is_occ(self, v):
        if v in self.occupied:
            return True
        if self.inflation > 0:
            R = self.inflation
            for dx in range(-R, R+1):
                for dy in range(-R, R+1):
                    if (v[0]+dx, v[1]+dy, v[2]) in self.occupied:
                        return True
        return False

    def in_bbox(self, p):
        return (self.bbox[0] <= p[0] <= self.bbox[1] and
                self.bbox[2] <= p[1] <= self.bbox[3] and
                self.bbox[4] <= p[2] <= self.bbox[5])

    def center(self, idx):
        return np.array([(idx[0]+0.5)*self.voxel, (idx[1]+0.5)*self.voxel, (idx[2]+0.5)*self.voxel])

    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for wp in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = \
                float(wp[0]), float(wp[1]), float(wp[2])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Navigator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
