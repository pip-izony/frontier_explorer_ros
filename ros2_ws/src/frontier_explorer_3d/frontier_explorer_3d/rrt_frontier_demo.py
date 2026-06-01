#!/usr/bin/env python3
"""RRT* frontier demo with auto-takeoff and path following.

State machine: TAKEOFF -> HOVER -> FOLLOW -> HOVER -> ...
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Twist
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Empty


class RRTStarFrontier(Node):
    TAKEOFF = 'TAKEOFF'
    HOVER   = 'HOVER'
    FOLLOW  = 'FOLLOW'

    def __init__(self):
        super().__init__('rrt_frontier_demo')

        # RRT* params
        self.declare_parameter('voxel_size',    0.2)
        self.declare_parameter('bbox',          [-7.0, 7.0, -7.0, 7.0, 0.0, 1.8])
        self.declare_parameter('max_iter',      1500)
        self.declare_parameter('step_size',     0.5)
        self.declare_parameter('search_radius', 1.5)
        self.declare_parameter('goal_bias',     0.1)
        self.declare_parameter('goal_tol',      0.6)
        self.declare_parameter('replan_period', 4.0)
        self.declare_parameter('safety_inflation', 0)
        # Flight params
        self.declare_parameter('takeoff_altitude', 0.3)
        self.declare_parameter('cruise_speed',     0.3)
        self.declare_parameter('reach_tol',        0.5)
        self.declare_parameter('lookahead',        1.0)
        self.declare_parameter('follow_path',      True)  # False면 호버만

        g = lambda n: self.get_parameter(n).value
        self.voxel        = float(g('voxel_size'))
        self.bbox         = list(g('bbox'))
        self.max_iter     = int(g('max_iter'))
        self.step_size    = float(g('step_size'))
        self.search_r     = float(g('search_radius'))
        self.goal_bias    = float(g('goal_bias'))
        self.goal_tol     = float(g('goal_tol'))
        self.replan_T     = float(g('replan_period'))
        self.inflation    = int(g('safety_inflation'))
        self.takeoff_alt  = float(g('takeoff_altitude'))
        self.cruise       = float(g('cruise_speed'))
        self.reach_tol    = float(g('reach_tol'))
        self.lookahead    = float(g('lookahead'))
        self.do_follow    = bool(g('follow_path'))

        # State
        self.state = self.TAKEOFF
        self.current_pos = None
        self.centroids = []
        self.occupied  = set()
        self.frame_id  = 'simple_drone/odom'
        self.current_path = None
        self.path_idx = 0
        self.takeoff_sent_t = None

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscribers
        self.create_subscription(Pose,
            '/simple_drone/gt_pose', self.on_pose, 10)
        self.create_subscription(PoseArray,
            '/frontier_extractor/cluster_centroids', self.on_centroids, 10)
        self.create_subscription(PointCloud2,
            '/octomap_point_cloud_centers', self.on_occ, latched)

        # Publishers
        self.pub_path    = self.create_publisher(Path,         '/rrt_path',  10)
        self.pub_tree    = self.create_publisher(MarkerArray,  '/rrt_tree',  10)
        self.pub_goal    = self.create_publisher(Marker,       '/rrt_goal',  10)
        self.pub_start   = self.create_publisher(Marker,       '/rrt_start', 10)
        self.pub_cmd     = self.create_publisher(Twist,        '/simple_drone/cmd_vel', 10)
        self.pub_takeoff = self.create_publisher(Empty,        '/simple_drone/takeoff', 10)

        # 20Hz: takeoff/hover/follow control
        self.create_timer(0.05, self.control_tick)
        # Slow: RRT* replanning
        self.create_timer(self.replan_T, self.planning_tick)

        self.get_logger().info(
            f"RRT* demo up. takeoff_alt={self.takeoff_alt}m, follow={self.do_follow}"
        )

    # ---- callbacks ----
    def on_pose(self, msg):
        self.current_pos = np.array([msg.position.x, msg.position.y, msg.position.z],
                                    dtype=np.float64)

    def on_centroids(self, msg):
        self.frame_id = msg.header.frame_id or self.frame_id
        self.centroids = [np.array([p.position.x, p.position.y, p.position.z])
                          for p in msg.poses]

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

    # ---- 20Hz control loop ----
    def control_tick(self):
        if self.current_pos is None:
            return

        if self.state == self.TAKEOFF:
            self._do_takeoff()
        elif self.state == self.HOVER:
            self.pub_cmd.publish(Twist())
        elif self.state == self.FOLLOW:
            self._do_follow()

    def _do_takeoff(self):
        now = self.get_clock().now()
        if self.takeoff_sent_t is None:
            self.get_logger().info("Sending takeoff...")
            self.pub_takeoff.publish(Empty())
            self.takeoff_sent_t = now

        elapsed = (now - self.takeoff_sent_t).nanoseconds * 1e-9
        if self.current_pos[2] >= self.takeoff_alt or elapsed > 12.0:
            self.get_logger().info(f"Reached altitude {self.current_pos[2]:.2f}m. Hovering.")
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return

        cmd = Twist()
        cmd.linear.z = 0.5
        self.pub_cmd.publish(cmd)

    def _do_follow(self):
        if self.current_path is None or self.path_idx >= len(self.current_path):
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return

        # Advance waypoint index
        while self.path_idx < len(self.current_path) - 1:
            wp = self.current_path[self.path_idx]
            if float(np.linalg.norm(wp - self.current_pos)) < self.reach_tol:
                self.path_idx += 1
            else:
                break

        # Lookahead target
        target = self.current_path[self.path_idx]
        for i in range(self.path_idx, len(self.current_path)):
            if float(np.linalg.norm(self.current_path[i] - self.current_pos)) <= self.lookahead:
                target = self.current_path[i]
            else:
                break

        direction = target - self.current_pos
        dist = float(np.linalg.norm(direction))
        if dist < 0.05:
            self.path_idx += 1
            return

        # 목표 도달?
        final = self.current_path[-1]
        if float(np.linalg.norm(final - self.current_pos)) < self.reach_tol:
            self.get_logger().info("Goal reached!")
            self.pub_cmd.publish(Twist())
            self.current_path = None
            self.state = self.HOVER
            return

        direction = direction / dist
        cmd = Twist()
        cmd.linear.x = float(direction[0] * self.cruise)
        cmd.linear.y = float(direction[1] * self.cruise)
        cmd.linear.z = float(direction[2] * self.cruise)
        self.pub_cmd.publish(cmd)

    # ---- Slow planning loop ----
    def planning_tick(self):
        if self.current_pos is None:
            return
        if self.state == self.TAKEOFF:
            self.get_logger().info("Still taking off, skip plan.",
                                   throttle_duration_sec=4.0)
            return
        if not self.centroids:
            self.get_logger().warn("No frontiers yet...", throttle_duration_sec=4.0)
            return

        # 가장 가까운 frontier
        dists = [np.linalg.norm(c - self.current_pos) for c in self.centroids]
        idx = int(np.argmin(dists))
        goal = self.centroids[idx].copy()
        # Goal을 takeoff_alt 근처로 (frontier가 너무 낮을 수 있으니)
        goal[2] = max(self.bbox[4] + 0.3, min(self.bbox[5] - 0.1, goal[2]))

        # Start clamp
        start = self.current_pos.copy()
        start[0] = max(self.bbox[0] + 0.05, min(self.bbox[1] - 0.05, start[0]))
        start[1] = max(self.bbox[2] + 0.05, min(self.bbox[3] - 0.05, start[1]))
        start[2] = max(self.bbox[4] + 0.05, min(self.bbox[5] - 0.05, start[2]))

        self.get_logger().info(
            f"Target: ({goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}) "
            f"dist={dists[idx]:.2f}m ({len(self.centroids)} frontiers)"
        )

        nodes, parent, path = self.rrt_star(start, goal)

        self.publish_start(start)
        self.publish_goal(goal, found=path is not None)
        self.publish_tree(nodes, parent)
        if path is not None:
            self.publish_path(path)
            self.get_logger().info(
                f"RRT* SUCCESS: {len(path)} waypoints, {len(nodes)} tree nodes."
            )
            if self.do_follow and self.state == self.HOVER:
                self.current_path = path
                self.path_idx = 0
                self.state = self.FOLLOW
                self.get_logger().info("→ FOLLOW")
        else:
            self.get_logger().warn(f"RRT* failed after {len(nodes)} nodes.")

    # ---- RRT* ----
    def rrt_star(self, start_w, goal_w):
        nodes  = [np.asarray(start_w, dtype=np.float64)]
        parent = {0: None}
        cost   = {0: 0.0}
        goal   = np.asarray(goal_w, dtype=np.float64)

        for it in range(self.max_iter):
            if np.random.random() < self.goal_bias:
                x_rand = goal.copy()
            else:
                x_rand = np.array([
                    np.random.uniform(self.bbox[0], self.bbox[1]),
                    np.random.uniform(self.bbox[2], self.bbox[3]),
                    np.random.uniform(self.bbox[4], self.bbox[5]),
                ])

            arr = np.array(nodes)
            dvec = np.linalg.norm(arr - x_rand, axis=1)
            idx_nearest = int(np.argmin(dvec))
            x_nearest = nodes[idx_nearest]

            direction = x_rand - x_nearest
            d = np.linalg.norm(direction)
            if d < 1e-6:
                continue
            if d > self.step_size:
                x_new = x_nearest + (direction / d) * self.step_size
            else:
                x_new = x_rand.copy()

            if not self.in_bbox(x_new):
                continue
            if not self.is_segment_free(x_nearest, x_new):
                continue

            dvec_new = np.linalg.norm(arr - x_new, axis=1)
            idx_near = np.where(dvec_new < self.search_r)[0].tolist()

            idx_min = idx_nearest
            c_min = cost[idx_nearest] + np.linalg.norm(x_nearest - x_new)
            for i in idx_near:
                c = cost[i] + np.linalg.norm(nodes[i] - x_new)
                if c < c_min and self.is_segment_free(nodes[i], x_new):
                    idx_min = i
                    c_min = c

            idx_new = len(nodes)
            nodes.append(x_new)
            parent[idx_new] = idx_min
            cost[idx_new] = c_min

            for i in idx_near:
                if i == idx_min:
                    continue
                c_through_new = cost[idx_new] + np.linalg.norm(x_new - nodes[i])
                if c_through_new < cost[i] and self.is_segment_free(x_new, nodes[i]):
                    parent[i] = idx_new
                    cost[i] = c_through_new

            if np.linalg.norm(x_new - goal) < self.goal_tol:
                path = [goal.copy(), x_new.copy()]
                idx = parent[idx_new]
                while idx is not None:
                    path.append(nodes[idx])
                    idx = parent[idx]
                path.reverse()
                return nodes, parent, path

        return nodes, parent, None

    def is_segment_free(self, p1, p2):
        d = float(np.linalg.norm(p2 - p1))
        n = max(int(d / (self.voxel * 0.5)), 1)
        for i in range(n + 1):
            t = i / n
            p = p1 + t * (p2 - p1)
            if not self.in_bbox(p):
                return False
            if i == 0:
                continue
            key = tuple(int(np.floor(c / self.voxel)) for c in p)
            if self.is_occ(key):
                return False
        return True

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

    # ---- viz ----
    def publish_tree(self, nodes, parent):
        ma = MarkerArray()
        clr = Marker(); clr.header.frame_id = self.frame_id
        clr.action = Marker.DELETEALL
        ma.markers.append(clr)

        edges = Marker()
        edges.header.frame_id = self.frame_id
        edges.header.stamp = self.get_clock().now().to_msg()
        edges.ns, edges.id = 'tree', 0
        edges.type, edges.action = Marker.LINE_LIST, Marker.ADD
        edges.pose.orientation.w = 1.0
        edges.scale.x = 0.02
        edges.color = ColorRGBA(r=0.4, g=0.4, b=0.8, a=0.5)
        for idx_child, idx_par in parent.items():
            if idx_par is None:
                continue
            p1 = nodes[idx_par]; p2 = nodes[idx_child]
            edges.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
            edges.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=float(p2[2])))
        ma.markers.append(edges)
        self.pub_tree.publish(ma)

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

    def publish_goal(self, p, found=True):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'goal', 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(p[0]), float(p[1]), float(p[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.5
        if found:
            m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        self.pub_goal.publish(m)

    def publish_start(self, p):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = 'start', 0
        m.type, m.action = Marker.SPHERE, Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = \
            float(p[0]), float(p[1]), float(p[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.4
        m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)
        self.pub_start.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = RRTStarFrontier()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
