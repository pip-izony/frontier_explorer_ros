#!/bin/bash
set -e

WS=/root/ros2_ws
PKG="$WS/src/frontier_explorer_3d"

# Remove existing package if present
rm -rf "$PKG"
mkdir -p "$PKG/frontier_explorer_3d" "$PKG/launch" "$PKG/resource"
touch "$PKG/resource/frontier_explorer_3d"
touch "$PKG/frontier_explorer_3d/__init__.py"

# ─────────────────────────────────────────────────────────
cat > "$PKG/package.xml" << 'XMLEOF'
<?xml version="1.0"?>
<package format="3">
  <name>frontier_explorer_3d</name>
  <version>0.1.0</version>
  <description>NBV (sparse raycast) + RRT* + path follower for 3D frontier exploration.</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>MIT</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>sensor_msgs_py</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <export><build_type>ament_python</build_type></export>
</package>
XMLEOF

cat > "$PKG/setup.py" << 'SETUPEOF'
from setuptools import setup
import os
from glob import glob

package_name = 'frontier_explorer_3d'
setup(
    name=package_name, version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='you', maintainer_email='you@example.com',
    description='Frontier explorer NBV + RRT* navigator.', license='MIT',
    entry_points={'console_scripts': [
        'nbv_selector = frontier_explorer_3d.nbv_selector:main',
        'navigator    = frontier_explorer_3d.navigator:main',
    ]},
)
SETUPEOF

cat > "$PKG/setup.cfg" << 'CFGEOF'
[develop]
script_dir=$base/lib/frontier_explorer_3d
[install]
install_scripts=$base/lib/frontier_explorer_3d
CFGEOF

# ─────────────────────────────────────────────────────────
# NBV selector
cat > "$PKG/frontier_explorer_3d/nbv_selector.py" << 'NBVEOF'
#!/usr/bin/env python3
"""NBV selector using frontier_extractor's cluster centroids as anchors.

Implements proposal eqs. (3), (4) with sparse raycast (Sec 4.3):
    I(x_c, M) = | Vis(x_c, M) cap F |
    x*        = argmax_{x_c}  I(x_c, M) / d(x_t, x_c)

Pipeline:
  1) Sample k candidates around each frontier cluster centroid
  2) For each candidate, compute information gain by sparse raycast
     (48 golden-spiral directions, one frontier hit per ray)
  3) Pick the candidate that maximizes gain / distance
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray


def quantize(pts, res):
    """Convert continuous positions to integer voxel indices."""
    return np.floor(np.asarray(pts) / res).astype(np.int32)


def sample_directions(n):
    """Golden-spiral unit vectors on the sphere — used for sparse raycast (Sec 4.3)."""
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

        # Parameters
        self.declare_parameter('voxel_size',         0.1)   # OctoMap resolution
        self.declare_parameter('r_max',              8.0)   # Max raycast distance
        self.declare_parameter('n_ray_dirs',         48)    # Number of sparse rays
        self.declare_parameter('k_per_cluster',      5)     # Candidates per frontier
        self.declare_parameter('sampling_radius',    2.0)   # Candidate sphere radius
        self.declare_parameter('min_z',              0.4)
        self.declare_parameter('max_z',              1.5)
        self.declare_parameter('replan_period',      1.0)
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

        # State
        self.occupied    = set()
        self.free        = set()
        self.frontier    = set()
        self.centroids   = []
        self.current_pos = None
        self.frame_id    = 'simple_drone/odom'

        # Precompute ray directions (golden spiral on the unit sphere)
        self.directions = sample_directions(self.n_rays)
        self.n_steps    = int(self.r_max / self.res)

        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Subscriptions
        self.create_subscription(PoseArray,
            '/frontier_extractor/cluster_centroids', self.on_centroids, 10)
        self.create_subscription(PointCloud2,
            '/frontier_extractor/frontier_cloud', self.on_frontier, 10)
        self.create_subscription(PointCloud2,
            '/octomap_point_cloud_centers', self.on_occ, latched)
        self.create_subscription(MarkerArray,
            '/free_cells_vis_array', self.on_free, latched)
        # NOTE: sjtu_drone publishes gt_pose as Pose (NOT PoseStamped)
        self.create_subscription(Pose,
            '/simple_drone/gt_pose', self.on_pose, 10)

        # Publishers
        self.pub_nbv  = self.create_publisher(PoseStamped, '/next_viewpoint', 10)
        self.pub_cand = self.create_publisher(MarkerArray, '/nbv_candidates', 10)

        self.create_timer(self.replan_T, self.tick)
        self.get_logger().info(
            f"NBV up. res={self.res}m r_max={self.r_max}m rays={self.n_rays}"
        )

    # ---- Callbacks ----
    def on_pose(self, msg):
        # msg is geometry_msgs/Pose — no header
        self.current_pos = np.array([
            msg.position.x, msg.position.y, msg.position.z
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

    # ---- Main loop ----
    def tick(self):
        if self.current_pos is None or not self.centroids or not self.frontier:
            return

        # Step 1: sample k candidates around each cluster centroid
        candidates = []
        for c in self.centroids:
            for _ in range(self.k_per_c):
                r = np.random.uniform(0.8, self.sample_r)
                theta = np.random.uniform(0, 2*np.pi)
                # Bias toward horizontal (phi in [pi/3, 2pi/3])
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

        # Step 2: compute information gain for each candidate
        scores = []
        gains  = []
        for c in candidates:
            gain = self.compute_gain(c)
            gains.append(gain)
            # Step 3 score: gain / distance (eq. 4)
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
        """Sparse raycast information gain (Sec 4.3).

        For each of n_rays directions, march along the ray:
          - if cell is occupied -> ray terminates
          - if cell is a frontier -> count +1 and terminate (one hit per ray)
        """
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
                    break  # Sec 4.3: only one frontier hit per ray
        return gain

    # ---- Publishers ----
    def publish_nbv(self, p):
        msg = PoseStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        # Orient toward NBV from current position
        dx = p[0] - self.current_pos[0]
        dy = p[1] - self.current_pos[1]
        yaw = math.atan2(dy, dx)
        msg.pose.orientation.z = math.sin(yaw/2)
        msg.pose.orientation.w = math.cos(yaw/2)
        self.pub_nbv.publish(msg)

    def publish_candidates(self, cands, scores, best_idx):
        ma = MarkerArray()
        # Clear old markers
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
                # Best candidate — large green sphere
                m.scale.x = m.scale.y = m.scale.z = 0.4
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            else:
                # Other candidates — small, colored by normalized score
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
NBVEOF

# ─────────────────────────────────────────────────────────
# Navigator (RRT* + path follower)
cat > "$PKG/frontier_explorer_3d/navigator.py" << 'NAVEOF'
#!/usr/bin/env python3
"""Navigator: auto-takeoff + RRT* planner + path follower.

Subscribes to /next_viewpoint published by the NBV selector.
Plans a 3D path with RRT*, then a lookahead path follower drives the drone.

State machine: TAKEOFF -> HOVER -> PLAN -> FOLLOW -> HOVER -> ...
"""
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist, Pose, Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Empty, ColorRGBA
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray


class Navigator(Node):
    TAKEOFF = 'TAKEOFF'
    HOVER   = 'HOVER'
    PLAN    = 'PLAN'
    FOLLOW  = 'FOLLOW'

    def __init__(self):
        super().__init__('navigator')

        # ---- Parameters ----
        # Topics
        self.declare_parameter('cmd_vel_topic',     '/simple_drone/cmd_vel')
        # Flight
        self.declare_parameter('takeoff_altitude',  0.6)
        self.declare_parameter('cruise_speed',      0.3)
        self.declare_parameter('reach_tol',         0.5)
        self.declare_parameter('lookahead',         1.2)
        self.declare_parameter('replan_period',     5.0)
        # Workspace bounding box [xmin, xmax, ymin, ymax, zmin, zmax]
        self.declare_parameter('bbox',              [-6.5, 6.5, -6.5, 6.5, 0.1, 1.5])
        # RRT*
        self.declare_parameter('voxel_size',        0.2)
        self.declare_parameter('safety_inflation',  1)
        self.declare_parameter('max_iter',          1500)
        self.declare_parameter('step_size',         0.5)
        self.declare_parameter('search_radius',     1.5)
        self.declare_parameter('goal_bias',         0.1)
        self.declare_parameter('goal_tol',          0.6)

        g = lambda n: self.get_parameter(n).value
        self.cmd_topic    = g('cmd_vel_topic')
        self.takeoff_alt  = float(g('takeoff_altitude'))
        self.cruise       = float(g('cruise_speed'))
        self.reach_tol    = float(g('reach_tol'))
        self.lookahead    = float(g('lookahead'))
        self.replan_T     = float(g('replan_period'))
        self.bbox         = list(g('bbox'))
        self.voxel        = float(g('voxel_size'))
        self.inflation    = int(g('safety_inflation'))
        self.max_iter     = int(g('max_iter'))
        self.step_size    = float(g('step_size'))
        self.search_r     = float(g('search_radius'))
        self.goal_bias    = float(g('goal_bias'))
        self.goal_tol_rrt = float(g('goal_tol'))

        # ---- State ----
        self.state          = self.TAKEOFF
        self.current_pos    = None
        self.frame_id       = 'simple_drone/odom'
        self.occupied       = set()
        self.current_goal   = None
        self.path           = []
        self.path_idx       = 0
        self.takeoff_sent_t = None
        self.last_replan_t  = None

        # ---- ROS interfaces ----
        latched = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        # NOTE: sjtu_drone publishes gt_pose as Pose (NOT PoseStamped)
        self.create_subscription(Pose, '/simple_drone/gt_pose', self.on_pose, 10)
        self.create_subscription(PoseStamped, '/next_viewpoint', self.on_goal, 10)
        self.create_subscription(PointCloud2,
            '/octomap_point_cloud_centers', self.on_occ, latched)

        self.pub_takeoff = self.create_publisher(Empty, '/simple_drone/takeoff', 10)
        self.pub_cmd     = self.create_publisher(Twist, self.cmd_topic, 10)
        self.pub_path    = self.create_publisher(Path, '/planned_path', 10)
        self.pub_tree    = self.create_publisher(MarkerArray, '/rrt_tree', 10)

        # 20 Hz control loop
        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f"Navigator up. Publishing cmd_vel to '{self.cmd_topic}'."
        )

    # ---- Callbacks ----
    def on_pose(self, msg):
        # msg is geometry_msgs/Pose — no header
        self.current_pos = np.array([
            msg.position.x, msg.position.y, msg.position.z
        ], dtype=np.float64)

    def on_goal(self, msg):
        self.frame_id = msg.header.frame_id or self.frame_id
        self.current_goal = np.array([
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ], dtype=np.float64)
        # If hovering or following, replan toward the new goal
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

    # ---- State machine dispatcher ----
    def tick(self):
        if self.current_pos is None:
            return
        getattr(self, f'do_{self.state.lower()}')()

    # ---- TAKEOFF ----
    def do_takeoff(self):
        now = self.get_clock().now()
        # Send /takeoff once at the very start
        if self.takeoff_sent_t is None:
            self.get_logger().info("Sending takeoff...")
            self.pub_takeoff.publish(Empty())
            self.takeoff_sent_t = now

        elapsed = (now - self.takeoff_sent_t).nanoseconds * 1e-9

        # Reached altitude or timeout — switch to HOVER
        if self.current_pos[2] >= self.takeoff_alt or elapsed > 12.0:
            self.get_logger().info(f"Hovering at z={self.current_pos[2]:.2f}m.")
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return

        # Actively command upward velocity until target altitude is reached
        cmd = Twist()
        cmd.linear.z = 0.5
        self.pub_cmd.publish(cmd)

    # ---- HOVER ----
    def do_hover(self):
        # Continuously publish zero velocity to hold position
        self.pub_cmd.publish(Twist())
        # If a goal arrived and we're not already there, switch to PLAN
        if self.current_goal is not None:
            if float(np.linalg.norm(self.current_goal - self.current_pos)) > self.reach_tol:
                self.state = self.PLAN

    # ---- PLAN (RRT*) ----
    def do_plan(self):
        if self.current_goal is None:
            self.state = self.HOVER
            return

        # Clamp start position into the workspace bbox
        # (RRT* requires the start to be inside the bbox)
        start = self.current_pos.copy()
        start[0] = max(self.bbox[0] + 0.05, min(self.bbox[1] - 0.05, start[0]))
        start[1] = max(self.bbox[2] + 0.05, min(self.bbox[3] - 0.05, start[1]))
        start[2] = max(self.bbox[4] + 0.05, min(self.bbox[5] - 0.05, start[2]))

        # Run RRT*
        nodes, parent, path = self.rrt_star(start, self.current_goal)

        # Publish tree for visualization
        self.publish_tree(nodes, parent)

        # Fallback to direct flight if RRT* fails
        if not path or len(path) < 2:
            self.get_logger().warn(
                f"RRT* failed after {len(nodes)} nodes. Direct flight fallback."
            )
            path = [self.current_pos.copy(), self.current_goal.copy()]
        else:
            self.get_logger().info(
                f"RRT* plan: {len(path)} waypoints, {len(nodes)} tree nodes."
            )

        self.path = path
        self.path_idx = 0
        self.publish_path(path)
        self.last_replan_t = self.get_clock().now()
        self.state = self.FOLLOW

    # ---- FOLLOW ----
    def do_follow(self):
        now = self.get_clock().now()
        # Periodic replanning
        if self.last_replan_t and (now - self.last_replan_t).nanoseconds * 1e-9 > self.replan_T:
            self.pub_cmd.publish(Twist())
            self.state = self.PLAN
            return

        # Final goal reached
        if float(np.linalg.norm(self.current_goal - self.current_pos)) < self.reach_tol:
            self.get_logger().info("Goal reached.")
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return

        if self.path_idx >= len(self.path):
            self.pub_cmd.publish(Twist())
            self.state = self.HOVER
            return

        # Advance waypoint index while waypoints are already close
        while self.path_idx < len(self.path) - 1:
            wp = self.path[self.path_idx]
            if float(np.linalg.norm(wp - self.current_pos)) < self.reach_tol:
                self.path_idx += 1
            else:
                break

        # Lookahead target — pick farthest waypoint within lookahead radius
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

        # Taper speed near final goal
        direction = direction / dist
        final_d = float(np.linalg.norm(self.path[-1] - self.current_pos))
        speed = min(self.cruise, max(final_d * 0.7, 0.1))

        cmd = Twist()
        cmd.linear.x = float(direction[0] * speed)
        cmd.linear.y = float(direction[1] * speed)
        cmd.linear.z = float(direction[2] * speed)
        self.pub_cmd.publish(cmd)

    # ─────────────────────────────────────────────────
    # RRT* algorithm
    # ─────────────────────────────────────────────────
    def rrt_star(self, start_w, goal_w):
        """RRT* in 3D voxel-collision space.

        Returns: (nodes, parent_dict, path) where path is None if planning failed.

        Key RRT* steps:
          1) Sample x_rand (with goal bias)
          2) Find x_nearest in tree
          3) Steer: x_new = x_nearest + step_size * direction
          4) Choose parent: pick lowest-cost parent within search_radius
          5) Rewire: re-link neighbors through x_new if cheaper
        """
        nodes  = [np.asarray(start_w, dtype=np.float64)]
        parent = {0: None}
        cost   = {0: 0.0}
        goal   = np.asarray(goal_w, dtype=np.float64)

        for it in range(self.max_iter):
            # Step 1: random sample (goal bias 10%)
            if np.random.random() < self.goal_bias:
                x_rand = goal.copy()
            else:
                x_rand = np.array([
                    np.random.uniform(self.bbox[0], self.bbox[1]),
                    np.random.uniform(self.bbox[2], self.bbox[3]),
                    np.random.uniform(self.bbox[4], self.bbox[5]),
                ])

            # Step 2: nearest tree node
            arr = np.array(nodes)
            dvec = np.linalg.norm(arr - x_rand, axis=1)
            idx_nearest = int(np.argmin(dvec))
            x_nearest = nodes[idx_nearest]

            # Step 3: steer
            direction = x_rand - x_nearest
            d = np.linalg.norm(direction)
            if d < 1e-6:
                continue
            if d > self.step_size:
                x_new = x_nearest + (direction / d) * self.step_size
            else:
                x_new = x_rand.copy()

            # Collision checks
            if not self.in_bbox(x_new):
                continue
            if not self.is_segment_free(x_nearest, x_new):
                continue

            # Step 4: choose parent — lowest-cost reachable parent within search_radius
            dvec_new = np.linalg.norm(arr - x_new, axis=1)
            idx_near = np.where(dvec_new < self.search_r)[0].tolist()

            idx_min = idx_nearest
            c_min = cost[idx_nearest] + np.linalg.norm(x_nearest - x_new)
            for i in idx_near:
                c = cost[i] + np.linalg.norm(nodes[i] - x_new)
                if c < c_min and self.is_segment_free(nodes[i], x_new):
                    idx_min = i
                    c_min = c

            # Add x_new to the tree
            idx_new = len(nodes)
            nodes.append(x_new)
            parent[idx_new] = idx_min
            cost[idx_new] = c_min

            # Step 5: rewire — re-link near nodes through x_new if cheaper
            for i in idx_near:
                if i == idx_min:
                    continue
                c_through_new = cost[idx_new] + np.linalg.norm(x_new - nodes[i])
                if c_through_new < cost[i] and self.is_segment_free(x_new, nodes[i]):
                    parent[i] = idx_new
                    cost[i] = c_through_new

            # Goal reached?
            if np.linalg.norm(x_new - goal) < self.goal_tol_rrt:
                # Reconstruct path from goal back to start
                path = [goal.copy(), x_new.copy()]
                idx = parent[idx_new]
                while idx is not None:
                    path.append(nodes[idx])
                    idx = parent[idx]
                path.reverse()
                return nodes, parent, path

        return nodes, parent, None

    # ---- Collision helpers ----
    def is_segment_free(self, p1, p2):
        """Check if the straight segment p1->p2 is collision-free.

        We skip the occupancy check at i==0 because the start of every segment
        is by definition a valid tree node (the drone or an existing waypoint).
        """
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
        """Voxel occupancy check with forced inflation (collision margin)."""
        if v in self.occupied:
            return True
        # Inflate: also block cells within radius R in 3D
        R = max(self.inflation, 1)
        for dx in range(-R, R + 1):
            for dy in range(-R, R + 1):
                for dz in range(-R, R + 1):
                    if (dx, dy, dz) == (0, 0, 0):
                        continue
                    if (v[0]+dx, v[1]+dy, v[2]+dz) in self.occupied:
                        return True
        return False

    def in_bbox(self, p):
        return (self.bbox[0] <= p[0] <= self.bbox[1] and
                self.bbox[2] <= p[1] <= self.bbox[3] and
                self.bbox[4] <= p[2] <= self.bbox[5])

    # ---- Visualization ----
    def publish_path(self, path):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for wp in path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(wp[0])
            ps.pose.position.y = float(wp[1])
            ps.pose.position.z = float(wp[2])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def publish_tree(self, nodes, parent):
        """Publish the RRT* tree as a LINE_LIST marker for RViz."""
        ma = MarkerArray()
        # Clear previous tree
        clr = Marker()
        clr.header.frame_id = self.frame_id
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
            p1 = nodes[idx_par]
            p2 = nodes[idx_child]
            edges.points.append(Point(x=float(p1[0]), y=float(p1[1]), z=float(p1[2])))
            edges.points.append(Point(x=float(p2[0]), y=float(p2[1]), z=float(p2[2])))
        ma.markers.append(edges)
        self.pub_tree.publish(ma)


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
NAVEOF

# ─────────────────────────────────────────────────────────
# Combined launch (NBV + RRT* Navigator)
cat > "$PKG/launch/explore.launch.py" << 'LAUNCHEOF'
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # NBV: picks the next viewpoint by sparse-raycast information gain
        Node(
            package='frontier_explorer_3d', executable='nbv_selector',
            name='nbv_selector', output='screen',
            parameters=[{'use_sim_time': False}],
        ),
        # Navigator: auto-takeoff + RRT* + path follower
        Node(
            package='frontier_explorer_3d', executable='navigator',
            name='navigator', output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])
LAUNCHEOF

cd "$WS"
colcon build --symlink-install --packages-select frontier_explorer_3d

cat << 'DONE'
================================================================
>> Build complete!
================================================================

Launch order (each in its own terminal inside the container):

  T1: ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py

  T2: ros2 run tf2_ros static_transform_publisher \
        0 0 0.10 0 0 0 simple_drone/base_footprint velodyne_link

  T3: ros2 run octomap_server octomap_server_node --ros-args \
        -r cloud_in:=/simple_drone/velodyne_points \
        -p frame_id:=simple_drone/odom -p resolution:=0.1 \
        -p sensor_model.max_range:=25.0 -p use_sim_time:=false \
        -p publish_free_space:=true

  T4: ros2 run frontier_explorer_py frontier_extractor

  T5 (NBV + RRT* Navigator):
      pkill -9 -f teleop 2>/dev/null
      ros2 launch frontier_explorer_3d explore.launch.py

RViz topics to add:
  /occupied_cells_vis_array          MarkerArray   (OctoMap voxels)
  /frontier_extractor/cluster_markers MarkerArray  (frontier clusters)
  /nbv_candidates                    MarkerArray   (NBV candidates; green = best)
  /next_viewpoint                    Pose          (current NBV target)
  /rrt_tree                          MarkerArray   (RRT* tree edges)
  /planned_path                      Path          (RRT* final path)

Fixed Frame: simple_drone/odom
DONE
