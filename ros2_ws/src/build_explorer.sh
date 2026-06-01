#!/bin/bash
set -e

WS=/root/ros2_ws
PKG="$WS/src/frontier_explorer_3d"

# 기존 폴더 있으면 삭제
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
  <description>NBV (sparse raycast) + A* + path follower.</description>
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
    description='Frontier explorer NBV + navigator.', license='MIT',
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
NBVEOF

# ─────────────────────────────────────────────────────────
# Navigator (A* + follower)
cat > "$PKG/frontier_explorer_3d/navigator.py" << 'NAVEOF'
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
NAVEOF

# ─────────────────────────────────────────────────────────
# Combined launch (NBV + Navigator)
cat > "$PKG/launch/explore.launch.py" << 'LAUNCHEOF'
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='frontier_explorer_3d', executable='nbv_selector',
            name='nbv_selector', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='frontier_explorer_3d', executable='navigator',
            name='navigator', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
LAUNCHEOF

cd "$WS"
colcon build --symlink-install --packages-select frontier_explorer_3d

cat << 'DONE'
================================================================
>> Build complete!
================================================================

  T1: ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py

  T2: ros2 run tf2_ros static_transform_publisher \
        0 0 0.10 0 0 0 simple_drone/base_footprint velodyne_link

  T3: ros2 run octomap_server octomap_server_node --ros-args \
        -r cloud_in:=/simple_drone/velodyne_points \
        -p frame_id:=simple_drone/odom -p resolution:=0.1 \
        -p sensor_model.max_range:=25.0 -p use_sim_time:=true \
        -p publish_free_space:=true

  T4: ros2 run frontier_explorer_py frontier_extractor

  T5 (★ collision_avoidance — takeoff):
      ros2 launch frontier_safety collision_avoidance.launch.py

  T6 (★ NBV + Navigator):
      pkill -9 -f teleop 2>/dev/null
      ros2 launch frontier_explorer_3d explore.launch.py

DONE
