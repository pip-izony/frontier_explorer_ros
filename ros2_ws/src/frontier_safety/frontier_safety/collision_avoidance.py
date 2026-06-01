#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import Marker


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Return an (N, 3) float32 array of XYZ points, robust across ROS2 distros."""
    try:
        from sensor_msgs_py.point_cloud2 import read_points_numpy
        arr = read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        return np.asarray(arr, dtype=np.float32).reshape(-1, 3)
    except Exception:
        pass
    gen = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    arr = np.array(list(gen))
    if arr.size == 0:
        return np.empty((0, 3), dtype=np.float32)
    if arr.dtype.names is not None:
        return np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float32)
    return arr.astype(np.float32).reshape(-1, 3)


def circular_gaussian_kernel(sigma_bins: float, n_sectors: int):
    """Normalized 1D gaussian kernel + its integer offsets, for circular blur."""
    sigma_bins = max(sigma_bins, 1e-3)
    radius = max(1, int(round(3.0 * sigma_bins)))
    radius = min(radius, n_sectors // 2)
    offsets = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * (offsets / sigma_bins) ** 2)
    kernel /= kernel.sum()
    return kernel, offsets


def circular_blur(values: np.ndarray, kernel: np.ndarray, offsets: np.ndarray) -> np.ndarray:
    """Convolve a periodic (wrap-around) 1D signal with a gaussian kernel."""
    out = np.zeros_like(values)
    for k, off in zip(kernel, offsets):
        out += k * np.roll(values, int(off))
    return out


class CollisionAvoidance(Node):
    def __init__(self):
        super().__init__('collision_avoidance')

        # ---- Topics / frames ----
        self.declare_parameter('cloud_topic', '/simple_drone/velodyne_points')
        self.declare_parameter('input_cmd',   '/simple_drone/cmd_vel_raw')
        self.declare_parameter('output_cmd',  '/simple_drone/cmd_vel')
        self.declare_parameter('cloud_frame', 'velodyne_link')

        # ---- Geometry / behaviour ----
        self.declare_parameter('influence_radius',   3.0)   # m: start reacting
        self.declare_parameter('emergency_distance', 0.8)   # m: hard veto
        self.declare_parameter('self_radius',        0.25)  # m: ignore body / near noise
        self.declare_parameter('repulsion_gain',     0.8)   # push strength
        self.declare_parameter('max_linear_speed',   1.0)   # m/s output clamp
        self.declare_parameter('control_rate',       20.0)  # Hz
        self.declare_parameter('cmd_timeout',        0.5)   # s: stop if command goes stale
        self.declare_parameter('avoid_vertical',     True)  # add a small Z push too

        # ---- Gaussian polar-histogram (VFH) ----
        self.declare_parameter('histogram_sectors',  72)    # 72 -> 5 deg per bin
        self.declare_parameter('gaussian_sigma_deg', 10.0)  # blur width in degrees
        self.declare_parameter('smoothing_alpha',    0.4)   # output EMA (1.0 = off)

        gp = lambda n: self.get_parameter(n).value
        self.influence = float(gp('influence_radius'))
        self.emergency = float(gp('emergency_distance'))
        self.self_r    = float(gp('self_radius'))
        self.gain      = float(gp('repulsion_gain'))
        self.max_speed = float(gp('max_linear_speed'))
        self.timeout   = float(gp('cmd_timeout'))
        self.avoid_z   = bool(gp('avoid_vertical'))
        self.frame     = gp('cloud_frame')
        rate           = float(gp('control_rate'))

        self.n_sectors = int(gp('histogram_sectors'))
        sigma_deg      = float(gp('gaussian_sigma_deg'))
        self.alpha     = float(gp('smoothing_alpha'))

        sigma_bins = (sigma_deg / 360.0) * self.n_sectors
        self.kernel, self.offsets = circular_gaussian_kernel(sigma_bins, self.n_sectors)
        # Direction unit-vectors for each sector center (in the body XY plane).
        centers = (-math.pi) + (np.arange(self.n_sectors) + 0.5) * (2 * math.pi / self.n_sectors)
        self.sector_cos = np.cos(centers).astype(np.float32)
        self.sector_sin = np.sin(centers).astype(np.float32)

        # ---- State updated by the cloud callback ----
        self.repulsion   = np.zeros(3, dtype=np.float32)
        self.nearest_d   = math.inf
        self.nearest_dir = np.zeros(3, dtype=np.float32)
        self.desired     = Twist()
        self.last_cmd_t  = self.get_clock().now()
        self.prev_out    = np.zeros(3, dtype=np.float32)

        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(PointCloud2, gp('cloud_topic'), self.on_cloud, cloud_qos)
        self.create_subscription(Twist, gp('input_cmd'), self.on_cmd, 10)

        self.pub_cmd     = self.create_publisher(Twist, gp('output_cmd'), 10)
        self.pub_nearest = self.create_publisher(Float32, '~/nearest_obstacle', 10)
        self.pub_estop   = self.create_publisher(Bool, '~/emergency_stop', 10)
        self.pub_marker  = self.create_publisher(Marker, '~/avoidance_vector', 10)

        self.create_timer(1.0 / rate, self.control_loop)
        self.get_logger().info(
            f"Collision avoidance up. sectors={self.n_sectors}, "
            f"gaussian_sigma={sigma_deg} deg, influence={self.influence} m, "
            f"emergency={self.emergency} m. in='{gp('input_cmd')}' out='{gp('output_cmd')}'."
        )

    # ------------------------------------------------------------------ #
    def on_cmd(self, msg: Twist):
        self.desired = msg
        self.last_cmd_t = self.get_clock().now()

    # ------------------------------------------------------------------ #
    def on_cloud(self, msg: PointCloud2):
        xyz = cloud_to_xyz(msg)
        if xyz.shape[0] == 0:
            self.repulsion = np.zeros(3, dtype=np.float32)
            self.nearest_d = math.inf
            return

        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        d3 = np.linalg.norm(xyz, axis=1)
        keep = (d3 > self.self_r) & (d3 < self.influence)
        xyz, d3 = xyz[keep], d3[keep]
        if xyz.shape[0] == 0:
            self.repulsion = np.zeros(3, dtype=np.float32)
            self.nearest_d = math.inf
            return

        # Nearest obstacle (3D) for the emergency veto.
        i_min = int(np.argmin(d3))
        self.nearest_d = float(d3[i_min])
        self.nearest_dir = (xyz[i_min] / d3[i_min]).astype(np.float32)

        # --- Build the polar histogram of CLOSEST distance per sector ---
        x, y = xyz[:, 0], xyz[:, 1]
        ang = np.arctan2(y, x)                       # [-pi, pi]
        dist_xy = np.hypot(x, y)
        valid = dist_xy > 1e-3
        ang, dist_xy = ang[valid], dist_xy[valid]

        sector_min = np.full(self.n_sectors, np.inf, dtype=np.float32)
        if ang.size > 0:
            idx = ((ang + math.pi) / (2 * math.pi) * self.n_sectors).astype(np.int32)
            idx = np.clip(idx, 0, self.n_sectors - 1)
            np.minimum.at(sector_min, idx, dist_xy)

        # Proximity (potential-field weight): high when close, 0 at the boundary.
        proximity = np.where(
            np.isfinite(sector_min),
            np.clip((1.0 / np.where(sector_min == 0, 1e-3, sector_min)) - (1.0 / self.influence), 0.0, None),
            0.0,
        ).astype(np.float32)

        # --- THE GAUSSIAN BLUR: smooth the histogram around the circle ---
        proximity = circular_blur(proximity, self.kernel, self.offsets)

        # Repulsion = sum of away-vectors weighted by smoothed proximity.
        rx = -np.sum(proximity * self.sector_cos) * self.gain
        ry = -np.sum(proximity * self.sector_sin) * self.gain

        rz = 0.0
        if self.avoid_z:
            # Light vertical push from raw points (VLP-16 vertical FOV is only +/-15 deg).
            units_z = (xyz[:, 2] / d3)
            w = np.clip((1.0 / d3) - (1.0 / self.influence), 0.0, None)
            rz = float(-np.sum(units_z * w) * self.gain)

        self.repulsion = np.array([rx, ry, rz], dtype=np.float32)

    # ------------------------------------------------------------------ #
    def control_loop(self):
        now = self.get_clock().now()
        stale = (now - self.last_cmd_t).nanoseconds * 1e-9 > self.timeout

        if stale:
            desired_lin = np.zeros(3, dtype=np.float32)
            desired_yaw = 0.0
        else:
            desired_lin = np.array(
                [self.desired.linear.x, self.desired.linear.y, self.desired.linear.z],
                dtype=np.float32,
            )
            desired_yaw = float(self.desired.angular.z)

        emergency = self.nearest_d < self.emergency

        # Hard veto: remove any velocity heading into the nearest obstacle.
        if emergency and np.isfinite(self.nearest_d):
            toward = float(np.dot(desired_lin, self.nearest_dir))
            if toward > 0.0:
                desired_lin = desired_lin - toward * self.nearest_dir

        # Soft push from the gaussian-smoothed histogram.
        safe_lin = desired_lin + self.repulsion

        # Temporal smoothing (EMA) to remove residual jitter.
        a = float(np.clip(self.alpha, 0.0, 1.0))
        safe_lin = a * safe_lin + (1.0 - a) * self.prev_out
        self.prev_out = safe_lin

        # Clamp speed.
        speed = float(np.linalg.norm(safe_lin))
        if speed > self.max_speed:
            safe_lin = safe_lin * (self.max_speed / speed)

        out = Twist()
        out.linear.x, out.linear.y, out.linear.z = map(float, safe_lin)
        out.angular.z = 0.0 if emergency else desired_yaw
        self.pub_cmd.publish(out)

        self.pub_nearest.publish(Float32(data=float(self.nearest_d)))
        self.pub_estop.publish(Bool(data=bool(emergency)))
        self._publish_marker(self.repulsion, emergency)

    # ------------------------------------------------------------------ #
    def _publish_marker(self, vec: np.ndarray, emergency: bool):
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'collision_avoidance'
        m.id = 0
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = 0.04, 0.08, 0.10
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.0 if emergency else 0.85
        m.color.b = 0.0
        m.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(vec[0]), y=float(vec[1]), z=float(vec[2])),
        ]
        self.pub_marker.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
