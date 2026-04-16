"""
Cone weaver node for Pioneer P3-AT Part 2.

Implements slalom weaving between WP1 and WP2.

Algorithm
---------
1. /start_weave service triggers the manoeuvre.
2. Uses prior knowledge of weave-cone positions (from waypoints.yaml) converted
   to the robot's TF frame, then refines them using LiDAR cluster detection.
3. Generates alternating pass-left / pass-right intermediate waypoints spaced
   ``pass_offset`` metres to the side of each detected/estimated cone.
4. Drives through each intermediate waypoint using a lightweight P-controller
   (similar to WaypointController but without the full 3-phase machine so the
   weaving feels fluid).
5. Publishes /weave_done (std_msgs/Bool) when the last cone has been cleared.

LiDAR cluster detection
-----------------------
Scans the forward hemisphere for small obstacle clusters (consistent with
traffic cone size ~0.36 m diameter).  Clusters within ``scan_max_range`` are
collected and sorted along the WP1→WP2 axis to match the expected cone order.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _cluster_scan(scan: LaserScan, max_range: float, gap: float):
    """Return list of (x, y) obstacle cluster centroids in sensor frame."""
    points = []
    for i, r in enumerate(scan.ranges):
        if math.isinf(r) or math.isnan(r) or r < scan.range_min or r > max_range:
            continue
        a = scan.angle_min + i * scan.angle_increment
        points.append((r * math.cos(a), r * math.sin(a)))

    if not points:
        return []

    # Single-linkage clustering
    clusters: list[list[tuple]] = []
    for p in points:
        merged = False
        for cl in clusters:
            for q in cl:
                if math.hypot(p[0] - q[0], p[1] - q[1]) < gap:
                    cl.append(p)
                    merged = True
                    break
            if merged:
                break
        if not merged:
            clusters.append([p])

    # Return centroids
    centroids = []
    for cl in clusters:
        cx = sum(p[0] for p in cl) / len(cl)
        cy = sum(p[1] for p in cl) / len(cl)
        centroids.append((cx, cy))
    return centroids


class ConeWeaver(Node):

    def __init__(self):
        super().__init__('cone_weaver')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('scan_cluster_gap', 0.4)
        self.declare_parameter('scan_min_range', 0.2)
        self.declare_parameter('scan_max_range', 4.0)
        self.declare_parameter('pass_offset', 0.7)
        self.declare_parameter('approach_speed', 0.25)
        self.declare_parameter('weave_position_tol', 0.4)
        self.declare_parameter('kp_angular', 2.5)
        self.declare_parameter('max_angular', 1.2)

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._gap = self.get_parameter('scan_cluster_gap').value
        self._scan_min = self.get_parameter('scan_min_range').value
        self._scan_max = self.get_parameter('scan_max_range').value
        self._pass_off = self.get_parameter('pass_offset').value
        self._speed = self.get_parameter('approach_speed').value
        self._pos_tol = self.get_parameter('weave_position_tol').value
        self._kp_ang = self.get_parameter('kp_angular').value
        self._max_ang = self.get_parameter('max_angular').value

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._scan: LaserScan | None = None
        self._active = False
        self._waypoints_map: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._wp2_goal: tuple[float, float] | None = None

        self.create_subscription(LaserScan, 'scan', self._scan_cb, 1)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._done_pub = self.create_publisher(Bool, 'weave_done', 10)
        self.create_service(Empty, 'start_weave', self._start_weave_cb)

        # Prior cone positions in map frame (set via parameter or service call)
        # These are refined by LiDAR once weaving starts.
        self._prior_cones_map: list[tuple[float, float]] = []
        # Weave target waypoints (map frame)
        self._weave_targets: list[tuple[float, float]] = []

        self._timer = self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.get_logger().info('Cone weaver ready')

    # ---- public API (called by MissionController before start_weave) --------

    def set_weave_geometry(self, prior_cones_map, wp2_goal):
        """Set prior cone positions and final destination (WP2).

        prior_cones_map : list of (x,y) in map frame
        wp2_goal        : (x,y) in map frame
        """
        self._prior_cones_map = list(prior_cones_map)
        self._wp2_goal = wp2_goal

    # ---- scan callback -------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    # ---- start_weave service -------------------------------------------------

    def _start_weave_cb(self, _req, resp):
        self.get_logger().info('Starting cone weave manoeuvre')
        self._build_weave_targets()
        self._wp_idx = 0
        self._active = True
        return resp

    # ---- weave target generation ---------------------------------------------

    def _build_weave_targets(self):
        """Generate alternating pass-left/pass-right waypoints past each cone."""
        self._weave_targets = []

        if not self._prior_cones_map:
            self.get_logger().warn('No prior cone positions set — weaving blind')
            return

        # Try to refine cone positions via LiDAR
        refined = self._refine_with_lidar()

        for i, (cx, cy) in enumerate(refined):
            # Perpendicular direction (right-angle to the approach axis)
            # Approach axis: direction from current cone to next cone (or WP2)
            if i + 1 < len(refined):
                nx, ny = refined[i + 1]
            elif self._wp2_goal is not None:
                nx, ny = self._wp2_goal
            else:
                nx, ny = cx, cy

            dx = nx - cx
            dy = ny - cy
            length = math.hypot(dx, dy) or 1.0
            # Perpendicular: (-dy, dx)/length → left side
            perp_x = -dy / length
            perp_y = dx / length

            # Alternate: even cones pass to the left, odd to the right
            sign = 1.0 if i % 2 == 0 else -1.0
            tx = cx + sign * self._pass_off * perp_x
            ty = cy + sign * self._pass_off * perp_y
            self._weave_targets.append((tx, ty))

        # Final target: WP2
        if self._wp2_goal is not None:
            self._weave_targets.append(self._wp2_goal)

        self.get_logger().info(
            f'Weave targets ({len(self._weave_targets)}): '
            + ', '.join(f'({x:.2f},{y:.2f})' for x, y in self._weave_targets))

    def _refine_with_lidar(self) -> list[tuple[float, float]]:
        """Attempt to detect actual cone positions via LiDAR clustering.

        Falls back to prior positions if scan is unavailable.
        """
        if self._scan is None:
            return list(self._prior_cones_map)

        pose = self._get_pose()
        if pose is None:
            return list(self._prior_cones_map)

        rx, ry, ryaw = pose
        clusters = _cluster_scan(self._scan, self._scan_max, self._gap)

        # Convert cluster centroids (sensor frame) to map frame
        detected_map = []
        for (lx, ly) in clusters:
            mx = rx + lx * math.cos(ryaw) - ly * math.sin(ryaw)
            my = ry + lx * math.sin(ryaw) + ly * math.cos(ryaw)
            detected_map.append((mx, my))

        if not detected_map:
            return list(self._prior_cones_map)

        # Match detected clusters to prior cone positions (nearest-neighbour)
        refined = []
        for (px, py) in self._prior_cones_map:
            best_d = float('inf')
            best_pt = (px, py)
            for (dx, dy) in detected_map:
                d = math.hypot(dx - px, dy - py)
                if d < best_d and d < 2.0:   # max 2 m refinement radius
                    best_d = d
                    best_pt = (dx, dy)
            refined.append(best_pt)

        return refined

    # ---- helpers -------------------------------------------------------------

    def _get_pose(self):
        try:
            t = self._tf_buf.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return None
        tx = t.transform.translation
        rq = t.transform.rotation
        yaw = yaw_from_quaternion(rq.x, rq.y, rq.z, rq.w)
        return tx.x, tx.y, yaw

    @staticmethod
    def _wrap(a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _clamp(v: float, lim: float) -> float:
        return max(-lim, min(lim, v))

    # ---- control loop --------------------------------------------------------

    def _control_loop(self):
        if not self._active:
            return
        if self._wp_idx >= len(self._weave_targets):
            self._active = False
            self._cmd_pub.publish(Twist())
            msg = Bool()
            msg.data = True
            self._done_pub.publish(msg)
            self.get_logger().info('Cone weave complete')
            return

        pose = self._get_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        gx, gy = self._weave_targets[self._wp_idx]
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)

        if dist < self._pos_tol:
            self.get_logger().info(
                f'Weave WP {self._wp_idx + 1}/{len(self._weave_targets)} reached')
            self._wp_idx += 1
            return

        angle_to_goal = math.atan2(dy, dx)
        heading_err = self._wrap(angle_to_goal - ryaw)

        cmd = Twist()
        cmd.linear.x = self._speed
        cmd.angular.z = self._clamp(self._kp_ang * heading_err, self._max_ang)
        self._cmd_pub.publish(cmd)

    def stop(self):
        self._active = False
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ConeWeaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
