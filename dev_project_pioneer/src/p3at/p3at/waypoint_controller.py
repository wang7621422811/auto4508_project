"""
Waypoint controller with reactive obstacle avoidance for diff-drive robots.

Subscribes to ``/scan`` (LaserScan) and checks a forward cone for obstacles.
When an obstacle is closer than *obstacle_distance*, the robot steers away
from the blocked side before resuming its drive toward the current waypoint.

State machine
-------------
IDLE → ROTATE_TO_GOAL → DRIVE (with inline avoidance) → ROTATE_TO_HEADING
     ↘ next waypoint ← ──────────────────────────────────┘
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointController(Node):

    PHASE_IDLE = 0
    PHASE_ROTATE_TO_GOAL = 1
    PHASE_DRIVE = 2
    PHASE_ROTATE_TO_HEADING = 3

    def __init__(self):
        super().__init__('waypoint_controller')

        # --- tuneable parameters ---
        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('max_linear', 0.35)
        self.declare_parameter('max_angular', 1.2)
        self.declare_parameter('position_tol', 0.15)
        self.declare_parameter('heading_tol', 0.10)
        self.declare_parameter('control_hz', 20.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        # obstacle avoidance
        self.declare_parameter('obstacle_distance', 0.7)
        self.declare_parameter('obstacle_stop_distance', 0.35)
        self.declare_parameter('scan_front_angle', 0.7)   # rad — half-width of forward cone

        self._kp_lin = self.get_parameter('kp_linear').value
        self._kp_ang = self.get_parameter('kp_angular').value
        self._max_lin = self.get_parameter('max_linear').value
        self._max_ang = self.get_parameter('max_angular').value
        self._pos_tol = self.get_parameter('position_tol').value
        self._hdg_tol = self.get_parameter('heading_tol').value
        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        hz = self.get_parameter('control_hz').value

        self._obs_dist = self.get_parameter('obstacle_distance').value
        self._obs_stop = self.get_parameter('obstacle_stop_distance').value
        self._scan_half = self.get_parameter('scan_front_angle').value

        # TF
        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # pub / sub
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._path_viz_pub = self.create_publisher(Path, 'waypoint_path', 10)

        self.create_subscription(PoseStamped, 'goal_pose', self._goal_cb, 10)
        self.create_subscription(Path, 'waypoints', self._waypoints_cb, 1)
        self.create_subscription(LaserScan, 'scan', self._scan_cb, 1)
        self.create_service(Empty, 'clear_waypoints', self._clear_cb)

        # state
        self._queue: deque[PoseStamped] = deque()
        self._current_goal: PoseStamped | None = None
        self._current_yaw = 0.0
        self._phase = self.PHASE_IDLE
        self._wp_index = 0
        self._wp_total = 0
        self._scan: LaserScan | None = None

        self._timer = self.create_timer(1.0 / hz, self._control_loop)
        self.get_logger().info(
            'Waypoint controller ready (with obstacle avoidance)')

    # ---- scan ------------------------------------------------------------

    def _scan_cb(self, msg: LaserScan):
        self._scan = msg

    def _check_obstacles(self):
        """Analyse the latest scan for obstacles in the forward cone.

        Returns (min_left, min_right) — minimum distances in the left and
        right halves of the forward cone.  ``inf`` when no obstacle detected.
        """
        scan = self._scan
        if scan is None:
            return float('inf'), float('inf')

        min_left = float('inf')
        min_right = float('inf')

        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r) or r < scan.range_min:
                continue
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle) > self._scan_half:
                continue
            if angle >= 0.0:
                min_left = min(min_left, r)
            else:
                min_right = min(min_right, r)

        return min_left, min_right

    # ---- incoming goals --------------------------------------------------

    def _goal_cb(self, msg: PoseStamped):
        self._queue.append(msg)
        q = msg.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f'[+] Waypoint enqueued: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f}), yaw={math.degrees(yaw):.1f}°  '
            f'queue={len(self._queue)}')
        if self._phase == self.PHASE_IDLE:
            self._advance()
        self._publish_path_viz()

    def _waypoints_cb(self, msg: Path):
        self._queue.clear()
        self._phase = self.PHASE_IDLE
        self._current_goal = None
        for pose in msg.poses:
            self._queue.append(pose)
        self._wp_index = 0
        self._wp_total = len(self._queue)
        self.get_logger().info(
            f'Received path with {self._wp_total} waypoints')
        if self._queue:
            self._advance()
        self._publish_path_viz()

    def _clear_cb(self, _req, resp):
        self._queue.clear()
        self._current_goal = None
        self._phase = self.PHASE_IDLE
        self._cmd_pub.publish(Twist())
        self.get_logger().info('Waypoints cleared — stopped')
        self._publish_path_viz()
        return resp

    # ---- queue management ------------------------------------------------

    def _advance(self):
        if not self._queue:
            self._phase = self.PHASE_IDLE
            self._current_goal = None
            self.get_logger().info('All waypoints completed')
            return
        self._current_goal = self._queue.popleft()
        q = self._current_goal.pose.orientation
        self._current_yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self._phase = self.PHASE_ROTATE_TO_GOAL
        self._wp_index += 1
        self.get_logger().info(
            f'[{self._wp_index}/{self._wp_total}] Navigating to '
            f'({self._current_goal.pose.position.x:.2f}, '
            f'{self._current_goal.pose.position.y:.2f})')

    # ---- visualization ---------------------------------------------------

    def _publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        if self._current_goal is not None:
            msg.poses.append(self._current_goal)
        msg.poses.extend(self._queue)
        self._path_viz_pub.publish(msg)

    # ---- helpers ---------------------------------------------------------

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
    def _wrap(angle: float) -> float:
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def _clamp(val: float, limit: float) -> float:
        return max(-limit, min(limit, val))

    # ---- control loop ----------------------------------------------------

    def _control_loop(self):
        if self._phase == self.PHASE_IDLE or self._current_goal is None:
            return

        pose = self._get_pose()
        if pose is None:
            return
        rx, ry, ryaw = pose

        gx = self._current_goal.pose.position.x
        gy = self._current_goal.pose.position.y
        dx, dy = gx - rx, gy - ry
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        heading_err = self._wrap(angle_to_goal - ryaw)
        orient_err = self._wrap(self._current_yaw - ryaw)

        cmd = Twist()

        # ------ Phase 1: rotate to face the goal ------
        if self._phase == self.PHASE_ROTATE_TO_GOAL:
            if dist < self._pos_tol:
                self._phase = self.PHASE_ROTATE_TO_HEADING
                return
            if abs(heading_err) < self._hdg_tol:
                self._phase = self.PHASE_DRIVE
                return
            cmd.angular.z = self._clamp(
                self._kp_ang * heading_err, self._max_ang)

        # ------ Phase 2: drive + reactive obstacle avoidance ------
        elif self._phase == self.PHASE_DRIVE:
            if dist < self._pos_tol:
                self._phase = self.PHASE_ROTATE_TO_HEADING
                self.get_logger().info(
                    f'[{self._wp_index}/{self._wp_total}] '
                    f'Position reached — aligning heading')
                return
            if abs(heading_err) > 0.8:
                self._phase = self.PHASE_ROTATE_TO_GOAL
                return

            min_left, min_right = self._check_obstacles()
            min_front = min(min_left, min_right)

            if min_front < self._obs_stop:
                # Very close — full stop, turn hard away from blocked side
                if min_left <= min_right:
                    cmd.angular.z = -self._max_ang
                else:
                    cmd.angular.z = self._max_ang
                self.get_logger().info(
                    f'OBSTACLE  stop & turn  L={min_left:.2f} R={min_right:.2f}',
                    throttle_duration_sec=1.0)
            elif min_front < self._obs_dist:
                # Approaching obstacle — slow down and steer away
                speed_scale = (min_front - self._obs_stop) / (
                    self._obs_dist - self._obs_stop)
                cmd.linear.x = self._clamp(
                    self._kp_lin * dist * speed_scale, self._max_lin * speed_scale)
                # Blend: steer toward goal but bias away from obstacle
                avoid_ang = self._max_ang * 0.6
                if min_left <= min_right:
                    steer = self._kp_ang * heading_err - avoid_ang
                else:
                    steer = self._kp_ang * heading_err + avoid_ang
                cmd.angular.z = self._clamp(steer, self._max_ang)
                self.get_logger().info(
                    f'OBSTACLE  slow & steer  L={min_left:.2f} R={min_right:.2f}',
                    throttle_duration_sec=1.0)
            else:
                # Clear path — normal proportional drive
                cmd.linear.x = self._clamp(
                    self._kp_lin * dist, self._max_lin)
                cmd.angular.z = self._clamp(
                    self._kp_ang * heading_err, self._max_ang)

        # ------ Phase 3: rotate to match goal heading ------
        elif self._phase == self.PHASE_ROTATE_TO_HEADING:
            if abs(orient_err) < self._hdg_tol:
                self._cmd_pub.publish(Twist())
                self.get_logger().info(
                    f'[{self._wp_index}/{self._wp_total}] '
                    f'Waypoint reached  pos_err={dist:.3f} m  '
                    f'yaw_err={math.degrees(orient_err):.1f}°')
                self._advance()
                self._publish_path_viz()
                return
            cmd.angular.z = self._clamp(
                self._kp_ang * orient_err, self._max_ang)

        self._cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node._cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
