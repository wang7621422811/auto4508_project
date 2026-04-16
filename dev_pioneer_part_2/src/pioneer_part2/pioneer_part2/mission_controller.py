"""
Mission controller for Pioneer P3-AT Part 2.

Orchestrates the complete mission:
  1. Wait for gamepad automated mode (X button + dead-man switch)
  2. Record start position from TF
  3. Navigate to each waypoint in sequence
     - Between WP1 and WP2: activate ConeWeaver for slalom
     - On arrival at each waypoint: capture photo + detect coloured object
  4. After last waypoint: return to start
  5. Print and publish journey summary

Waypoint sources
----------------
  Simulation (use_gps_waypoints: false, default):
    Loaded from waypoints.yaml (map-frame x/y coordinates).

  Real robot  (use_gps_waypoints: true):
    Subscribed from /gps_waypoints (nav_msgs/Path) as published by
    gps_converter.  The node waits until waypoints are received before
    starting the mission.  Also subscribes to /gps/fix to log the raw
    GPS position in the journey summary.

State machine
-------------
IDLE
  → WAITING_FOR_AUTO   (gamepad mode subscribed)
  → NAVIGATING         (waypoints dispatched to WaypointController)
  → WEAVING            (ConeWeaver active for WP1→WP2)
  → AT_WAYPOINT        (trigger vision services, short pause)
  → RETURNING          (navigate back to start)
  → COMPLETE           (publish summary)

Topics subscribed
-----------------
  /gamepad_mode      std_msgs/String
  /waypoint_reached  std_msgs/Int32
  /weave_done        std_msgs/Bool
  /detection_result  std_msgs/String
  /gps_waypoints     nav_msgs/Path          (when use_gps_waypoints: true)
  /gps/fix           sensor_msgs/NavSatFix  (when use_gps_waypoints: true)

Topics published
----------------
  /waypoints         nav_msgs/Path   (to WaypointController)
  /mission_active    std_msgs/Bool
  /mission_status    std_msgs/String

Services called
---------------
  /capture_photo     std_srvs/Empty
  /detect_object     std_srvs/Empty
  /start_weave       std_srvs/Empty
  /path_recorder/save  std_srvs/Empty
"""

import json
import math
import os
import time

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Int32, String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _make_pose_stamped(x, y, yaw, frame='map') -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = float(x)
    ps.pose.position.y = float(y)
    ps.pose.position.z = 0.0
    half = yaw / 2.0
    ps.pose.orientation.z = math.sin(half)
    ps.pose.orientation.w = math.cos(half)
    return ps


def _load_waypoints(yaml_path: str) -> tuple[list[dict], list[int], list[dict]]:
    """Return (waypoints, weave_segment, weave_cone_positions)."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    wps = data.get('waypoints', [])
    seg = data.get('weave_segment', [0, 1])
    cones = data.get('weave_cone_positions', [])
    return wps, seg, cones


class MissionController(Node):

    # States
    S_IDLE = 'IDLE'
    S_WAIT_AUTO = 'WAIT_AUTO'
    S_NAVIGATE = 'NAVIGATE'
    S_WEAVE = 'WEAVE'
    S_AT_WAYPOINT = 'AT_WAYPOINT'
    S_RETURN = 'RETURN'
    S_COMPLETE = 'COMPLETE'

    def __init__(self):
        super().__init__('mission_controller')

        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('waypoint_reached_radius', 1.5)
        self.declare_parameter('photo_save_dir',
                               os.path.expanduser('~/mission_photos'))
        self.declare_parameter('weave_enabled', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        # GPS waypoint mode: when true, waypoints arrive via /gps_waypoints topic
        # instead of being loaded from a local YAML file.
        self.declare_parameter('use_gps_waypoints', False)

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._weave_enabled = self.get_parameter('weave_enabled').value
        self._use_gps_waypoints = self.get_parameter('use_gps_waypoints').value

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        # Waypoint storage (populated from YAML or GPS topic)
        self._waypoints: list[dict] = []
        self._weave_seg: list[int] = [0, 1]
        self._weave_cones: list[dict] = []
        self._gps_waypoints_received = False

        if self._use_gps_waypoints:
            self.get_logger().info(
                'GPS waypoint mode: waiting for /gps_waypoints from gps_converter')
        else:
            wp_file = self.get_parameter('waypoints_file').value
            if not wp_file:
                from ament_index_python.packages import get_package_share_directory
                pkg = get_package_share_directory('pioneer_part2')
                wp_file = os.path.join(pkg, 'config', 'waypoints.yaml')
            self._waypoints, self._weave_seg, self._weave_cones = \
                _load_waypoints(wp_file)
            self._gps_waypoints_received = True   # already have waypoints
            self.get_logger().info(
                f'Loaded {len(self._waypoints)} waypoints from YAML, '
                f'weave segment: WP{self._weave_seg[0]+1}→WP{self._weave_seg[1]+1}')

        # GPS fix tracking (real robot only, used in journey summary)
        self._latest_gps: NavSatFix | None = None

        # Mission state
        self._state = self.S_IDLE
        self._current_wp_idx = -1         # waypoint currently being navigated to
        self._start_pose: tuple | None = None
        self._mission_start_time: float = 0.0
        self._detections: list[dict] = []  # indexed by wp index
        self._photos: list[str] = []
        self._mode = 'stopped'
        self._weave_done = False

        # Publishers
        self._wp_pub = self.create_publisher(Path, 'waypoints', 1)
        self._active_pub = self.create_publisher(Bool, 'mission_active', 10)
        self._status_pub = self.create_publisher(String, 'mission_status', 10)

        # Subscriptions
        self.create_subscription(String, 'gamepad_mode', self._mode_cb, 10)
        self.create_subscription(Int32, 'waypoint_reached', self._reached_cb, 10)
        self.create_subscription(Bool, 'weave_done', self._weave_done_cb, 10)
        self.create_subscription(String, 'detection_result',
                                 self._detection_cb, 10)

        if self._use_gps_waypoints:
            # Transient-local QoS to receive the latched waypoints path even if
            # we subscribe after gps_converter has already published.
            latched_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(
                Path, 'gps_waypoints', self._gps_waypoints_cb, latched_qos)
            self.create_subscription(
                NavSatFix, 'gps/fix', self._gps_fix_cb, 10)

        # Service clients
        self._cli_photo = self.create_client(Empty, 'capture_photo')
        self._cli_detect = self.create_client(Empty, 'detect_object')
        self._cli_weave = self.create_client(Empty, 'start_weave')
        self._cli_save = self.create_client(Empty, 'path_recorder/save')

        # State machine timer (10 Hz)
        self._timer = self.create_timer(0.1, self._sm_tick)
        # Pause timer for AT_WAYPOINT dwell time
        self._dwell_until: float = 0.0

        self._publish_status('IDLE — waiting for automated mode (X button)')
        self.get_logger().info('Mission controller ready')

    # ---- subscriptions -------------------------------------------------------

    def _mode_cb(self, msg: String):
        self._mode = msg.data
        if self._state == self.S_WAIT_AUTO and self._mode == 'auto':
            self.get_logger().info('Automated mode engaged — starting mission')
            self._start_mission()

    def _reached_cb(self, msg: Int32):
        if self._state == self.S_NAVIGATE:
            self.get_logger().info(
                f'Waypoint {msg.data} reached — triggering vision')
            self._transition_to_at_waypoint()
        elif self._state == self.S_RETURN:
            self.get_logger().info('Return waypoint reached — mission complete')
            self._state = self.S_COMPLETE

    def _weave_done_cb(self, msg: Bool):
        if msg.data and self._state == self.S_WEAVE:
            self.get_logger().info(
                'Weave complete — proceeding to WP2')
            # Dispatch WP2 to the waypoint controller
            wp_idx = self._weave_seg[1]
            self._current_wp_idx = wp_idx
            self._dispatch_single_waypoint(wp_idx)
            self._state = self.S_NAVIGATE

    def _detection_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('type') == 'photo':
            self._photos.append(data.get('path', ''))
        elif data.get('type') == 'detection':
            self._detections.append(
                {'wp': self._current_wp_idx + 1, **data})

    def _gps_waypoints_cb(self, msg: Path):
        """Convert received nav_msgs/Path poses into internal waypoint dicts."""
        if self._gps_waypoints_received:
            return  # already have waypoints; ignore re-publishes
        wps = []
        for ps in msg.poses:
            wps.append({
                'x': ps.pose.position.x,
                'y': ps.pose.position.y,
                'yaw': 0.0,   # GPS converter does not supply heading
            })
        if not wps:
            self.get_logger().warn('Received empty /gps_waypoints — ignoring')
            return
        self._waypoints = wps
        self._gps_waypoints_received = True
        self.get_logger().info(
            f'GPS waypoints received: {len(wps)} poses '
            f'(weave segment WP{self._weave_seg[0]+1}→WP{self._weave_seg[1]+1})')

    def _gps_fix_cb(self, msg: NavSatFix):
        """Store latest GPS fix for inclusion in journey summary."""
        self._latest_gps = msg

    # ---- state machine -------------------------------------------------------

    def _sm_tick(self):
        if self._state == self.S_IDLE:
            self._state = self.S_WAIT_AUTO
            self._publish_status('Waiting for automated mode (press X button)')

        elif self._state == self.S_WAIT_AUTO:
            if not self._gps_waypoints_received:
                # Still waiting for gps_converter to deliver waypoints
                self.get_logger().info(
                    'Waiting for GPS waypoints from /gps_waypoints ...',
                    throttle_duration_sec=5.0)
                return
            if self._mode == 'auto':
                self._start_mission()

        elif self._state == self.S_AT_WAYPOINT:
            if time.monotonic() >= self._dwell_until:
                self._advance_after_waypoint()

        elif self._state == self.S_COMPLETE:
            self._print_summary()
            self._state = 'DONE'
            self._publish_status('Mission complete')

        elif self._state == 'DONE':
            pass  # terminal

        # Safety: stop if auto mode is revoked mid-mission
        if self._state not in (self.S_IDLE, self.S_WAIT_AUTO, 'DONE'):
            if self._mode not in ('auto',):
                self.get_logger().warn(
                    'Auto mode disabled mid-mission — pausing')
                self._publish_active(False)

    # ---- mission flow --------------------------------------------------------

    def _start_mission(self):
        self._mission_start_time = time.monotonic()
        self._start_pose = self._get_pose()
        if self._start_pose is None:
            self.get_logger().warn(
                'Cannot get start pose from TF — retrying next tick')
            return
        self.get_logger().info(
            f'Mission start pose: ({self._start_pose[0]:.2f}, '
            f'{self._start_pose[1]:.2f})')
        self._current_wp_idx = 0
        self._publish_active(True)
        self._navigate_to_current_wp()

    def _navigate_to_current_wp(self):
        idx = self._current_wp_idx
        if idx >= len(self._waypoints):
            self.get_logger().info('All waypoints done — returning to start')
            self._dispatch_return()
            return

        wp = self._waypoints[idx]
        self.get_logger().info(
            f'Navigating to WP{idx+1}: ({wp["x"]:.2f}, {wp["y"]:.2f})')
        self._publish_status(f'NAVIGATE → WP{idx+1}')
        self._dispatch_single_waypoint(idx)
        self._state = self.S_NAVIGATE

    def _dispatch_single_waypoint(self, idx: int):
        wp = self._waypoints[idx]
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        ps = _make_pose_stamped(wp['x'], wp['y'], wp.get('yaw', 0.0))
        ps.header.stamp = path.header.stamp
        path.poses = [ps]
        self._wp_pub.publish(path)

    def _transition_to_at_waypoint(self):
        self._state = self.S_AT_WAYPOINT
        self._publish_status(f'AT_WAYPOINT WP{self._current_wp_idx+1} — capturing')
        self._publish_active(False)  # pause waypoint controller
        # Trigger photo + detection (fire-and-forget; results come back async)
        self._call_service(self._cli_photo, 'capture_photo')
        self._call_service(self._cli_detect, 'detect_object')
        self._dwell_until = time.monotonic() + 3.0  # 3 s dwell

    def _advance_after_waypoint(self):
        idx = self._current_wp_idx
        next_idx = idx + 1

        # Check if this is the weave segment start
        if (self._weave_enabled
                and idx == self._weave_seg[0]
                and next_idx == self._weave_seg[1]):
            self.get_logger().info(
                f'Starting weave manoeuvre WP{idx+1}→WP{next_idx+1}')
            self._state = self.S_WEAVE
            self._publish_status(f'WEAVING WP{idx+1}→WP{next_idx+1}')
            self._publish_active(True)
            self._call_service(self._cli_weave, 'start_weave')
            # After weave, _weave_done_cb advances to WP{next_idx}
        else:
            self._current_wp_idx = next_idx
            self._publish_active(True)
            self._navigate_to_current_wp()

    def _dispatch_return(self):
        if self._start_pose is None:
            self.get_logger().warn('Start pose unknown — cannot return')
            self._state = self.S_COMPLETE
            return
        sx, sy, _ = self._start_pose
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        ps = _make_pose_stamped(sx, sy, 0.0)
        ps.header.stamp = path.header.stamp
        path.poses = [ps]
        self._wp_pub.publish(path)
        self._state = self.S_RETURN
        self._publish_status('RETURNING to start')
        self._publish_active(True)
        # Save map + path
        self._call_service(self._cli_save, 'path_recorder/save')

    # ---- journey summary -----------------------------------------------------

    def _print_summary(self):
        elapsed = time.monotonic() - self._mission_start_time
        mins, secs = divmod(int(elapsed), 60)

        lines = [
            '',
            '=' * 60,
            '  MISSION JOURNEY SUMMARY',
            '=' * 60,
            f'  Total mission time : {mins}m {secs}s',
            f'  Waypoints visited  : {len(self._waypoints)}',
            f'  Photos taken       : {len(self._photos)}',
        ]

        # Include last known GPS fix if running on real robot
        if self._latest_gps is not None:
            gps = self._latest_gps
            lines.append(
                f'  Final GPS position : '
                f'lat={gps.latitude:.6f}  lon={gps.longitude:.6f}  '
                f'alt={gps.altitude:.1f} m')

        lines.append('')

        if self._photos:
            lines.append('  Photos:')
            for p in self._photos:
                lines.append(f'    {p}')
            lines.append('')

        if self._detections:
            lines.append('  Object detections at waypoints:')
            for d in self._detections:
                wp = d.get('wp', '?')
                color = d.get('color', '?')
                shape = d.get('shape', '?')
                dist = d.get('distance_m', -1.0)
                dist_str = f'{dist:.2f} m' if dist >= 0 else 'unknown'
                lines.append(
                    f'    WP{wp}: {color} {shape} — distance {dist_str}')
        else:
            lines.append('  No objects detected.')

        lines.append('=' * 60)
        summary = '\n'.join(lines)
        self.get_logger().info(summary)
        self._status_pub.publish(String(data=summary))

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

    def _publish_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self._active_pub.publish(msg)

    def _publish_status(self, text: str):
        self.get_logger().info(f'[MISSION] {text}')
        self._status_pub.publish(String(data=text))

    def _call_service(self, client, name: str):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Service {name} not available')
            return
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
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
