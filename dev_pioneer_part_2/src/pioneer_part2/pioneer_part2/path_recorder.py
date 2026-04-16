"""
Path recorder with journey summary for Pioneer P3-AT Part 2.

Extends the Part 1 path_recorder with:
  - /path_recorder/save  service — saves SLAM map + driven path CSV (as before)
  - /path_recorder/journey_summary  service — returns a formatted text summary
    including total distance, mission time, waypoints visited, photos taken,
    and detected objects (pulled from /detection_result topic)

Subscriptions
-------------
  (TF) map → base_link   sampled at record_hz
  /detection_result       std_msgs/String  (JSON objects from VisionDetector)

Services
--------
  ~/save               std_srvs/Empty
  ~/journey_summary    std_srvs/Empty

Published
---------
  /driven_path         nav_msgs/Path
"""

import csv
import json
import math
import os
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathRecorder(Node):

    def __init__(self):
        super().__init__('path_recorder')

        self.declare_parameter('record_hz', 2.0)
        self.declare_parameter('publish_hz', 1.0)
        self.declare_parameter('min_move', 0.05)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_dir',
                               os.path.expanduser('~/mission_data'))

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._min_move = self.get_parameter('min_move').value
        self._output_dir = self.get_parameter('output_dir').value
        rec_hz = self.get_parameter('record_hz').value
        pub_hz = self.get_parameter('publish_hz').value

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)

        self._path_pub = self.create_publisher(Path, 'driven_path', 10)
        self.create_service(Empty, '~/save', self._save_cb)
        self.create_service(Empty, '~/journey_summary', self._summary_cb)

        self.create_subscription(String, 'detection_result',
                                 self._detection_cb, 10)

        self._poses: list[PoseStamped] = []
        self._last_x = None
        self._last_y = None
        self._total_dist = 0.0
        self._start_time = self.get_clock().now()

        # Journey data accumulated from detection_result topic
        self._photos: list[str] = []
        self._detections: list[dict] = []
        self._waypoints_visited = 0

        self.create_timer(1.0 / rec_hz, self._record_tick)
        self.create_timer(1.0 / pub_hz, self._publish_tick)

        self.get_logger().info(
            f'Path recorder ready — output dir: {self._output_dir}')

    # ---- detection listener --------------------------------------------------

    def _detection_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if data.get('type') == 'photo':
            path = data.get('path', '')
            if path:
                self._photos.append(path)
        elif data.get('type') == 'detection':
            self._detections.append(data)
            self._waypoints_visited = max(
                self._waypoints_visited, data.get('wp', 0))

    # ---- TF sampling ---------------------------------------------------------

    def _record_tick(self):
        try:
            t = self._tf_buf.lookup_transform(
                self._map_frame, self._base_frame, rclpy.time.Time())
        except Exception:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y

        if self._last_x is not None:
            d = math.hypot(x - self._last_x, y - self._last_y)
            if d < self._min_move:
                return
            self._total_dist += d

        self._last_x = x
        self._last_y = y

        ps = PoseStamped()
        ps.header.frame_id = self._map_frame
        ps.header.stamp = t.header.stamp
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.orientation = t.transform.rotation
        self._poses.append(ps)

    # ---- publish for RViz ----------------------------------------------------

    def _publish_tick(self):
        if not self._poses:
            return
        msg = Path()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = list(self._poses)
        self._path_pub.publish(msg)

    # ---- save service --------------------------------------------------------

    def _save_cb(self, _req, resp):
        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Driven path CSV
        csv_path = os.path.join(self._output_dir, f'path_{ts}.csv')
        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['stamp_sec', 'stamp_nanosec', 'x', 'y', 'yaw_rad'])
            for ps in self._poses:
                q = ps.pose.orientation
                yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
                w.writerow([
                    ps.header.stamp.sec,
                    ps.header.stamp.nanosec,
                    f'{ps.pose.position.x:.4f}',
                    f'{ps.pose.position.y:.4f}',
                    f'{yaw:.4f}',
                ])
        self.get_logger().info(
            f'Path saved: {csv_path}  '
            f'({len(self._poses)} poses, {self._total_dist:.2f} m)')

        # SLAM map
        # Use the node's own use_sim_time setting instead of hardcoding true
        # so map_saver_cli behaves correctly both in sim and on the real robot.
        use_sim = self.get_parameter('use_sim_time').value
        sim_time_str = 'true' if use_sim else 'false'
        map_prefix = os.path.join(self._output_dir, f'map_{ts}')
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_prefix, '--ros-args', '-p',
             f'use_sim_time:={sim_time_str}'],
            capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            self.get_logger().info(f'Map saved: {map_prefix}.pgm / .yaml')
        else:
            self.get_logger().warn(
                f'map_saver_cli returned {result.returncode}: '
                f'{result.stderr.strip()}')

        return resp

    # ---- journey summary service ---------------------------------------------

    def _summary_cb(self, _req, resp):
        now = self.get_clock().now()
        elapsed_sec = (now - self._start_time).nanoseconds / 1e9
        mins, secs = divmod(int(elapsed_sec), 60)

        lines = [
            '',
            '=' * 60,
            '  JOURNEY SUMMARY',
            '=' * 60,
            f'  Total distance     : {self._total_dist:.2f} m',
            f'  Mission time       : {mins}m {secs}s',
            f'  Waypoints visited  : {self._waypoints_visited}',
            f'  Photos taken       : {len(self._photos)}',
            f'  Path poses logged  : {len(self._poses)}',
            '',
        ]

        if self._photos:
            lines.append('  Photos:')
            for p in self._photos:
                lines.append(f'    {p}')
            lines.append('')

        if self._detections:
            lines.append('  Detected objects:')
            for d in self._detections:
                wp = d.get('wp', '?')
                color = d.get('color', '?')
                shape = d.get('shape', '?')
                dist = d.get('distance_m', -1.0)
                dist_str = f'{dist:.2f} m' if isinstance(dist, float) and dist >= 0 else 'unknown'
                lines.append(f'    WP{wp}: {color} {shape} at {dist_str}')
        else:
            lines.append('  No object detections recorded.')

        lines.append('=' * 60)
        summary = '\n'.join(lines)
        self.get_logger().info(summary)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = PathRecorder()
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
