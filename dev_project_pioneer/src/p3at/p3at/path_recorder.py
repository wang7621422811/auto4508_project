"""
Records the robot's driven path and provides map/path saving utilities.

* Periodically samples the ``map → base_link`` TF and accumulates poses.
* Publishes the full driven path on ``/driven_path`` (``nav_msgs/Path``)
  so it can be visualised in RViz.
* ``~/save`` service (``std_srvs/Empty``) — saves the SLAM map **and** the
  driven path (CSV) to ``~/map_data/`` in one call.
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
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
        self.declare_parameter('min_move', 0.05)          # m — skip if robot barely moved
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('output_dir', os.path.expanduser('~/map_data'))

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

        self._poses: list[PoseStamped] = []
        self._last_x = None
        self._last_y = None
        self._total_dist = 0.0

        self.create_timer(1.0 / rec_hz, self._record_tick)
        self.create_timer(1.0 / pub_hz, self._publish_tick)

        self.get_logger().info(
            f'Path recorder ready — call ~/save to save map + path to {self._output_dir}')

    # ---- TF sampling -----------------------------------------------------

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

    # ---- publish for RViz ------------------------------------------------

    def _publish_tick(self):
        if not self._poses:
            return
        msg = Path()
        msg.header.frame_id = self._map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = list(self._poses)
        self._path_pub.publish(msg)

    # ---- save service ----------------------------------------------------

    def _save_cb(self, _req, resp):
        os.makedirs(self._output_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')

        # 1. Save driven path as CSV
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
            f'Path saved: {csv_path}  ({len(self._poses)} poses, '
            f'{self._total_dist:.2f} m total)')

        # 2. Save SLAM map via map_saver_cli
        map_prefix = os.path.join(self._output_dir, f'map_{ts}')
        self.get_logger().info(f'Saving map to {map_prefix} ...')
        import subprocess
        result = subprocess.run(
            ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_prefix, '--ros-args', '-p', 'use_sim_time:=true'],
            capture_output=True, text=True, timeout=15)
        if result.returncode == 0:
            self.get_logger().info(f'Map saved: {map_prefix}.pgm / .yaml')
        else:
            self.get_logger().warn(
                f'map_saver_cli returned {result.returncode}: {result.stderr.strip()}')

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
