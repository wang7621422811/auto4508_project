"""
GPS Converter node for Pioneer P3-AT Part 2 (real robot).

Responsibilities
----------------
1. Waits for the first valid GPS fix on /gps/fix (sensor_msgs/NavSatFix).
   If a datum (reference lat/lon) is configured, that is used immediately;
   otherwise the first valid fix is stored as the ENU origin.

2. Loads GPS waypoints from waypoints_gps.yaml and converts each
   latitude/longitude to a local East-North-Up (x=East, y=North) coordinate
   using the WGS-84 spherical approximation (accurate to ~10 cm over 100 m).

3. Publishes the converted waypoints once as /gps_waypoints (nav_msgs/Path)
   so mission_controller can consume them (when use_gps_waypoints: true).

4. Continuously publishes the robot's current GPS position as
   /gps_odom (nav_msgs/Odometry) so it can be used for odometry fusion
   or as a fallback localisation source.

5. Publishes a static TF: map → gps_origin (z=0, identity rotation) so
   the ENU frame is aligned with the ROS map frame origin.

Parameters
----------
  waypoints_gps_file   str    path to waypoints_gps.yaml
  map_frame            str    default "map"
  fix_covariance_max   float  max acceptable fix covariance (m^2), default 25.0
  publish_hz           float  rate for /gps_odom, default 5.0

Subscriptions
-------------
  /gps/fix   sensor_msgs/NavSatFix

Published
---------
  /gps_waypoints   nav_msgs/Path          (latched, published once on first fix)
  /gps_odom        nav_msgs/Odometry      (continuous)

Static TF published
-------------------
  map → gps_origin  (identity, aligns ENU origin with map frame)
"""

import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import NavSatFix
from tf2_ros import StaticTransformBroadcaster


# ---- ENU conversion ---------------------------------------------------------

def gps_to_enu(lat: float, lon: float,
               ref_lat: float, ref_lon: float) -> tuple[float, float]:
    """Convert (lat, lon) to local ENU (x=East, y=North) in metres.

    Uses the spherical WGS-84 approximation valid to <1% for distances
    under ~1 km (more than sufficient for a 150 m oval).
    """
    EARTH_R = 6371000.0
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    x = dlon * EARTH_R * math.cos(math.radians(ref_lat))   # East
    y = dlat * EARTH_R                                       # North
    return x, y


def _make_pose_stamped(x: float, y: float, frame: str = 'map') -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = 0.0
    ps.pose.orientation.w = 1.0
    return ps


# ---- Node -------------------------------------------------------------------

class GpsConverter(Node):

    def __init__(self):
        super().__init__('gps_converter')

        self.declare_parameter('waypoints_gps_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('fix_covariance_max', 25.0)
        self.declare_parameter('publish_hz', 5.0)
        # Datum can be pre-configured; if not set (0.0, 0.0) the first valid
        # fix is used as the reference origin.
        self.declare_parameter('datum_latitude', 0.0)
        self.declare_parameter('datum_longitude', 0.0)

        self._map_frame = self.get_parameter('map_frame').value
        self._cov_max = self.get_parameter('fix_covariance_max').value
        self._datum_lat = self.get_parameter('datum_latitude').value
        self._datum_lon = self.get_parameter('datum_longitude').value

        # ENU origin
        self._ref_lat: float | None = None
        self._ref_lon: float | None = None

        # Accept datum from parameter if non-zero
        if self._datum_lat != 0.0 or self._datum_lon != 0.0:
            self._ref_lat = self._datum_lat
            self._ref_lon = self._datum_lon
            self.get_logger().info(
                f'Using configured datum: '
                f'lat={self._ref_lat:.6f}, lon={self._ref_lon:.6f}')

        # Latest GPS fix for continuous odometry publishing
        self._latest_fix: NavSatFix | None = None
        self._waypoints_published = False

        # Load GPS waypoints from file
        wp_file = self.get_parameter('waypoints_gps_file').value
        if not wp_file:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory('pioneer_part2')
            wp_file = os.path.join(pkg, 'config', 'waypoints_gps.yaml')
        self._wp_data = self._load_gps_waypoints(wp_file)

        # If file contains a datum and parameter datum is not set, use file datum
        if self._ref_lat is None and self._wp_data.get('datum'):
            d = self._wp_data['datum']
            self._ref_lat = d['latitude']
            self._ref_lon = d['longitude']
            self.get_logger().info(
                f'Datum loaded from waypoints file: '
                f'lat={self._ref_lat:.6f}, lon={self._ref_lon:.6f}')

        # QoS: latched for waypoints (transient_local so late subscribers get it)
        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._wp_pub = self.create_publisher(Path, 'gps_waypoints', latched_qos)
        self._odom_pub = self.create_publisher(Odometry, 'gps_odom', 10)

        self.create_subscription(NavSatFix, 'gps/fix', self._fix_cb, 10)

        self._static_tf_pub = StaticTransformBroadcaster(self)

        hz = self.get_parameter('publish_hz').value
        self.create_timer(1.0 / hz, self._publish_odom)

        self.get_logger().info(
            'GPS converter ready — waiting for /gps/fix\n'
            f'  waypoints file: {wp_file}')

    # ---- YAML loading -------------------------------------------------------

    def _load_gps_waypoints(self, path: str) -> dict:
        try:
            with open(path, 'r') as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            self.get_logger().warn(f'waypoints_gps_file not found: {path}')
            return {}
        except Exception as e:
            self.get_logger().error(f'Failed to load {path}: {e}')
            return {}

    # ---- GPS fix callback ---------------------------------------------------

    def _fix_cb(self, msg: NavSatFix):
        # Reject fixes with large position covariance (poor accuracy)
        cov = msg.position_covariance[0]
        if cov > self._cov_max:
            self.get_logger().warn(
                f'GPS fix rejected: covariance {cov:.1f} > {self._cov_max:.1f}',
                throttle_duration_sec=5.0)
            return

        self._latest_fix = msg

        # Use first valid fix as ENU origin if datum not yet set
        if self._ref_lat is None:
            self._ref_lat = msg.latitude
            self._ref_lon = msg.longitude
            self.get_logger().info(
                f'ENU origin set from first fix: '
                f'lat={self._ref_lat:.6f}, lon={self._ref_lon:.6f}')
            self._publish_static_tf()

        # Publish waypoints once we have a reference origin
        if not self._waypoints_published and self._ref_lat is not None:
            self._publish_gps_waypoints()

    # ---- Convert + publish waypoints ----------------------------------------

    def _publish_gps_waypoints(self):
        wps = self._wp_data.get('waypoints', [])
        if not wps:
            self.get_logger().warn('No GPS waypoints found in config file')
            return

        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()

        for wp in wps:
            lat = wp.get('latitude') or wp.get('lat')
            lon = wp.get('longitude') or wp.get('lon')
            if lat is None or lon is None:
                self.get_logger().warn(f'Waypoint missing lat/lon: {wp}')
                continue
            x, y = gps_to_enu(lat, lon, self._ref_lat, self._ref_lon)
            ps = _make_pose_stamped(x, y, self._map_frame)
            ps.header.stamp = path.header.stamp
            path.poses.append(ps)
            self.get_logger().info(
                f"WP '{wp.get('name','?')}': "
                f"({lat:.6f}, {lon:.6f}) → ({x:.2f}, {y:.2f}) m")

        self._wp_pub.publish(path)
        self._waypoints_published = True
        self.get_logger().info(
            f'Published {len(path.poses)} converted GPS waypoints on /gps_waypoints')

    # ---- Publish static TF --------------------------------------------------

    def _publish_static_tf(self):
        if self._ref_lat is None:
            return
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = 'gps_origin'
        t.transform.rotation.w = 1.0   # identity
        self._static_tf_pub.sendTransform(t)

    # ---- Publish GPS odometry -----------------------------------------------

    def _publish_odom(self):
        if self._latest_fix is None or self._ref_lat is None:
            return

        x, y = gps_to_enu(
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            self._ref_lat, self._ref_lon)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self._map_frame
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0

        # Propagate GPS position covariance to odometry
        cov = self._latest_fix.position_covariance[0]
        odom.pose.covariance[0] = cov   # xx
        odom.pose.covariance[7] = cov   # yy
        odom.pose.covariance[35] = 9999.0  # yaw — GPS does not provide heading

        self._odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GpsConverter()
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
