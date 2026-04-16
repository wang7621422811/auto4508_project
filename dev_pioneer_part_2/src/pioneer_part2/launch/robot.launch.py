"""
Real-robot launch file for Pioneer P3-AT Part 2.

Replaces Gazebo + bridges with actual hardware drivers:

  ┌─────────────────────────────────────────────────────────┐
  │  Hardware driver         Package          Topic(s)       │
  ├─────────────────────────────────────────────────────────┤
  │  ARIA motor driver       ros2aria         /odom, /cmd_vel│
  │  SICK TIM781 LiDAR       sick_scan_xd     /scan          │
  │    (or Lakibeam alt)     lakibeam_ros2                   │
  │  OAK-D camera            depthai_ros_driver /camera      │
  │  Phidget spatial IMU     phidgets_spatial /imu           │
  │  GPS (gpsd + NMEA)       nmea_navsat_driver /gps/fix     │
  │  Bluetooth gamepad       joy              /joy           │
  └─────────────────────────────────────────────────────────┘

Then starts:
  - robot_state_publisher  (uses URDF from p3at package)
  - SLAM Toolbox (online async)
  - gps_converter          (converts /gps/fix → /gps_waypoints)
  - All 6 Part 2 application nodes

Launch arguments
----------------
  lidar_driver  : "sick" (default) or "lakibeam"
  rviz          : "true" / "false" (default true)
  slam_params   : override SLAM config file path

Usage
-----
  ros2 launch pioneer_part2 robot.launch.py
  ros2 launch pioneer_part2 robot.launch.py lidar_driver:=lakibeam rviz:=false

Prerequisites
-------------
  sudo apt install ros-jazzy-ros2aria  # or build from source
  sudo apt install ros-jazzy-sick-scan-xd
  sudo apt install ros-jazzy-depthai-ros
  sudo apt install ros-jazzy-phidgets-spatial
  sudo apt install ros-jazzy-nmea-navsat-driver
  sudo apt install ros-jazzy-joy
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_part2 = get_package_share_directory('pioneer_part2')
    pkg_p3at = get_package_share_directory('p3at')
    pkg_slam = get_package_share_directory('slam_toolbox')

    # Config files
    robot_params = os.path.join(pkg_part2, 'config', 'mission_params_robot.yaml')
    gps_wp_file = os.path.join(pkg_part2, 'config', 'waypoints_gps.yaml')
    slam_params_default = os.path.join(pkg_p3at, 'config', 'slam_params.yaml')
    robot_urdf = os.path.join(pkg_p3at, 'robots', 'pioneer.urdf')

    with open(robot_urdf, 'r') as f:
        robot_desc = f.read()

    # ---- Arguments -----------------------------------------------------------
    lidar_arg = DeclareLaunchArgument(
        'lidar_driver', default_value='sick',
        description='LiDAR driver to use: "sick" (TIM781) or "lakibeam"')

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 visualiser')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params', default_value=slam_params_default,
        description='Path to SLAM Toolbox parameter file')

    # ---- robot_state_publisher -----------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': False, 'robot_description': robot_desc}],
    )

    # =========================================================================
    # Hardware drivers
    # =========================================================================

    # ---- ARIA motor driver --------------------------------------------------
    # ros2aria publishes /odom and /tf (base_link → odom) and subscribes /cmd_vel
    aria_driver = Node(
        package='ros2aria',
        executable='ros2aria',
        name='ros2aria',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': '/dev/ttyUSB0',   # adjust to actual serial port
        }],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
            ('odom', 'odom'),
        ],
    )

    # ---- SICK TIM781 LiDAR --------------------------------------------------
    sick_lidar = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_lidar',
        output='screen',
        condition=IfCondition(
            # Only start when lidar_driver == "sick"
            # LaunchConfiguration returns a string; convert using PythonExpression
            '$(eval "' + "sick" + '" == "$(var lidar_driver)")'
        ),
        parameters=[{
            'use_sim_time': False,
            'scanner_type': 'sick_tim_7xx',
            'hostname': '192.168.0.1',    # default TIM781 IP
            'frame_id': 'laser',
        }],
        remappings=[('scan', 'scan')],
    )

    # ---- Lakibeam LiDAR (alternative) ----------------------------------------
    lakibeam_lidar = Node(
        package='lakibeam_ros2',
        executable='lakibeam_node',
        name='lakibeam_lidar',
        output='screen',
        condition=IfCondition(
            '$(eval "$(var lidar_driver)" == "lakibeam")'
        ),
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'laser',
            'ip_address': '192.168.0.3',   # adjust to device IP
        }],
        remappings=[('scan', 'scan')],
    )

    # ---- OAK-D camera (DepthAI ROS driver) ----------------------------------
    # depthai_ros_driver publishes:
    #   /oak/rgb/image_raw        → remapped to /camera
    #   /oak/rgb/camera_info      → remapped to /camera_info
    #   /oak/stereo/image_raw     → kept as-is (used by vision_detector depth)
    oakd_camera = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oakd_camera',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'camera_model': 'OAK-D',
            'tf_prefix': 'oak',
        }],
        remappings=[
            ('oak/rgb/image_raw', 'camera'),
            ('oak/rgb/camera_info', 'camera_info'),
        ],
    )

    # ---- Phidgets spatial IMU -----------------------------------------------
    phidget_imu = Node(
        package='phidgets_spatial',
        executable='phidgets_spatial_node',
        name='phidgets_imu',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'imu_link',
            'linear_acceleration_stdev': 0.01,
            'angular_velocity_stdev': 0.005,
        }],
        remappings=[
            ('imu/data_raw', 'imu'),
        ],
    )

    # ---- GPS (gpsd + nmea_navsat_driver) ------------------------------------
    # Requires:  sudo systemctl start gpsd
    #            sudo gpsd /dev/ttyUSB1 -F /var/run/gpsd.sock
    gps_driver = Node(
        package='nmea_navsat_driver',
        executable='nmea_topic_driver',
        name='gps_driver',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': '/dev/ttyUSB1',   # adjust to actual GPS serial port
            'baud': 4800,
            'frame_id': 'gps',
        }],
        remappings=[('fix', 'gps/fix')],
    )

    # ---- Bluetooth gamepad (joy) ---------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autorepeat_rate': 20.0,
            'device_id': 0,
        }],
    )

    # =========================================================================
    # Localisation / mapping
    # =========================================================================

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params'),
            'use_sim_time': 'false',
        }.items(),
    )

    # =========================================================================
    # GPS converter
    # =========================================================================

    gps_converter = Node(
        package='pioneer_part2',
        executable='gps_converter',
        name='gps_converter',
        output='screen',
        parameters=[
            robot_params,
            {'waypoints_gps_file': gps_wp_file},
        ],
    )

    # =========================================================================
    # Part 2 application nodes
    # =========================================================================

    mission_ctrl = Node(
        package='pioneer_part2',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[
            robot_params,
            {'waypoints_file': gps_wp_file,
             'use_gps_waypoints': True},
        ],
    )

    waypoint_ctrl = Node(
        package='pioneer_part2',
        executable='waypoint_controller',
        name='waypoint_controller',
        output='screen',
        parameters=[robot_params],
    )

    cone_weaver = Node(
        package='pioneer_part2',
        executable='cone_weaver',
        name='cone_weaver',
        output='screen',
        parameters=[robot_params],
    )

    vision_detector = Node(
        package='pioneer_part2',
        executable='vision_detector',
        name='vision_detector',
        output='screen',
        parameters=[robot_params],
    )

    gamepad_ctrl = Node(
        package='pioneer_part2',
        executable='gamepad_controller',
        name='gamepad_controller',
        output='screen',
        parameters=[robot_params],
    )

    path_recorder = Node(
        package='pioneer_part2',
        executable='path_recorder',
        name='path_recorder',
        output='screen',
        parameters=[robot_params],
    )

    # ---- RViz (optional) ----------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': False}],
    )

    # =========================================================================
    # Assembly
    # =========================================================================
    #
    # Startup sequence (seconds from launch):
    #   0 s  : hardware drivers + robot_state_publisher
    #   3 s  : SLAM Toolbox  (needs TF from aria_driver to be up)
    #   5 s  : gps_converter (needs /gps/fix)
    #   6 s  : all Part 2 app nodes + RViz
    #            (SLAM must be producing /map before mission_controller starts)
    #
    return LaunchDescription([
        lidar_arg,
        rviz_arg,
        slam_params_arg,

        # --- Immediate: hardware + tf ------------------------------------------
        robot_state_publisher,
        aria_driver,
        sick_lidar,
        lakibeam_lidar,
        oakd_camera,
        phidget_imu,
        gps_driver,
        joy_node,

        # --- After drivers settle (3 s) ----------------------------------------
        TimerAction(period=3.0, actions=[slam]),

        # --- After SLAM has a map frame (5 s) ----------------------------------
        TimerAction(period=5.0, actions=[gps_converter]),

        # --- Application nodes — after TF + map + GPS are ready (6 s) ---------
        TimerAction(period=6.0, actions=[
            waypoint_ctrl,
            cone_weaver,
            vision_detector,
            gamepad_ctrl,
            path_recorder,
            mission_ctrl,
            rviz,
        ]),
    ])
