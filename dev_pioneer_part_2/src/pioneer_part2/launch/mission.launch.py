"""
Mission launch file for Pioneer P3-AT Part 2.

Launches the full stack:
  - Gazebo Harmonic (mission_world.sdf)
  - ros_gz_bridge (clock, cmd_vel, odom, scan, tf, imu, camera)
  - robot_state_publisher
  - SLAM Toolbox (online async)
  - Nav2 navigation stack
  - RViz2
  - joy_node (gamepad input)
  - All Part 2 nodes: mission_controller, waypoint_controller, cone_weaver,
    vision_detector, gamepad_controller, path_recorder

Timing (seconds from launch):
   0  : Gazebo + bridges + robot_state_publisher
   3  : bridge_sensors
   5  : SLAM Toolbox
   6  : RViz, rqt_robot_steering, Part 2 application nodes
   8  : Nav2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_part2 = get_package_share_directory('pioneer_part2')
    # Part 1 package provides the URDF robot description
    pkg_p3at = get_package_share_directory('p3at')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Paths
    sdf_file = os.path.join(pkg_part2, 'worlds', 'mission_world.sdf')
    robot_file = os.path.join(pkg_p3at, 'robots', 'pioneer.urdf')
    params_file = os.path.join(pkg_part2, 'config', 'mission_params.yaml')
    waypoints_file = os.path.join(pkg_part2, 'config', 'waypoints.yaml')
    # Reuse Part 1 Nav2 + SLAM params (they are world-agnostic)
    nav2_params = os.path.join(pkg_p3at, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_p3at, 'config', 'slam_params.yaml')

    with open(robot_file, 'r') as f:
        robot_desc = f.read()

    # Gazebo needs to find models relative to the p3at share folder
    gz_resource_parent = os.path.dirname(pkg_p3at)

    # ---- Arguments -----------------------------------------------------------
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    # ---- Environment ---------------------------------------------------------
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_parent + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    # ---- Gazebo --------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': '-r ' + sdf_file}.items(),
    )

    # ---- Robot state publisher -----------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
    )

    # ---- Bridges -------------------------------------------------------------
    bridge_main = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_main',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
            f'/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    bridge_sensors = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge_sensors',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ---- SLAM ----------------------------------------------------------------
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    # ---- Nav2 ----------------------------------------------------------------
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py'),
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
        }.items(),
    )

    # ---- RViz ----------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    # ---- rqt steering (manual override convenience) -------------------------
    robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        parameters=[{'use_sim_time': True}],
    )

    # ---- Joy node (gamepad) --------------------------------------------------
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'use_sim_time': True,
                     'autorepeat_rate': 20.0}],
        output='screen',
    )

    # ---- Part 2 application nodes -------------------------------------------
    mission_ctrl = Node(
        package='pioneer_part2',
        executable='mission_controller',
        name='mission_controller',
        parameters=[params_file,
                    {'waypoints_file': waypoints_file,
                     'use_sim_time': True}],
        output='screen',
    )

    waypoint_ctrl = Node(
        package='pioneer_part2',
        executable='waypoint_controller',
        name='waypoint_controller',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    cone_weaver = Node(
        package='pioneer_part2',
        executable='cone_weaver',
        name='cone_weaver',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    vision_detector = Node(
        package='pioneer_part2',
        executable='vision_detector',
        name='vision_detector',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    gamepad_ctrl = Node(
        package='pioneer_part2',
        executable='gamepad_controller',
        name='gamepad_controller',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    path_recorder = Node(
        package='pioneer_part2',
        executable='path_recorder',
        name='path_recorder',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # ---- Assembly ------------------------------------------------------------
    return LaunchDescription([
        gz_resource_path,
        rviz_arg,
        # Immediate
        gazebo,
        robot_state_publisher,
        bridge_main,
        # After sensors boot
        TimerAction(period=3.0, actions=[bridge_sensors]),
        # After world loads
        TimerAction(period=5.0, actions=[slam]),
        # Application nodes — after TF is available
        TimerAction(period=6.0, actions=[
            joy_node,
            waypoint_ctrl,
            cone_weaver,
            vision_detector,
            gamepad_ctrl,
            path_recorder,
            mission_ctrl,
            rviz,
            robot_steering,
        ]),
        # Nav2 — after SLAM is active
        TimerAction(period=8.0, actions=[nav2]),
    ])
