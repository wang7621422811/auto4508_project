"""Launch: Gazebo sim + SLAM + Nav2 full stack for Pioneer 3-AT."""

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

    pkg_p3at = get_package_share_directory('p3at')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    gz_resource_parent = os.path.dirname(pkg_p3at)
    sdf_file = os.path.join(pkg_p3at, 'worlds', 'basic_urdf.sdf')
    robot_file = os.path.join(pkg_p3at, 'robots', 'pioneer.urdf')
    nav2_params = os.path.join(pkg_p3at, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_p3at, 'config', 'slam_params.yaml')

    with open(robot_file, 'r') as f:
        robot_desc = f.read()

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_parent + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': '-r ' + sdf_file}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
    )

    # --- Bridges (split for reliability) ---
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

    # --- SLAM (lifecycle-managed via slam_toolbox's own launch) ---
    """
        在RViz中，无法看到地图。
        问题清楚了：async_slam_toolbox_node 是一个 lifecycle 节点，需要经过 configure → activate 才能工作。官方 launch 文件用 LifecycleNode + EmitEvent(ChangeState) 来自动完成这两步。我们的 launch 用了普通 Node，所以它一直停留在 unconfigured 状态。

        修复方案：用 IncludeLaunchDescription 引入 slam_toolbox 官方的 online_async_launch.py，它会自动处理 lifecycle 转换。
    """
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py'),
        ),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time': 'true',
        }.items(),
    )

    # --- Nav2 ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py'),
        ),
        launch_arguments={
            'params_file': nav2_params,
            'use_sim_time': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}],
    )

    robot_steering = Node(
        package='rqt_robot_steering',
        executable='rqt_robot_steering',
        parameters=[{'use_sim_time': True}],
    )

    waypoint_ctrl = Node(
        package='p3at',
        executable='waypoint_controller',
        name='waypoint_controller',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    path_recorder = Node(
        package='p3at',
        executable='path_recorder',
        name='path_recorder',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        gz_resource_path,
        rviz_arg,
        gazebo,
        robot_state_publisher,
        bridge_main,
        TimerAction(period=3.0, actions=[bridge_sensors]),
        TimerAction(period=5.0, actions=[slam]),
        TimerAction(period=8.0, actions=[nav2]),
        TimerAction(period=6.0, actions=[waypoint_ctrl]),
        TimerAction(period=6.0, actions=[path_recorder]),
        rviz,
        robot_steering,
    ])
