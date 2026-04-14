# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    pkg_p3at = get_package_share_directory('p3at')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Parent of the p3at share folder so Gazebo can resolve model://p3at/...
    gz_resource_parent = os.path.dirname(pkg_p3at)

    sdf_file = os.path.join(pkg_p3at, 'worlds', 'basic_urdf.sdf')
    robot_file = os.path.join(pkg_p3at, 'robots', 'pioneer.urdf')

    with open(robot_file, 'r') as infp:
        robot_desc = infp.read()

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz.'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={'gz_args': sdf_file}.items(),
    )


    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Launch rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # arguments=['-d', os.path.join(project_path, 'rviz', 'vehicle.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': True},
        ]
    )

    # Since basic_urdf.sdf now includes the pioneer.urdf, we don't need a separate spawn process.
    # However, to be consistent with common ROS2-Gazebo patterns, we keep robot_state_publisher.

    # ROS–Gazebo bridges: split so IMU/camera reliably register (single parameter_bridge
    # with many entries sometimes only creates the first few on some setups).
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
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
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

    # A gui tool for easy tele-operation.
    robot_steering = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        parameters=[{'use_sim_time': True}]
    )

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=gz_resource_parent + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    return LaunchDescription([
        gz_resource_path,
        rviz_launch_arg,
        gazebo,
        # robot, # Removed spawn since it's already in basic_urdf.sdf
        robot_state_publisher,
        bridge_main,
        # Start after Gazebo/sensors exist (avoids empty subscriptions at process start).
        TimerAction(period=3.0, actions=[bridge_sensors]),
        rviz,
        robot_steering
    ])
