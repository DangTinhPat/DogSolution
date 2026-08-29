"""Launch the read-only real IMU pipeline: micro-ROS agent + Kalman + monitor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_megadog = get_package_share_directory('megadog_description')
    imu_filter_yaml = os.path.join(pkg_megadog, 'config', 'imu_filter.yaml')
    imu_filter_real_yaml = os.path.join(pkg_megadog, 'config', 'imu_filter_real.yaml')

    serial_dev = LaunchConfiguration('serial_dev')
    serial_baud = LaunchConfiguration('serial_baud')

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', serial_dev, '-b', serial_baud],
    )

    imu_kalman_filter = Node(
        package='imu_kalman_filter',
        executable='imu_kalman_node',
        output='screen',
        parameters=[imu_filter_yaml, imu_filter_real_yaml, {
            'input_source': 'compact',
            'compact_topic': '/imu/raw',
            'output_topic': '/imu/data',
            'frame_id': 'base_imu',
        }],
    )

    imu_monitor = Node(
        package='gui',
        executable='imu_monitor',
        output='screen',
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='921600'),
        micro_ros_agent,
        imu_kalman_filter,
        imu_monitor,
    ])
