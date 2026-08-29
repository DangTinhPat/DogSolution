"""Launch megaDog real hardware through ros2_control and micro-ROS."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_megadog = get_package_share_directory('megadog_description')

    xacro_file = PathJoinSubstitution([pkg_megadog, 'urdf', 'robot.xacro'])
    controllers_yaml = os.path.join(pkg_megadog, 'config', 'controllers_real.yaml')
    imu_filter_yaml = os.path.join(pkg_megadog, 'config', 'imu_filter.yaml')
    imu_filter_real_yaml = os.path.join(pkg_megadog, 'config', 'imu_filter_real.yaml')

    serial_dev = LaunchConfiguration('serial_dev')
    serial_baud = LaunchConfiguration('serial_baud')
    start_controller = LaunchConfiguration('start_controller')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' robot_type:=a1',
            ' sim:=false',
            ' real_hardware:=true',
            ' megadog_share:=', pkg_megadog,
            ' megadog_description_share:=', pkg_megadog,
        ]),
        value_type=str,
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['serial', '--dev', serial_dev, '-b', serial_baud],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}, controllers_yaml],
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

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster', '--switch-timeout', '30'],
    )

    megadog_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['megadog_controller', '--switch-timeout', '30'],
        condition=IfCondition(start_controller),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        DeclareLaunchArgument('serial_dev', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('serial_baud', default_value='921600'),
        DeclareLaunchArgument(
            'start_controller',
            default_value='false',
            description='Spawn megadog_controller. Default false is safer for first real-hardware bring-up.',
        ),
        micro_ros_agent,
        imu_kalman_filter,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        megadog_controller_spawner,
    ])
