"""Spawns A1 (urdf/robot.xacro) into Gazebo Harmonic with joint_state_broadcaster
and megadog_controller (OCS2 SqpMpc + HierarchicalWbc) active, and optionally
an RViz view of the same live simulation (RobotModel/TF plus megadog_wbc's
OCS2 desired/optimized trajectory and contact markers - see megadog.rviz).
Trimmed from babyDog's main_bot/launch/sim.launch.py (no imu_kalman_filter/
foot_contact_analyzer/leg_pd_controller - megaDog has no state-estimator or
separate low-level PD controller pipeline yet).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# See babyDog's main_bot/launch/sim.launch.py for why this is needed (snap-
# confined VS Code leaking SNAP_*/GTK_*/XDG_DATA_* vars that break gz sim's
# GUI dynamic linker).
_SNAP_LEAK_VARS = [
    'SNAP', 'SNAP_LIBRARY_PATH', 'SNAP_NAME', 'SNAP_DATA', 'SNAP_USER_DATA',
    'SNAP_USER_COMMON', 'SNAP_COMMON', 'SNAP_ARCH', 'SNAP_REVISION',
    'SNAP_INSTANCE_NAME', 'SNAP_CONTEXT', 'SNAP_COOKIE', 'SNAP_REAL_HOME',
    'SNAP_EUID', 'SNAP_UID', 'SNAP_LAUNCHER_ARCH_TRIPLET', 'SNAP_VERSION',
    'GTK_PATH', 'GTK_EXE_PREFIX', 'GTK_IM_MODULE_FILE',
    'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'GIO_MODULE_DIR',
    'GSETTINGS_SCHEMA_DIR', 'LOCPATH', 'XDG_DATA_DIRS', 'XDG_DATA_HOME',
]


def generate_launch_description():
    pkg_megadog = get_package_share_directory('megadog_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = PathJoinSubstitution([pkg_megadog, 'urdf', 'robot.xacro'])
    default_world = PathJoinSubstitution([pkg_megadog, 'worlds', 'megadog_world.sdf'])
    rviz_config = PathJoinSubstitution([pkg_megadog, 'rviz', 'megadog.rviz'])

    world = LaunchConfiguration('world')
    robot_type = LaunchConfiguration('robot_type')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    rviz = LaunchConfiguration('rviz')

    declare_world = DeclareLaunchArgument('world', default_value=default_world)
    declare_robot_type = DeclareLaunchArgument(
        'robot_type', default_value='a1',
        description='legged_unitree_description robot_type xacro:arg (a1/aliengo/go1) - only a1 meshes/const.xacro are vendored so far.'
    )
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='a1',
        description='Entity name spawned in Gazebo.'
    )
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument(
        'z', default_value='0.3',
        description='Spawn height, above devq/babyDog-scaled A1\'s ~0.19m standing height so it settles onto the ground.'
    )
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo server only (-s), without the Gazebo GUI.'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description=(
            'Also open RViz (RobotModel + TF from the live simulation, plus '
            "megadog_wbc's OCS2 desired/optimized trajectory and contact "
            'markers - see megadog.rviz). Pass rviz:=false to skip it.'
        )
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' robot_type:=', robot_type,
            ' megadog_share:=', pkg_megadog,
            ' megadog_description_share:=', pkg_megadog,
        ]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [
            world,
            PythonExpression([
                "' -s -r' if '", headless,
                "'.lower() in ['true', '1', 'yes', 'on'] else ' -r'"
            ]),
        ]}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', robot_name,
            '-x', x, '-y', y, '-z', z,
        ],
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_megadog, 'config', 'gz_bridge.yaml'),
            'use_sim_time': use_sim_time,
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
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    spawn_jsb_on_robot_spawned = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    spawn_megadog_controller_on_jsb_active = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[megadog_controller_spawner],
        )
    )

    unset_snap_vars = [UnsetEnvironmentVariable(var) for var in _SNAP_LEAK_VARS]

    return LaunchDescription([
        declare_world,
        declare_robot_type,
        declare_robot_name,
        declare_x,
        declare_y,
        declare_z,
        declare_use_sim_time,
        declare_headless,
        declare_rviz,
        *unset_snap_vars,
        gz_sim,
        gz_bridge,
        robot_state_publisher,
        spawn_robot,
        spawn_jsb_on_robot_spawned,
        spawn_megadog_controller_on_jsb_active,
        rviz_node,
    ])
