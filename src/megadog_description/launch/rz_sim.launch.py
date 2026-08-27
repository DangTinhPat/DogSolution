"""RViz only - opens megadog.rviz without launching Gazebo/robot_state_publisher/
controller_manager itself. Meant to run alongside sim.launch.py (started
separately, e.g. rviz:=false from the GUI's own Sim button) so RViz can be
closed and reopened without restarting the whole simulation; it just sits and
displays whatever /tf, /robot_description and /legged_robot/... topics are
already on the ROS graph. Ported from babyDog's main_bot/launch/rz_sim.launch.py.

Note: RobotModel's Description Topic uses QoS Volatile, while
robot_state_publisher publishes /robot_description transient_local exactly
once at startup - so if this is opened well after sim.launch.py, the robot
mesh may never appear (TF/OCS2 markers are unaffected, they don't depend on
/robot_description).
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, UnsetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Snap-confined apps leak SNAP_*/GTK_*/XDG_DATA_* into terminals spawned from
# them, crashing rviz2 (Qt) - same fix as sim.launch.py.
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
    rviz_config = PathJoinSubstitution([pkg_megadog, 'rviz', 'megadog.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='true (default) - matches Gazebo (/clock) from sim.launch.py.'
    )

    node_rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    unset_snap_vars = [UnsetEnvironmentVariable(var) for var in _SNAP_LEAK_VARS]

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    for action in unset_snap_vars:
        ld.add_action(action)
    ld.add_action(node_rviz2)

    return ld
