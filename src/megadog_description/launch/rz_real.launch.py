"""RViz-only view for real megaDog hardware.

Real micro-ROS firmware is built for DDS domain 0, so this launch pins RViz
to ROS_DOMAIN_ID=0 as well. The fixed frame is "base" because safe real
bring-up can run without megadog_controller and therefore without odom->base.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


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
    rviz_config = PathJoinSubstitution([pkg_megadog, 'rviz', 'megadog_real.rviz'])

    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        *[UnsetEnvironmentVariable(var) for var in _SNAP_LEAK_VARS],
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_real',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
        ),
    ])
