from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hfmc_pkg = get_package_share_directory('hybrid_force_motion_controller')
    controller_cfg = os.path.join(hfmc_pkg, 'config', 'controller.yaml')
    rviz_cfg = os.path.join(hfmc_pkg, 'config', 'hybrid_force_motion_hardware.rviz')

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for hybrid nodes')

    log_level = LaunchConfiguration('log_level')

    hybrid_node = Node(
        package='hybrid_force_motion_controller',
        executable='hybrid_force_motion_node',
        name='hybrid_force_motion_controller',
        output='screen',
        parameters=[controller_cfg],
        arguments=['--ros-args', '--log-level', log_level]
    )

    velocity_node = Node(
        package='hybrid_force_motion_controller',
        executable='cartesian_velocity_controller_node',
        name='cartesian_velocity_controller',
        output='screen',
        parameters=[controller_cfg],
        arguments=['--ros-args', '--log-level', log_level]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='hybrid_force_motion_rviz',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        log_level_arg,
        hybrid_node,
        velocity_node,
        rviz_node,
    ])
