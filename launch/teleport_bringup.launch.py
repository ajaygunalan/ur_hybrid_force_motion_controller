from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ur_sim_pkg = get_package_share_directory('ur_simulation_gz')
    hfmc_pkg = get_package_share_directory('hybrid_force_motion_controller')

    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur5e')
    world_path = os.path.join(hfmc_pkg, 'worlds', 'contact_dome.sdf')

    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    new_resource_path = hfmc_pkg if not existing_resource_path else f"{hfmc_pkg}:{existing_resource_path}"
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = hfmc_pkg if not existing_gz_path else f"{hfmc_pkg}:{existing_gz_path}"

    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_sim_pkg, 'launch', 'ur_sim_control.launch.py')),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'world_file': world_path,
        }.items())

    ee_teleporter = Node(
        package='hybrid_force_motion_controller',
        executable='ee_teleporter.py',
        output='screen',
        parameters=[{
            'model_name': 'ur5e',
            'base_frame': 'base_link',
            'tool_frame': 'tool0',
        }])

    return LaunchDescription([
        ur_type_arg,
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', new_resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_path),
        ur_launch,
        ee_teleporter,
    ])
