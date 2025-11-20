from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    hfmc_pkg = get_package_share_directory('hybrid_force_motion_controller')
    ur_sim_pkg = get_package_share_directory('ur_simulation_gz')

    controller_cfg = os.path.join(hfmc_pkg, 'config', 'controller.yaml')
    rviz_cfg = os.path.join(hfmc_pkg, 'config', 'hybrid_force_motion_sim.rviz')
    world_name = 'contact_dome'
    world_path = os.path.join(hfmc_pkg, 'worlds', f'{world_name}.sdf')

    ur_type_arg = DeclareLaunchArgument('ur_type', default_value='ur5e')

    existing_resource_path = os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
    new_resource_path = hfmc_pkg if not existing_resource_path else f"{hfmc_pkg}:{existing_resource_path}"
    existing_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    new_gz_path = hfmc_pkg if not existing_gz_path else f"{hfmc_pkg}:{existing_gz_path}"

    # 1. Include UR sim but disable its RViz; we will start our own
    ur_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ur_sim_pkg, 'launch', 'ur_sim_control.launch.py')),
        launch_arguments={
            'ur_type': LaunchConfiguration('ur_type'),
            'world_file': world_path,
            'rviz_config_file': rviz_cfg,
            'use_sim_time': 'true',
            'launch_rviz': 'false',  # Disable default RViz so we control it here
        }.items())

    # 2. Our RViz instance, explicitly using simulation time
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cfg],
        parameters=[{'use_sim_time': True}]
    )

    # 3. Bridge clock and set_entity_pose service
    service_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'/world/{world_name}/set_entity_pose@ros_gz_interfaces/srv/SetEntityPose',
        ])

    controller_node = Node(
        package='hybrid_force_motion_controller',
        executable='hybrid_force_motion_node',
        name='hybrid_force_motion_controller',
        output='screen',
        parameters=[controller_cfg])

    velocity_node = Node(
        package='hybrid_force_motion_controller',
        executable='cartesian_velocity_controller_node',
        name='cartesian_velocity_controller',
        output='screen',
        parameters=[controller_cfg])

    wrench_node = Node(
        package='ur_admittance_controller',
        executable='wrench_node',
        name='wrench_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        ur_type_arg,
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', new_resource_path),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', new_gz_path),
        ur_launch,
        service_bridge,
        controller_node,
        velocity_node,
        wrench_node,
        rviz_node,
    ])
