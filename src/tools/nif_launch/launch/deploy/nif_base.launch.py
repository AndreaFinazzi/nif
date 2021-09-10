import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('nif_launch')
    pkg_dir_robot_description = get_package_share_directory('av21_description')
    # pkg_dir_localization = get_package_share_directory('nif_localization_nodes')
    pgk_dir_lgsvl_simulation = get_package_share_directory('nif_lgsvl_simulation')

    nif_global_parameters_file = os.path.join(
        get_package_share_directory('nif_launch'),
        'config',
        'deploy',
        'params.global.0.yaml'
    )

    global_parameters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_common_nodes') + '/launch/parameters.launch.py'
        ),
        launch_arguments={
            'nif_global_parameters_file': nif_global_parameters_file
        }.items()
    )

    system_status_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_system_status_manager_nodes') + '/launch/default.launch.py'
        ),
    )

    safety_node = Node(
        package='bvs_safety',
        executable='safety_executive_node',
        name='safety_executive_node',
        output='screen'
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_robot_description + '/launch/default.launch.py'
        )
    )

    return LaunchDescription([
        global_parameters_launch,
        system_status_manager_launch,
        safety_node,
        robot_description_launch,
    ])
