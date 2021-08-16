import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    pkg_dir = get_package_share_directory('nif_launch')
    pkg_dir_iac_launch = get_package_share_directory('iac_launch')
    pkg_dir_localization = get_package_share_directory('nif_localization_nodes')

    nif_global_parameters_file = os.path.join(
        get_package_share_directory('nif_launch'),
        'config',
        'params.global.yaml'
    )

    global_parameters_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_common_nodes') + '/launch/parameters.launch.py'
        ),
        launch_arguments={
            'nif_global_parameters_file': nif_global_parameters_file
            }.items()
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('iac_launch') + '/launch/robot_description.launch.py'
        )
    )

    csl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_control_safety_layer_nodes') + '/launch/default.launch.py'
        ),
        
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/control.launch.py'
        ),
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/planning.launch.py'
        ),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_localization_nodes') + '/launch/localization_node_gtsam_sim.launch.py'
        )
    )

    return LaunchDescription([
        global_parameters_node,
        robot_description_launch,
        csl_launch,
        control_launch,
        planning_launch,
        localization_launch
    ])
