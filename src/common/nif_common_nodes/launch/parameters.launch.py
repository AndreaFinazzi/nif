import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    nif_global_parameters_default_file = os.path.join(
        get_package_share_directory('nif_common_nodes'),
        'config',
        'params.global.yaml'
    )

    nif_global_parameters_file = DeclareLaunchArgument(
        'nif_global_parameters_file',
        default_value=nif_global_parameters_default_file,
        description='Path to config file for global parameters'
    )

    global_param_node = Node(
        package='nif_common_nodes',
        executable='nif_global_parameters_node_exe',
        name='global_parameters_node',
        parameters=[
            LaunchConfiguration('nif_global_parameters_file')
            ]
    )

    return LaunchDescription([
        nif_global_parameters_file,
        global_param_node
    ])