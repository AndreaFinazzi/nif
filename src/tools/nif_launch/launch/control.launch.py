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
    # long_control_param_file = get_share_file(
    #     package_name='long_control', file_name='config/params.yaml'
    # )
    #
    # long_control_param = DeclareLaunchArgument(
    #     'long_control_param_file',
    #     default_value=long_control_param_file,
    #     description='Path to config file for long_control'
    # )

    nif_control_minimal_param_file = get_share_file(
        package_name='nif_control_minimal_nodes', file_name='config/params.yaml'
    )

    nif_control_minimal_param = DeclareLaunchArgument(
        'nif_control_minimal_param_file',
        default_value=nif_control_minimal_param_file,
        description='Path to config file for nif_control_minimal'
    )

    lat_control_node = Node(
        package='nif_control_minimal_nodes',
        executable='nif_control_minimal_nodes_exe',
        output='screen',
        parameters=[LaunchConfiguration('nif_control_minimal_param_file')],
        remappings={
            ('in_control_cmd_prev', '/control_safety_layer/out/control_cmd'),
            ('out_control_cmd', '/control_pool/control_cmd')
        }
    )
    
    # long_control_node = Node(
    #     package='long_control',
    #     executable='long_control',
    #     output='screen',
    #     parameters=[LaunchConfiguration('long_control_param_file')],
    # )

    return LaunchDescription([
        nif_control_minimal_param,
        # long_control_param,
        lat_control_node,
        # long_control_node,
    ])
