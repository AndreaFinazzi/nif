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
    long_control_param_file = get_share_file(
        package_name='long_control', file_name='config/params.yaml'
    )

    long_control_param = DeclareLaunchArgument(
        'long_control_param_file',
        default_value=long_control_param_file,
        description='Path to config file for long_control'
    )

    kin_control_param_file = get_share_file(
        package_name='nif_control_minimal_nodes', file_name='config/params.yaml'
    )

    kin_control_param = DeclareLaunchArgument(
        'nif_control_minimal_param_file',
        default_value=kin_control_param_file,
        description='Path to config file for kin_control'
    )

    lat_control_node = Node(
        package='nif_control_minimal_nodes',
        executable='nif_control_minimal_nodes_exe',
        output='screen',
        parameters=[LaunchConfiguration('nif_control_minimal_param_file')],
        remappings={
            ('out_control_cmd', '/control_pool/control_cmd')
        }
    )

    long_control_node = Node(
        package='long_control',
        executable='long_control',
        output='screen',
        parameters=[LaunchConfiguration('long_control_param_file')],
    )

    control_safety_layer_node = Node(
        package='nif_control_safety_layer_nodes',
        executable='control_safety_layer_node_exe',
        output='screen',
        parameters=[LaunchConfiguration('nif_control_minimal_param_file')],
        remappings=[
            ('in_control_cmd', '/control_pool/control_cmd'),
            ('out_control_cmd', '/control_safety_layer/out/control_cmd'),
            ('out_steering_control_cmd', '/raptor_dbw_interface/steering_cmd'),
            ('out_accelerator_control_cmd', '/raptor_dbw_interface/accelerator_pedal_cmd'),

        ]
    )

    path_server_node = Node(
        package='path_server',
        executable='path_server',
        output='screen',
    )

    return LaunchDescription([
        kin_control_param,
        long_control_param,
        lat_control_node,
        long_control_node,
        control_safety_layer_node,
        path_server_node
    ])
