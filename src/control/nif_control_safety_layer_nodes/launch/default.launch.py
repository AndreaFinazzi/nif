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

    return LaunchDescription([
        control_safety_layer_node
    ])