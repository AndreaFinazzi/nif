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

    control_safety_layer_param_file = get_share_file(
        package_name='nif_control_safety_layer_nodes', file_name='config/deploy.yaml'
    )

    csl_param_argument = DeclareLaunchArgument(
        'control_safety_layer_param_file',
        default_value=control_safety_layer_param_file,
        description='Path to config file for csl'
    )

    control_safety_layer_node = Node(
        package='nif_control_safety_layer_nodes',
        executable='nif_control_safety_layer_nodes_exe',
        output='screen',
        parameters=[LaunchConfiguration('control_safety_layer_param_file')],
        remappings=[
            ('in_control_cmd', '/control_pool/control_cmd'),
            ('in_override_control_cmd', '/control_pool/override_cmd'),
            ('out_control_cmd', '/control_safety_layer/out/control_cmd'),
            ('out_steering_control_cmd', '/joystick/steering_cmd'),
            ('out_accelerator_control_cmd', '/joystick/accelerator_cmd'),
            ('out_braking_control_cmd', '/joystick/brake_cmd'),
            ('out_gear_control_cmd', '/joystick/gear_cmd'),
        ]
    )

    return LaunchDescription([
        csl_param_argument,
        control_safety_layer_node
    ])
