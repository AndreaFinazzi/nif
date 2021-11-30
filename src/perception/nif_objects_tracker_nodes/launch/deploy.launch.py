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

    config_file = (
        os.path.join(
            get_package_share_directory("nif_objects_tracker_nodes"),
            "config",
            "ros_tracking_param.yaml",
        ),
    )

    param_file_argument = DeclareLaunchArgument(
        'nif_objects_tracker_nodes_param_file',
        default_value=config_file,
        description='Path to config file for nif_objects_tracker_nodes'
    )

    objects_tracking_node = Node(
        package='nif_objects_tracker_nodes',
        executable='nif_objects_tracker_nodes_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('nif_objects_tracker_nodes_param_file')
        ]
    )

    return LaunchDescription([
        param_file_argument,
        objects_tracking_node
    ])
