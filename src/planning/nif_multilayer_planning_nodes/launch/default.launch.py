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
    nif_multilayer_planning_node = Node(
        package='nif_multilayer_planning_nodes',
        executable='nif_multilayer_planning_nodes_exe',
        output='screen',
        parameters=[],
        remappings={
            ('out_local_maptrack_inglobal', '/planning/path_global'),
            # ('in_vehicle_odometry', '/localization/ego_odom'),
            ('in_vehicle_odometry', '/sensor/odom_ground_truth'),
            # ('in_perception_result', '/perception/result'),
        }
    )

    return LaunchDescription([
        nif_multilayer_planning_node
    ])
