import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    params_file = os.path.join(
            get_package_share_directory("nif_points_clustering_nodes"),
            "config",
            "config.yaml"
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        
        'nif_points_clustering_param_file',
        default_value=params_file,
        description='nif_points_clustering_param_file'
    )

    points_clustering_node =  Node(
                package="nif_points_clustering_nodes",
                executable="nif_points_clustering_nodes_exe",
                output="screen",
                respawn=True,
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('nif_points_clustering_param_file')
                ],
                remappings=[
                    ("in_lidar_points", "/luminar_front_points"),
                    ("in_left_points", "/luminar_left_points"),
                    ("in_right_points", "/luminar_right_points"),
                    ("out_clustered_points", "/clustered_points"),
                    ("out_inflation_points", "/inflated_points"),
                    ("out_clustered_center_points", "/cluster_center_points"),
                    ("out_clustered_array", "/clustered_markers"),
                    ("out_clustered_array", "/clustered/perception_list")    
                ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            points_clustering_node
        ])
    

