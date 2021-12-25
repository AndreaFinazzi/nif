import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    points_clustering_node =  Node(
                package="nif_radar_clustering_nodes",
                executable="aptiv_esr_interface_exe",
                output="screen",
                respawn=True,
                emulate_tty=True,
                remappings=[
                    ("in_radar_track", "/radar_front/esr_track/filtered"),
                    ("out_perception_list", "out_perception_list"),
                    ("out_track_list", "out_track_list"),
                    
                ]
    )

    return LaunchDescription(
        [
            points_clustering_node
        ])
    

