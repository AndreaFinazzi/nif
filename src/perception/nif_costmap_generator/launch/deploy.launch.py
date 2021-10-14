import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    params_file = os.path.join(
            get_package_share_directory("nif_costmap_generator"),
            "config",
            "config.yaml"
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        'nif_costmap_param_file',
        default_value=params_file,
        description='nif_costmap_param_file'
    )

    costmap_generator_node =  Node(
                package="nif_costmap_generator",
                executable="nif_costmap_generator_exe",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('nif_costmap_param_file')
                ],
                remappings=[
                    ("in_odometry_ekf_estimated", "/aw_localization/ekf/odom"), 
                    ("in_wall_points", "/ransac_filtered_points/both"), 
                    ("in_object_points", "/clustered_points"), 
                    ("in_fake_obs_points", "/graph_planner/fake_obs"),
                    ("out_occupancy_map", "/semantics/costmap_generator/occupancy_grid"), 
                ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            costmap_generator_node
        ])
    

