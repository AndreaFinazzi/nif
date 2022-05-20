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
        "imitation_planner_vis_config.yaml",
    )

    param_file_launch_arg = DeclareLaunchArgument(
        "nif_costmap_param_file",
        default_value=params_file,
        description="nif_costmap_param_file",
    )

    costmap_generator_node = Node(
        package="nif_costmap_generator",
        executable="imitative_planner_visualizer_exe",
        output="screen",
        emulate_tty=True,
        parameters=[LaunchConfiguration("nif_costmap_param_file")],
        remappings=[
            ("in_predicted_samples", "imitative/out_path/not_used"),
            ("out_imitation_distribution_map", "imitative/distribution"),
            ("in_predicted_samples_marker_array", "imitative/samples"),
        ],
    )

    return LaunchDescription([param_file_launch_arg, costmap_generator_node])
