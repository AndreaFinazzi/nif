from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():

    params_file = os.path.join(
            get_package_share_directory("nif_control_pure_pursuit_nodes"),
            "config",
            "default.yaml"
        )
    # make sure the dbc file gets installed with the launch file
    # some_file = get_package_share_directory('nif_localization_nodes') + \
    #                 ""
    param_file_launch_arg = DeclareLaunchArgument(
        'nif_control_pure_pursuit_param_file',
        default_value=params_file,
        description='Path to config file for pure pursuit control'
    )

    control_node = Node(
        package='nif_control_pure_pursuit_nodes',
        executable='nif_control_pure_pursuit_nodes_exe',
        output='screen',
        # namespace='nif',
        parameters=[
            LaunchConfiguration('nif_control_pure_pursuit_param_file')
        ],
        remappings=[
            ('topic_ego_odometry', 'localization/ego_odom'),
            # ('target_path', 'wpt_manager/maptrack_path/global'),
            ('target_path', 'planning/path_global'),
            ('out_control_cmd', 'control_pool/control_cmd'),
        ],
    )

    return LaunchDescription([
            param_file_launch_arg,
            control_node
        ])
