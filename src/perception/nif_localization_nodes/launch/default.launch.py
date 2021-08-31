import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # make sure the dbc file gets installed with the launch file
    # some_file = get_package_share_directory('nif_localization_nodes') + \
    #                 ""
    params_file_default = os.path.join(
        get_package_share_directory("nif_localization_nodes"),
        "config",
        "ecef_ref_lgsvl_sim.yaml",
    )

    params_file = DeclareLaunchArgument(
        'nif_localization_nodes_param_file',
        default_value=params_file_default)

    localization_node = Node(
        package='nif_localization_nodes',
        executable='nif_localization_nodes_exe',
        output='screen',
        # namespace='nif',
        # parameters=[LaunchConfiguration('nif_localization_nodes_param_file')],
        remappings=[
            ('gnss_01', '/novatel_top/inspva'),
            ('gnss_02', '/novatel_bottom/inspva'),
        ],
    )

    return LaunchDescription(
        [
            params_file,
            localization_node
        ])
