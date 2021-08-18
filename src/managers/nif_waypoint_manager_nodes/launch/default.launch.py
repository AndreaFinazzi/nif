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
    # params_file = LaunchConfiguration(
    #     'params',
    #     default=[ThisLaunchFileDir(), '/launch_params.yaml'])

    # make sure the dbc file gets installed with the launch file
    # some_file = get_package_share_directory('nif_localization_nodes') + \
    #                 ""
    config = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "default.yaml",
        ),
    )

    return LaunchDescription(
        [
            Node(
                package='nif_waypoint_manager_nodes',
                executable='waypoint_manager_node_exe',
                output='screen',
                # namespace='nif',
                # parameters=[config],
                parameters=[
                    {
                        'file_path_list': [
                            '/home/sw/src/managers/nif_waypoint_manager_nodes/maps/lgsvl_sim/wpt_manual_points.csv',
                            '/home/sw/src/managers/nif_waypoint_manager_nodes/maps/lgsvl_sim/wpt_manual_points.csv'
                        ]
                    },
                ],
                remappings=[
                    ('topic_ego_odometry', 'localization/ego_odom')
                ],

            ),
        ])
