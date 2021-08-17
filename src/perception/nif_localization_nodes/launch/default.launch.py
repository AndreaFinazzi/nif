from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # params_file = LaunchConfiguration(
    #     'params',
    #     default=[ThisLaunchFileDir(), '/launch_params.yaml'])

    # make sure the dbc file gets installed with the launch file
    # some_file = get_package_share_directory('nif_localization_nodes') + \
    #                 ""

    return LaunchDescription(
        [
            Node(
                package='nif_localization_nodes',
                executable='localization_node_exe',
                output='screen',
                namespace='nif',
                remappings=[
                    ('gnss_01', '/novatel_top/inspva'),
                    ('gnss_02', '/novatel_bottom/inspva'),
                ],
            ),
        ])
