from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # params_file = LaunchConfiguration(
    #     'params',
    #     default=[ThisLaunchFileDir(), '/launch_params.yaml'])

    # make sure the dbc file gets installed with the launch file
    # some_file = get_package_share_directory('nif_localization_nodes') + \
    #                 ""

    # config = (
    #     os.path.join(
    #         get_package_share_directory('lgsvl_simulation'),
    #         "config",
    #         "lgsvl_sim_param.yaml",
    #     ),
    #     )

    lgsvl_sim_subscriber_node = Node(
                package='nif_lgsvl_simulation',
                executable='subscriber_member_function.py',
                output='screen',
                namespace='nif',
                # parameters=[config]
    )
    lgsvl_sim_publisher_node = Node(
                package='nif_lgsvl_simulation',
                executable='publisher_member_function.py',
                output='screen',
                namespace='nif',
                # parameters=[config]
    )
    lgsvl_sim_controller_node = Node(
                package='nif_lgsvl_simulation',
                executable='controller.py',
                output='screen',
                namespace='nif',
                # parameters=[config]
    )
    return LaunchDescription(
        [
            lgsvl_sim_subscriber_node,
            lgsvl_sim_publisher_node
        ])
