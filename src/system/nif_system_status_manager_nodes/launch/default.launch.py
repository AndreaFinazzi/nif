import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    # nif_ssm_param_default_file = os.path.join(
    #     get_package_share_directory('nif_system_status_manager_nodes'),
    #     'config',
    #     'params.yaml'
    # )
    #
    # nif_ssm_param_file = DeclareLaunchArgument(
    #     'nif_system_status_manager_nodes',
    #     default_value=nif_ssm_param_default_file,
    #     description='Path to config file for system status manager'
    # )

    system_status_manager_node = Node(
        package='nif_system_status_manager_nodes',
        executable='nif_system_status_manager_nodes_exe',
        remappings=[
            ('in_joystick_cmd', '/joystick/command'),
            ('in_novatel_bestpos', '/novatel_bottom/bestpos'),
            ('in_novatel_insstdev', '/novatel_bottom/insstdev'),
            ('in_localization_error', '/aw_localization/ekf/error'),
            ('in_mission_status', '/system/mission'),
            ('out_system_status', '/system/status'),
        ]
    )

    return LaunchDescription([
        system_status_manager_node
    ])
