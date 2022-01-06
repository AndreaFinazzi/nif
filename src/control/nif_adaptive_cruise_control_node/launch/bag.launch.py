import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    acc_rosparam_file = os.path.join(
        get_package_share_directory('nif_adaptive_cruise_control_node'),
        'config',
        'rosparam.yaml'
    )

    nif_adaptive_cruise_control_param = DeclareLaunchArgument(
        'nif_adaptive_cruise_control_param',
        default_value=acc_rosparam_file,
        description='Path to config file for nif_adaptive_cruise_control_param'
    )

    idm_acc_param_file = os.path.join(
        get_package_share_directory('nif_adaptive_cruise_control_node'),
        'config',
        'idm_acc_config.yaml'
    )

    idm_based_control_node = Node(
        package='nif_adaptive_cruise_control_node',
        executable='nif_adaptive_cruise_control_node_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('nif_adaptive_cruise_control_param'),
            {
                'idm_acc_config_file': idm_acc_param_file
            }
            ]
        remappings=[
            ('/oppo/prediction', '/oppo/prediction_dis'),
        ]
    )

    return LaunchDescription([
        nif_adaptive_cruise_control_param,
        idm_based_control_node])
