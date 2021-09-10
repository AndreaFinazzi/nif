import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_dir = get_package_share_directory('nif_launch')
    pkg_dir_robot_description = get_package_share_directory('av21_description')
    # pkg_dir_localization = get_package_share_directory('nif_localization_nodes')
    pgk_dir_lgsvl_simulation = get_package_share_directory('nif_lgsvl_simulation')

    nif_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/deploy/nif_base.launch.py'
        ),
    )

    vehicle_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/deploy/vehicle_interface.launch.py'
        ),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/deploy/localization.launch.py'
        ),
    )

    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/deploy/planning.launch.py'
        ),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/deploy/control.launch.py'
        ),
    )

    launch_description = [
        nif_base_launch,
        vehicle_interface_launch,
        localization_launch,
        planning_launch,
        control_launch
    ]

    return LaunchDescription(launch_description)
