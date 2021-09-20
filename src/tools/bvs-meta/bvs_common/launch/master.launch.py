import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # get package directory
    pkg_dir = get_package_share_directory('bvs_common')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/robot_description.launch.py'
        )
    )

    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/vehicle_autonomous_lat_long.launch.py'
        )
    )

    launch_description = [
        robot_description_launch,
        vehicle_launch
    ]

    return LaunchDescription(launch_description) #, param_declarations])
