from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
import os

#enable_cameras  = False
#enable_radar    = False

def generate_launch_description():
    # get package directory
    pkg_dir = get_package_share_directory('bvs_common')

    # get vehicle id for IP addresses
    vehicle_id = os.environ.get('VEHICLE_ID')
    if vehicle_id is None or not vehicle_id.isnumeric():
        vehicle_id = '4'
    
    vehicle_id = vehicle_id.strip()

    ip_gps_bottom = "10.42.{}.60".format(vehicle_id)
    ip_gps_top = "10.42.{}.61".format(vehicle_id)
    ip_gps_port = "3001"

    novatel_bottom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/sensors/oem7_net_bottom.launch.py'
        ),
        launch_arguments={
		"oem7_ip_addr": ip_gps_bottom,
		"oem7_port": ip_gps_port
            }.items()
    )

    novatel_top_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/sensors/oem7_net_top.launch.py'
        ),
        launch_arguments={
		"oem7_ip_addr": ip_gps_top,
		"oem7_port": ip_gps_port
            }.items()
    )

    launch_description = [
        novatel_bottom_launch,
        novatel_top_launch,
    ]

    return LaunchDescription(launch_description)
