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
        veh_id = '4'

    print("LAUNCHING WITH VEHICLE ID: " + vehicle_id)
    fp_lidar_front         = "10" + vehicle_id.strip() + "20"
    fp_lidar_right         = "10" + vehicle_id.strip() + "21"
    fp_lidar_left          = "10" + vehicle_id.strip() + "22"
    ip_gps_bottom          ="10.42." + vehicle_id.strip() + ".60"
    ip_gps_top             ="10.42." + vehicle_id.strip() + ".61"
    ip_camera_left         ="10.42." + vehicle_id.strip() + ".50"
    ip_camera_left_front   ="10.42." + vehicle_id.strip() + ".51"
    ip_camera_right_front  ="10.42." + vehicle_id.strip() + ".52"
    ip_camera_right        ="10.42." + vehicle_id.strip() + ".53"
    ip_camera_right_rear   ="10.42." + vehicle_id.strip() + ".54"
    ip_camera_left_rear    ="10.42." + vehicle_id.strip() + ".55"
    ip_gps_port            ="3001"

    enable_cameras_arg = DeclareLaunchArgument(
        'enable_cameras', default_value=TextSubstitution(text="False")
    )
    enable_radars_arg = DeclareLaunchArgument(
        "enable_radars", default_value=TextSubstitution(text="False")
    )
    enable_lidars_arg = DeclareLaunchArgument(
        "enable_lidars", default_value=TextSubstitution(text="False")
    )

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

    novatel_bottom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/oem7_net_bottom.launch.py'
        ),
        launch_arguments={
		"oem7_ip_addr": ip_gps_bottom,
		"oem7_port": ip_gps_port
            }.items()
    )

    novatel_top_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/oem7_net_top.launch.py'
        ),
        launch_arguments={
		"oem7_ip_addr": ip_gps_top,
		"oem7_port": ip_gps_port
            }.items()
    )

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/cameras.launch.py'
        ),
        launch_arguments={
                "ip_camera_left": ip_camera_left,
                "ip_camera_left_front": ip_camera_left_front,
                "ip_camera_right_front": ip_camera_right_front,
                "ip_camera_right": ip_camera_right,
                "ip_camera_right_rear": ip_camera_right_rear,
                "ip_camera_left_rear": ip_camera_left_rear,
            }.items(),
    	condition=IfCondition(LaunchConfiguration('enable_cameras'))
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/lidars.launch.py'
        ),
        launch_arguments={
                "fp_lidar_front": fp_lidar_front,
                "fp_lidar_left": fp_lidar_left,
                "fp_lidar_right": fp_lidar_right,
            }.items(),
    	condition=IfCondition(LaunchConfiguration('enable_lidars'))
    )

    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/radar.launch.py'
        ),
    	condition=IfCondition(LaunchConfiguration('enable_radars'))
    )

    launch_description = [
        enable_cameras_arg,
        enable_radars_arg,
        enable_lidars_arg,
        robot_description_launch,
        vehicle_launch,
        novatel_bottom_launch,
        # novatel_top_launch,
        cameras_launch,
        radar_launch,
        lidar_launch
    ]

    return LaunchDescription(launch_description) #, param_declarations])
