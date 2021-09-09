import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    outer_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            "wall.pcd" # IMS
            # "LOR_wall.pcd", #LOR
        )
    inner_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            "inner.pcd", #IMS
            # "LOR_inner.pcd", #LOR
        )
    ns = ""
    return LaunchDescription(
        [
            Node(
                package="nif_localization_nodes",
                executable="nif_localization_nodes_exe",
                name="nif_localization_nodes",
                output="screen",
                emulate_tty=True,
                namespace=ns,
                parameters=[{
                    'config': config,
                    'outer_geofence_filename': outer_geofence_map,
                    'inner_geofence_filename': inner_geofence_map}],

                remappings=[
                    ("ins_gps", "novatel_bottom/inspva_"),
                    ("bestpos_gps", "novatel_bottom/bestpos_"),
                    ("ins_oem", "novatel_bottom/inspva"),
                    ("bestpos_oem", "novatel_bottom/bestpos"),
                    ("gps", "novatel_bottom/fix"),
                    ("imu_novatel", "novatel_bottom/raw_imu"),
                    ("imu", "novatel_bottom/imu/data"),
                ]
            ),
            
        ]
    )

