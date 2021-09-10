import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    global_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            "ims",
            "indy_full.pcd", #IMS
        )    
    outer_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            "ims",
            "wall.pcd" # IMS
            # "LOR_wall.pcd", #LOR
        )
    inner_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            "ims",
            "inner.pcd", #IMS
            # "LOR_inner.pcd", #LOR
        )
    ns = ""

    localization_node = Node(
                package="nif_localization_nodes",
                executable="nif_localization_nodes_exe",
                output="screen",
                emulate_tty=True,
                namespace=ns,
                parameters=[{
                    'globalmap_file_name' : global_map,
                    'outer_geofence_filename': outer_geofence_map,
                    'inner_geofence_filename': inner_geofence_map,
                    # 'origin_lat' : 39.809786,
                    # 'origin_lon' : -86.235148,
                    }],

                remappings=[
                    ("in_inspva", "novatel_bottom/inspva"),
                    ("in_bestpos", "novatel_bottom/bestpos"),
                    ("in_imu", "novatel_bottom/imu/data"),
                    ("in_wheel_speed_report", "raptor_dbw_interface/wheel_speed_report"),
                ]
            )

    return LaunchDescription(
        [
            localization_node
        ]
    )



