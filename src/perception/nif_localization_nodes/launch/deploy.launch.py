import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

IMS = 0
LOR = 1
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS":
    track = IMS
elif track_id == "LOR":
    track = LOR
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))


def generate_launch_description():
    ns = ""
    track_subdir = ""

    if track == LOR:
        track_subdir = "lor"
    elif track == IMS:
        track_subdir = "ims"
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))


    global_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            track_subdir,
            "full_map.pcd", #IMS
        )    
    outer_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            track_subdir,
            "outer_map.pcd" # IMS
            # "LOR_wall.pcd", #LOR
        )
    inner_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            track_subdir,
            "inner_map.pcd", #IMS
            # "LOR_inner.pcd", #LOR
        )



    localization_node = Node(
                package="nif_localization_nodes",
                executable="nif_localization_nodes_exe",
                name="nif_localization_nodes",
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



