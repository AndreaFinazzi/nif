
  
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
IMS_SIM = 2
LVMS = 3
LVMS_SIM = 4
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS" or track_id == "ims":
    track = IMS
elif track_id == "LOR" or track_id == "lor":
    track = LOR
elif track_id == "IMS_SIM" or track_id == "ims_sim":
    track = IMS_SIM
elif track_id == "LVMS" or track_id == "lvms":
    track = LVMS
elif track_id == "LVMS_SIM" or track_id == "lvms_sim":
    track = LVMS_SIM
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))

def generate_launch_description():
    ns = ""
    track_subdir = ""

    if track == LOR:
        track_subdir = "lor"
    elif track == IMS:
        track_subdir = "ims"
    elif track == LVMS:
        track_subdir = "lvms"
    elif track == LVMS_SIM:
        track_subdir = "lvms_sim"
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    global_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            track_subdir,
            "full_map.pcd",
        )    
    outer_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            track_subdir,
            "outer_map.pcd" 
        )
    inner_geofence_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            track_subdir,
            "inner_map.pcd",
        )

    trajectory_map = os.path.join(
            get_package_share_directory("nif_localization_nodes"),
            "map",
            track_subdir,
            "trajectory.pcd", 
        )
    # TOOO +++++ INCLUDE WALL DETECTION (UNWIRED) + TOPICS FOR ROSBAGS


    # TOOO +++++ INCLUDE WALL DETECTION (UNWIRED) + TOPICS FOR ROSBAGS

    localization_node = Node(
                package="nif_localization_nodes",
                executable="nif_localization_nodes_exe",
                output="screen",
                emulate_tty=True,
                namespace=ns,
                parameters=[{
                    # global map loader
                    # 'globalmap_file_name' : global_map,
                    'trajectory_pcd_file' : trajectory_map,
                    'use_trajectory' : True,

                    # geofence node
                    'outer_geofence_filename': outer_geofence_map,
                    'inner_geofence_filename': inner_geofence_map,
                    'outer_geofence_bias': -0.5,
                    'inner_geofence_bias': 0.0,
                    'distance_low_pass_filter': 0.5,
                    
                    # resilient localization node
                    'thres_for_distance_error_flag' : 1.0,
                    'thres_for_distance_to_wall' : 2.0,
                    }],

                remappings=[
                    ("in_odometry_ekf_estimated", "/aw_localization/ekf/odom"),
                ]
            )

    return LaunchDescription(
        [
            localization_node
        ]
    )



