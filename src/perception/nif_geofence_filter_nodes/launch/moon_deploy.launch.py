
  
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
        track_subdir = "LOR"
    elif track == IMS:
        track_subdir = "IMS"
    elif track == IMS_SIM:
        track_subdir = "IMS_SIM"
    elif track == LVMS:
        track_subdir = "LVMS"
    elif track == LVMS_SIM:
        track_subdir = "LVMS_SIM"
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    file_path_inner_line = os.path.join(
                get_package_share_directory("nif_waypoint_manager_nodes"),
                "maps",
                track_subdir,
                "trackboundary_left.csv",
            )
    file_path_outer_line = os.path.join(
                get_package_share_directory("nif_waypoint_manager_nodes"),
                "maps",
                track_subdir,
                "trackboundary_right.csv",
            )

    geofence_filter_node = Node(
                package="nif_geofence_filter_nodes",
                executable="nif_geofence_filter_nodes_exe",
                output="screen",
                respawn=True,
                emulate_tty=True,
                parameters=[{
                    # geofence file path
                    'file_path_inner_line' : file_path_inner_line,
                    'file_path_outer_line' : file_path_outer_line,

                    # Filters out the tracks under a certain distance from the track geofences
                    'distance_filter_active': True,
                    'distance_filter_threshold_m': 1.,

                    # Filters out the tracks outside the track boundaries
                    'boundaries_filter_active': True,

                    # Filters out the tracks considered static
                    'range_rate_filter_active': True,
                    'range_rate_filter_threshold_mps': 2.0,

                    # Filters out the tracks considered static
                    'track_angle_filter_active': True,
                    'track_angle_filter_threshold_deg': 15.0,

                    # Filters out the tracks considered static
                    'track_range_filter_active': True,
                    'track_range_filter_threshold_min_m': 15.0,

                    }],

                remappings=[
                    ("in_perception_array", "/clustered/perception_list"),
                    ("out_filtered_perception_array", "/clustered/perception_list/filtered"),
                    # ("out_filtered_perception_array", "/perception/concat"),

                    ("in_radar_track", "/radar_front/esr_track"),
                    ("out_filtered_radar_track", "/radar_front/esr_track/filtered"),
                    ("out_filtered_radar_track_vis", "/radar_front/esr_track/filtered/vis"),
                    ("out_filtered_radar_perception_list", "/radar_front/perception_list/filtered"),
                    # ("out_filtered_radar_perception_list", "/perception/concat"),
                ]
            )

    return LaunchDescription(
        [
            geofence_filter_node
        ]
    )



