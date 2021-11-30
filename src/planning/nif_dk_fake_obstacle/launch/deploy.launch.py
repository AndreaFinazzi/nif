import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
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

    directory = None
    origin_lat = 0.0
    origin_lon = 0.0

    if track == LOR:
        directory = 'LOR'
        origin_lat = 39.8125900071711
        origin_lon = -86.3418060783425

    elif track == IMS:
        directory = 'IMS'
        origin_lat = 39.809786
        origin_lon = -86.235148

    elif track == IMS_SIM:
        directory = 'LG_SIM'
        origin_lat = 39.79312996
        origin_lon = -86.23524024

    elif track == LVMS:
        directory = 'LVMS'
        origin_lat = 36.268252
        origin_lon = -115.015914

    elif track == LVMS_SIM:
        directory = 'LVMS_SIM'
        origin_lat = 36.268252
        origin_lon = -115.015914
        
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    fake_obstacle_path = os.path.join(
                    get_package_share_directory("nif_dk_fake_obstacle"),
                    "map", directory,
                    'fake_obstacle.osm'
    )  


    dk_fake_obs_node =  Node(
                package="nif_dk_fake_obstacle",
                executable="nif_dk_fake_obstacle_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[{
                    'fake_obs_osm_name': fake_obstacle_path,
                    'origin_lat' : origin_lat,
                    'origin_lon' : origin_lon,
                }],
                remappings=[ 
                    # ("in_ekf_odometry", "/aw_localization/ekf/odom"),
                    # ("in_oc_grid", "/semantics/costmap_generator/occupancy_grid"),
                ]
    )

    return LaunchDescription(
        [
            dk_fake_obs_node    
        ])
    
