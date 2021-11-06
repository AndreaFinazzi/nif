import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

IMS = 0
LOR = 1
LG_SVL = 2

track_id = os.environ.get('TRACK').strip()

if track_id == "IMS":
    track = IMS
elif track_id == "LOR":
    track = LOR
elif track_id == "LG_SVL":
    track = LG_SVL
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))

def generate_launch_description():

    map_file = None
    directory = None
    origin_lat = 0.0
    origin_lon = 0.0

    if track == LOR:
        map_file = 'LOR.osm'
        directory = 'LOR'
        origin_lat = 39.8125900071711
        origin_lon = -86.3418060783425

    elif track == IMS:
        map_file = 'IMS.osm'
        directory = 'IMS'
        origin_lat = 39.809786
        origin_lon = -86.235148

    elif track == LG_SVL:
        map_file = 'LG_SIM.osm'
        directory = 'LG_SIM'
        origin_lat = 39.79312996
        origin_lon = -86.23524024
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    map_file_path = os.path.join(
                    get_package_share_directory("nif_dk_graph_planner"),
                    "map", directory,
                    map_file
    )  

    racing_traj_path = os.path.join(
                    get_package_share_directory("nif_dk_graph_planner"),
                    "map", directory,
                    'traj_race_cl.csv'
    )  


    dk_planner_node =  Node(
                package="nif_dk_graph_planner",
                executable="nif_dk_graph_planner_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[{
                    'osm_name' : map_file_path,
                    'racing_trajectory' : racing_traj_path,
                    'origin_lat' : origin_lat,
                    'origin_lon' : origin_lon,
                    
                    'ref_gain' : 5.0, #0.5,
                    'collision_gain' : 30.0, #30.0,
                    'curvature_gain' : 10.0, #10.0,
                    'transient_gain' : 2.0,

                    'inflation_size' : 3.0,
                    'collision_check_radius' : 2.0,
                    'final_path_update_dist' : 300., # 170.0,
                }],
                remappings=[ 
                    ("in_ekf_odometry", "/aw_localization/ekf/odom"),
                    ("in_inflated_points", "/inflated_points"),
                    ("in_cluster_center_points", "/cluster_center_points"),
                    ("in_wall_points", "/ransac_filtered_points/right_no_use"),
                
                ]
    )

    return LaunchDescription(
        [
            dk_planner_node    
        ])
    

