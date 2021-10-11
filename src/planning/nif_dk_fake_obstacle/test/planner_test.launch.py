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

    if track == LOR:
        map_file = 'LOR.osm'
    elif track == IMS:
        map_file = 'IMS.osm'
    elif track == LG_SVL:
        map_file = 'LG_SVL.osm'
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    map_file_path = os.path.join(
                    get_package_share_directory("nif_dk_graph_planner"),
                    "map",
                    map_file
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
                    'globalmap_file_name' : map_file_path,
                    'origin_lat' : 39.809786,
                    'origin_lon' : -86.235148,
                }],
                remappings=[]
    )

    return LaunchDescription(
        [
            dk_planner_node    
        ])
    

