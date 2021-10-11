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

    if track == LOR:
        map_file = 'LOR.osm'
        directory = 'LOR'
    elif track == IMS:
        map_file = 'IMS.osm'
        directory = 'IMS'
    elif track == LG_SVL:
        map_file = 'LG_SVL.osm'
        directory = 'LG_SVL'    
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    fake_obstacle_path = os.path.join(
                    get_package_share_directory("nif_dk_fake_obstacle_node"),
                    "map", directory,
                    'fake_obstacle.osm'
    )  


    dk_planner_node =  Node(
                package="nif_dk_fake_obstacle_node",
                executable="nif_dk_fake_obstacle_node_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[{
                    'fake_obs_osm_name': fake_obstacle_path,
                    # IMS
                    # 'origin_lat' : 39.809786,
                    # 'origin_lon' : -86.235148,

                    #LOR
                    'origin_lat' : 39.8125900071711,
                    'origin_lon' : -86.3418060783425,

                }],
                remappings=[]
    )

    return LaunchDescription(
        [
            dk_planner_node    
        ])
    

