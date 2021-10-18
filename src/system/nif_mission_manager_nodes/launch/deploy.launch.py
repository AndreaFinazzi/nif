import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


IMS = 0
LOR = 1
LG_SVL = 2
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS":
    track = IMS
elif track_id == "LOR":
    track = LOR
elif track_id == "LG_SVL":
    track = LG_SVL
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    param_file = None

    if track == LOR:
        param_file = 'transitions.lor.yaml'
    elif track == IMS:
        param_file = 'transitions.ims.yaml'
    elif track == LG_SVL:
        param_file = 'transitions.sim.yaml'
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))


    mm_param_default_file = os.path.join(
        get_package_share_directory('nif_mission_manager_nodes'),
        'config',
        param_file
    )
    
    mission_manager_node = Node(
        package='nif_mission_manager_nodes',
        executable='nif_mission_manager_nodes_exe',
        parameters=[{
            "missions_file_path": mm_param_default_file,
            "velocity.zero": 0.0,
            "velocity.max": 15.0,
            "velocity.avoidance": 15.0,
            "velocity.warmup": 15.0,
            "velocity.pit_in": 8.0,
            "velocity.pit_out": 8.0,
            "velocity.slow_drive": 12.0,
            # RC interface params
            "listen_to_override": False,
            "listen_to_nominal": True,

            # Mission avoidance auto switch
            "mission.avoidance.auto_switch": False,
            "mission.avoidance.lap_count": 0,
            "mission.avoidance.previous_track_flag": 1,
            "mission.avoidance.lap_distance_min": 0,
            "mission.avoidance.lap_distance_max": 0,

            "mission.warmup.auto_switch": False,
        }],
        remappings=[
            ('out_mission_status', '/system/mission'),
        ]
    )

    return LaunchDescription([
        mission_manager_node
    ])
