import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


IMS = 0
LOR = 1
LG_SIM = 2
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS" or track_id == "ims":
    track = IMS
elif track_id == "LOR" or track_id == "lor":
    track = LOR
elif track_id == "LG_SIM" or track_id == "lg_sim":
    track = LG_SIM
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    if track == LOR:
        globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/lor/traj_ltpl_cl.csv")
        graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/track_offline_graphs/lor/stored_graph.pckl")
        ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lor/ltpl_config_offline.ini")
        ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lor/ltpl_config_online.ini")
        log_path = get_share_file("nif_multilayer_planning_nodes", "logs/lor/graph_ltpl")
    elif track == IMS:
        globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/ims/traj_ltpl_cl.csv")
        graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/track_offline_graphs/ims/stored_graph.pckl")
        ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ims/ltpl_config_offline.ini")
        ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ims/ltpl_config_online.ini")
        log_path = get_share_file("nif_multilayer_planning_nodes", "logs/ims/graph_ltpl")
    elif track == LG_SIM:
        globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/lg_sim/traj_ltpl_cl.csv")
        graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/track_offline_graphs/lg_sim/stored_graph.pckl")
        ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lg_sim/ltpl_config_offline.ini")
        ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lg_sim/ltpl_config_online.ini")
        log_path = get_share_file("nif_multilayer_planning_nodes", "logs/lg_sim/graph_ltpl")
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    nif_multilayer_planning_node = Node(
        package='nif_multilayer_planning_nodes',
        executable='nif_multilayer_planning_nodes_exe',
        output='screen',
        # output={
        #     'stdout': 'log',
        #     'stderr': 'log',
        # },
        parameters=[
            {
                "globtraj_input_path": globtraj_input_path,
                "graph_store_path": graph_store_path,
                "ltpl_offline_param_path": ltpl_offline_param_path,
                "ltpl_online_param_path": ltpl_online_param_path,
                "log_path": log_path,
            }
        ],
        remappings={
            ('out_local_maptrack_inglobal', '/planning/graph/path_global'),
            ('in_ego_odometry', '/localization/ekf/odom'),
        }
    )

    return LaunchDescription([
        nif_multilayer_planning_node
    ])
