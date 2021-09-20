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
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS":
    track = IMS
elif track_id == "LOR":
    track = LOR
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    if track == LOR:
        globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/traj_ltpl_cl_lor_test.csv")
        graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/stored_graph.pckl")
        ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ltpl_config_offline.ini")
        ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ltpl_config_online.ini")
        log_path = get_share_file("nif_multilayer_planning_nodes", "logs/graph_ltpl")
    elif track == IMS:
        globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/traj_ltpl_cl_ims.csv")
        graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/stored_graph.pckl")
        ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ltpl_config_offline.ini")
        ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ltpl_config_online.ini")
        log_path = get_share_file("nif_multilayer_planning_nodes", "logs/graph_ltpl")
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    nif_multilayer_planning_node = Node(
        package='nif_multilayer_planning_nodes',
        executable='nif_multilayer_planning_nodes_exe',
        output={
            'stdout': 'log',
            'stderr': 'log',
        },
        parameters=[
            {
                "globtraj_input_path": globtraj_input_path,
                "graph_store_path": graph_store_path,
                "ltpl_offline_param_path": ltpl_offline_param_path,
                "ltpl_online_param_path": ltpl_online_param_path,
                "log_path": log_path,
                # "graph_log_id": ,
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
