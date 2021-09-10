import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    nif_multilayer_planning_node = Node(
        package='nif_multilayer_planning_nodes',
        executable='nif_multilayer_planning_nodes_exe',
        output='screen',
        parameters=[
            {
                "globtraj_input_path": get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/traj_ltpl_cl_lgsim_ims.csv"),
                "graph_store_path": get_share_file("nif_multilayer_planning_nodes", "inputs/stored_graph.pckl"),
                "ltpl_offline_param_path": get_share_file("nif_multilayer_planning_nodes", "params/ltpl_config_offline.ini"),
                "ltpl_online_param_path": get_share_file("nif_multilayer_planning_nodes", "params/ltpl_config_online.ini"),
                "log_path": get_share_file("nif_multilayer_planning_nodes", "logs/graph_ltpl"),
                # "graph_log_id": ,
            }
        ],
        remappings={
            ('out_local_maptrack_inglobal', '/planning/path_global'),
            # ('in_vehicle_odometry', '/localization/ego_odom'),
            ('in_ego_odometry', '/sensor/odom_converted'),
            # ('in_perception_result', '/perception/result'),
        }
    )

    return LaunchDescription([
        nif_multilayer_planning_node
    ])
