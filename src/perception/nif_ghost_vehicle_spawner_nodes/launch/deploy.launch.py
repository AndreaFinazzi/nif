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

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
  
    config_file_lor = (
        os.path.join(
            get_package_share_directory("nif_ghost_vehicle_spawner_nodes"),
            "config",
            "LVMS_SIM.yaml",
        ),
    )

    config_file_ims = (
        os.path.join(
            get_package_share_directory("nif_ghost_vehicle_spawner_nodes"),
            "config",
            "LVMS_SIM.yaml",
        ),
    )

    config_file_lvms_sim = (
        os.path.join(
            get_package_share_directory("nif_ghost_vehicle_spawner_nodes"),
            "config",
            "LVMS_SIM.yaml",
        ),
    )

    config_file_lvms = (
        os.path.join(
            get_package_share_directory("nif_ghost_vehicle_spawner_nodes"),
            "config",
            "LVMS.yaml",
        ),
    )

    config_file = None

    if track == LOR:
        config_file = config_file_lor
    elif track == IMS:
        config_file = config_file_ims
    elif track == LVMS:
        config_file = config_file_lvms
    elif track == LVMS_SIM:
        config_file = config_file_lvms_sim
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    param_file_argument = DeclareLaunchArgument(
        'nif_ghost_vehicle_spawner_param_file',
        default_value=config_file,
        description='Path to config file for waypoint manager'
    )

    ghost_spawner_node = Node(
        package='nif_ghost_vehicle_spawner_nodes',
        executable='nif_ghost_vehicle_spawner_nodes_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('nif_ghost_vehicle_spawner_param_file')
        ],
        remappings=[
            ('out_perception_result', '/ghost/perception'),
            ('out_marker', '/out_marker'),
        ]
    )

    return LaunchDescription([
        param_file_argument,
        ghost_spawner_node
    ])
