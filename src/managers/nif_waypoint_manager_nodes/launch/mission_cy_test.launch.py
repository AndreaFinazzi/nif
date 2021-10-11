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

    config_file_lor = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "mission",
            "lor_new.yaml",
        ),
    )

    config_file_ims = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "mission",
            "ims_new.yaml",
        ),
    )

    nif_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_launch') + '/launch/deploy/nif_base.launch.py'
        ),
    )

    config_file = None

    if track == LOR:
        config_file = config_file_lor
    elif track == IMS:
        config_file = config_file_ims
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    param_file_argument = DeclareLaunchArgument(
        'nif_waypoint_manager_param_file',
        default_value=config_file,
        description='Path to config file for waypoint manager'
    )

    waypoint_manager_node = Node(
        package='nif_waypoint_manager_nodes',
        executable='nif_waypoint_manager_nodes_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('nif_waypoint_manager_param_file')
        ],
        remappings=[
            # ('topic_ego_odometry', '/bvs_localization/ltp_odom'),
            ('topic_ego_odometry', '/aw_localization/ekf/odom'),
            ('wpt_manager/maptrack_path/global', '/planning/path_global'),
            ('wpt_manager/maptrack_path/body', '/planning/path_body')
        ]
    )

    return LaunchDescription([
        nif_base_launch,
        param_file_argument,
        waypoint_manager_node
    ])
