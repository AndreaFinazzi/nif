# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch file for IAC vehicle."""

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
KAIST = 5
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
elif track_id == "KAIST" or track_id == "kaist":
    track = KAIST
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """Launch all packages for the vehicle in IAC."""

    nif_aw_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(
                'nif_aw_localization_nodes') + '/launch/deploy.launch.py'
        ),
    )

# NIF CONTROL STACK ###############################################

    # nif_csl_param_file = get_share_file(
    #     package_name='nif_control_safety_layer_nodes', file_name='config/deploy.yaml'
    # )

    # nif_csl_param = DeclareLaunchArgument(
    #     'control_safety_layer_param_file',
    #     default_value=nif_csl_param_file,
    #     description='Path to config file for csl'
    # )

    # nif_csl_node = Node(
    #     package='nif_control_safety_layer_nodes',
    #     executable='nif_control_safety_layer_nodes_exe',
    #     output='screen',
    #     parameters=[LaunchConfiguration('control_safety_layer_param_file')],
    #     remappings=[
    #         ('in_control_cmd', '/control_pool/control_cmd'),
    #         ('in_override_control_cmd', '/control_pool/override_cmd'),
    #         ('in_perception_steering', '/no'),
    #         ('in_wall_distance_inner', '/no'),
    #         ('in_wall_distance_outer', '/no'),
    #         ('out_control_cmd', '/control_safety_layer/out/control_cmd'),
    #         ('out_steering_control_cmd', '/joystick/steering_cmd'),
    #         ('out_accelerator_control_cmd', '/joystick/accelerator_cmd'),
    #         ('out_braking_control_cmd', '/joystick/brake_cmd'),
    #         ('out_gear_control_cmd', '/joystick/gear_cmd'),
    #         ('out_desired_acceleration_cmd', '/control_safety_layer/out/desired_accel'),
    #     ]
    # )

    lqr_joint_config_file = get_share_file(
        package_name='nif_control_joint_lqr_nodes', file_name='config/lqr/lqr_params.deploy.yaml'
    )

    nif_joint_lqr_rosparams_file = get_share_file(
        package_name='nif_control_joint_lqr_nodes', file_name='config/deploy.params.yaml'
    )

    nif_joint_lqr_param = DeclareLaunchArgument(
        'control_joint_lqr_params_file',
        default_value=nif_joint_lqr_rosparams_file,
        description='Path to config file for nif_control_lqr'
    )

    nif_joint_lqr_control_node = Node(
        package='nif_control_joint_lqr_nodes',
        executable='nif_control_joint_lqr_nodes_exe',
        parameters=[
            LaunchConfiguration('control_joint_lqr_params_file'),
            {
                'lqr_config_file': lqr_joint_config_file,
                'use_tire_velocity': True,
            }
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        remappings=[
            ('in_control_cmd_prev', '/control_safety_layer/out/control_cmd'),
            ('out_control_cmd', '/control_pool/control_cmd'),
            ('in_reference_path', '/planning/dynamic/vis/traj_global'),
            ('in_reference_trajectory', '/planning/dynamic/traj_global'),
        ]
    )

    gear_track = None

    if track == LOR:
        gear_track = 'LOR'
    elif track == IMS:
        gear_track = 'IMS'
    elif track == LVMS:
        gear_track = 'LVMS'

    # nif_accel_control_node = Node(
    #     package='nif_accel_control_nodes',
    #     executable='nif_accel_control_nodes_exe',
    #     output='screen',
    #     remappings=[
    #         ('/in_imu_data', '/novatel_bottom/rawimux')
    #     ],
    #     parameters=[{
    #         'engine_based_throttle_enabled': True,
    #         'gear.track': gear_track,
    #         'lateral_error_deadband_m': 1.0,
    #     }]
    # )

    nif_velocity_planning_node = Node(
        package='nif_velocity_planning_node',
        executable='nif_velocity_planning_node_exe',
        output='screen',
        remappings=[
            ('out_desired_velocity', 'velocity_planner/des_vel'),
            # ('in_reference_path', 'planning/dynamic/traj_global'),
            ('in_ego_odometry', '/aw_localization/ekf/odom'),
            ('in_wheel_speed_report', 'raptor_dbw_interface/wheel_speed_report'),
            ('in_imu_data', 'novatel_bottom/imu/data'),
            ('in_steering_report', 'raptor_dbw_interface/steering_report'),
            ('in_control_error', 'control_joint_lqr/lqr_error')
        ],
        parameters=[{
            'max_ddes_vel_dt_default': 3.0,
            'lateral_tire_model_factor': 0.8,
        }]
    )


# NIF LQR + CSL END ###############################################

    global_params_file = None

    if track == LOR:
        global_params_file = 'params_LOR.global.yaml'
    elif track == IMS:
        global_params_file = 'params_IMS.global.yaml'
    elif track == LVMS:
        global_params_file = 'params_LVMS.global.yaml'
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    nif_global_param = os.path.join(
        get_package_share_directory('nif_launch'),
        'config',
        'deploy',
        global_params_file
    )

    nif_global_param = DeclareLaunchArgument(
        'nif_global_parameters_file',
        default_value=nif_global_param,
        description='Path to config file for global parameters'
    )

    nif_global_param_node = Node(
        package='nif_common_nodes',
        executable='nif_global_parameters_exe',
        name='global_parameters_node',
        parameters=[
            LaunchConfiguration('nif_global_parameters_file')
        ]
    )

    # nif_system_status_manager_node = Node(
    #     package='nif_system_status_manager_nodes',
    #     executable='nif_system_status_manager_nodes_exe',
    #     remappings=[
    #         ('in_joystick_cmd', '/joystick/command'),
    #         ('in_localization_status', '/aw_localization/ekf/status'),
    #         ('in_mission_status', '/system/mission'),
    #         ('in_ll_diagnostic_report', '/raptor_dbw_interface/diag_report'),
    #         ('out_system_status', '/system/status'),
    #     ]
    # )

    # robot_description_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_package_share_directory('av21_description') + '/launch/deploy.launch.py'
    #     )
    # )

### NIF WAYPOINT MANAGER #############################

    wpt_config_file_lor = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "mission",
            "lor.yaml",
        ),
    )

    wpt_config_file_ims = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "mission",
            "ims.yaml",
        ),
    )

    wpt_config_file_lvms = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "mission",
            "lvms.yaml",
        ),
    )

    wpt_config_file_kaist = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "mission",
            "kaist.yaml",
        ),
    )

    config_file = None

    if track == LOR:
        config_file = wpt_config_file_lor
    elif track == IMS:
        config_file = wpt_config_file_ims
    elif track == LVMS:
        config_file = wpt_config_file_lvms
    elif track == KAIST:
        config_file = wpt_config_file_kaist
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    nif_wpt_param = DeclareLaunchArgument(
        'nif_waypoint_manager_param_file',
        default_value=config_file,
        description='Path to config file for waypoint manager'
    )

    nif_waypoint_manager_node = Node(
        package='nif_waypoint_manager_nodes',
        executable='nif_waypoint_manager_nodes_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('nif_waypoint_manager_param_file')
        ],
        remappings=[
            ('topic_ego_odometry', '/aw_localization/ekf/odom'),
            ('wpt_manager/maptrack_path/global', '/planning/path_global'),
            ('wpt_manager/maptrack_path/body', '/planning/path_body')
        ]
    )

    acc_rosparam_file = os.path.join(
        get_package_share_directory('nif_adaptive_cruise_control_node'),
        'config',
        'rosparam.yaml'
    )

    # nif_adaptive_cruise_control_param = DeclareLaunchArgument(
    #     'nif_adaptive_cruise_control_param',
    #     default_value=acc_rosparam_file,
    #     description='Path to config file for nif_adaptive_cruise_control_param'
    # )

    # idm_acc_param_file = os.path.join(
    #     get_package_share_directory('nif_adaptive_cruise_control_node'),
    #     'config',
    #     'idm_acc_config.yaml'
    # )

    # idm_based_control_node = Node(
    #     package='nif_adaptive_cruise_control_node',
    #     executable='nif_adaptive_cruise_control_node_exe',
    #     output='screen',
    #     parameters=[
    #         LaunchConfiguration('nif_adaptive_cruise_control_param'),
    #         {
    #             'idm_acc_config_file': idm_acc_param_file
    #         }
    #         ]
    # )


### NIF WAYPOINT MANAGER END #############################

    # nif_mission_manager_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_share_file("nif_mission_manager_nodes", 'launch/deploy.launch.py')
    #     )
    # )

    nif_points_clustering = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("nif_points_clustering_nodes",
                           'launch/deploy.launch.py')
        )
    )

    nif_dynamic_planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("nif_dynamic_planning_nodes",
                           'launch/deploy.launch.py')
        )
    )


    carnival_fake_system_node = Node(
        package='carnival_interface',
        executable='fake_system_status_publisher_exe',
        output='screen'
        # remappings=[
        #     ('out_desired_velocity', 'velocity_planner/des_vel'),
        #     # ('in_reference_path', 'planning/dynamic/traj_global'),
        #     ('in_ego_odometry', '/aw_localization/ekf/odom'),
        #     ('in_wheel_speed_report', 'raptor_dbw_interface/wheel_speed_report'),
        #     ('in_imu_data', 'novatel_bottom/imu/data'),
        #     ('in_steering_report', 'raptor_dbw_interface/steering_report'),
        #     ('in_control_error', 'control_joint_lqr/lqr_error')
        # ],
        # parameters=[{
        #     'max_ddes_vel_dt_default': 3.0,
        #     'lateral_tire_model_factor': 0.8,
        # }]
    )

    carnival_interface_node = Node(
        package='carnival_interface',
        executable='carnival_msg_conversion_exe',
        output='screen'
        # remappings=[
            # ('out_desired_velocity', 'velocity_planner/des_vel'),
            # ('in_reference_path', 'planning/dynamic/traj_global'),
            # ('in_ego_odometry', '/aw_localization/ekf/odom'),
            # ('in_wheel_speed_report', 'raptor_dbw_interface/wheel_speed_report'),
            # ('in_imu_data', 'novatel_bottom/imu/data'),
            # ('in_steering_report', 'raptor_dbw_interface/steering_report'),
            # ('in_control_error', 'control_joint_lqr/lqr_error')
        # ],
        # parameters=[{
            # 'max_ddes_vel_dt_default': 3.0,
            # 'lateral_tire_model_factor': 0.8,
        # }]
    )

### NIF MULTILAYER PLANNER END #############################

    return LaunchDescription([
        # ssc_interface_param,
        nif_global_param,
        # nif_csl_param,
        nif_wpt_param,
        nif_joint_lqr_param,

        # ssc_interface,
        # socketcan_receiver_launch,
        # socketcan_sender_launch,
        # raptor_node,

        nif_global_param_node,
        # nif_system_status_manager_node,
        # nif_csl_node,
        nif_aw_localization_launch,
        # nif_localization_launch,

        # nif_adaptive_cruise_control_param,
        # idm_based_control_node,
        nif_velocity_planning_node,

        nif_points_clustering,
        # robot_description_launch,
        nif_joint_lqr_control_node,
        # nif_accel_control_node,
        # nif_mission_manager_launch,
        nif_waypoint_manager_node,
        nif_dynamic_planner_launch,
        carnival_fake_system_node,
        carnival_interface_node
    ])
