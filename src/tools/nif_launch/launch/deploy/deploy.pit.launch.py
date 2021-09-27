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
    """Launch all packages for the vehicle in IAC."""
    ssc_interface_param_file = get_share_file(
        package_name='ssc_interface', file_name='param/defaults.param.yaml'
    )

    dbc_file_path = get_share_file(
        package_name='raptor_dbw_can', file_name='launch/CAN1_INDY_V4.dbc'
    )

    ssc_interface_param = DeclareLaunchArgument(
        'ssc_interface_param_file',
        default_value=ssc_interface_param_file,
        description='Path to config file for ssc_interface'
    )

    # diagnostics_node = Node(
    #     package='diagnostics',
    #     executable='emergency_diagnostics',
    #     name='emergency_diagnostics',
    #     output='screen'
    # )

    ssc_interface = Node(
        package='ssc_interface',
        name='ssc_interface_node',
        executable='ssc_interface_node_exe',
        namespace='vehicle',
        output='screen',
        parameters=[LaunchConfiguration('ssc_interface_param_file')],
        remappings=[
            ('gear_select', '/ssc/gear_select'),
            ('arbitrated_speed_commands', '/ssc/arbitrated_speed_commands'),
            ('arbitrated_steering_commands', '/ssc/arbitrated_steering_commands'),
            ('turn_signal_command', '/ssc/turn_signal_command'),
            ('dbw_enabled_feedback', '/ssc/dbw_enabled_fedback'),
            ('gear_feedback', '/ssc/gear_feedback'),
            ('velocity_accel_cov', '/ssc/velocity_accel_cov'),
            ('steering_feedback', '/ssc/steering_feedback'),
            ('vehicle_kinematic_state_cog', '/vehicle/vehicle_kinematic_state'),
            ('state_report_out', '/vehicle/vehicle_state_report'),
            ('state_command', '/vehicle/vehicle_state_command'),
            ####
            ('accelerator_pedal_cmd', '/raptor_dbw_interface/accelerator_pedal_cmd'),
            ('brake_cmd', '/raptor_dbw_interface/brake_cmd'),
            ('steering_cmd', '/raptor_dbw_interface/steering_cmd'),
            ('gear_cmd', '/raptor_dbw_interface/gear_cmd'),
            ('ct_status', '/raptor_dbw_interface/ct_report'),

            ('/vehicle/misc_report', '/raptor_dbw_interface/misc_report_do'),
            ('/vehicle/pt_report', '/raptor_dbw_interface/pt_report'),
            ('/vehicle/accelerator_report', '/raptor_dbw_interface/accelerator_report'),
            ('/vehicle/brake_pressure_report', '/raptor_dbw_interface/brake_pressure_report'),
            ('/vehicle/steer_report', '/raptor_dbw_interface/steer_report'),
            ('/vehicle/rc_to_ct_info', '/raptor_dbw_interface/rc_to_ct'),
            ('/vehicle/flag_summary', '/raptor_dbw_interface/flag_summary'),
            ('/vehicle/accelerator_cmd_pt', '/accelerator_cmd_pt'),
            ('/vehicle/brake_cmd_pt', '/brake_cmd_pt'),
            ('/vehicle/gear_cmd_pt', '/gear_cmd_pt')
        ]
    )

    socketcan_receiver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_share_file(
                package_name='ros2_socketcan',
                file_name='launch/socket_can_receiver.launch.py'
            )
        ]),
        launch_arguments={'interface': 'can2'}.items()
    )

    socketcan_sender_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_share_file(
                package_name='ros2_socketcan',
                file_name='launch/socket_can_sender.launch.py'
            )
        ]),
        launch_arguments={'interface': 'can2'}.items()
    )

    raptor_node = Node(
        package='raptor_dbw_can',
        executable='raptor_dbw_can_node',
        output='screen',
        namespace='raptor_dbw_interface',
        parameters=[
            {'dbw_dbc_file': dbc_file_path}
        ],
        remappings=[
            ('/raptor_dbw_interface/can_rx', '/from_can_bus'),
            ('/raptor_dbw_interface/can_tx', '/to_can_bus'),
        ],
    )

    telemetry_node = Node(
        package='telemetry',
        executable='telemetry',
        output='screen',
    )

    # Localization
    nif_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_localization_nodes') + '/launch/deploy.launch.py'
        ),
    )

    nif_wall_node_launch_bg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_points_preprocessor_nodes') + '/launch/deploy.launch.py'
        ),
    )

# NIF CONTROL STACK ###############################################

    nif_csl_param_file = get_share_file(
        package_name='nif_control_safety_layer_nodes', file_name='config/deploy.yaml'
    )

    nif_csl_param = DeclareLaunchArgument(
        'control_safety_layer_param_file',
        default_value=nif_csl_param_file,
        description='Path to config file for csl'
    )

    nif_csl_node = Node(
        package='nif_control_safety_layer_nodes',
        executable='nif_control_safety_layer_nodes_exe',
        output='screen',
        parameters=[LaunchConfiguration('control_safety_layer_param_file')],
        remappings=[
            ('in_control_cmd', '/control_pool/control_cmd'),
            ('in_override_control_cmd', '/control_pool/override_cmd'),
            ('out_control_cmd', '/control_safety_layer/out/control_cmd'),
            ('out_steering_control_cmd', '/joystick/steering_cmd'),
            ('out_accelerator_control_cmd', '/joystick/accelerator_cmd'),
            ('out_braking_control_cmd', '/joystick/brake_cmd'),
            ('out_gear_control_cmd', '/joystick/gear_cmd'),
            ('out_desired_acceleration_cmd', '/control_safety_layer/out/desired_accel'),
        ]
    )

    nif_velocity_planning_node = Node(
        package='nif_velocity_planning_node',
        executable='nif_velocity_planning_node_exe',
        output='screen',
        remappings=[
            ('out_desired_velocity', 'velocity_planner/des_vel'),
            # ('in_reference_path', 'planning/graph/path_global'),
            ('in_reference_path', 'planning/path_global'),
            ('in_ego_odometry', 'localization/ekf/odom'),
            ('in_wheel_speed_report', 'raptor_dbw_interface/wheel_speed_report'),
            ('in_imu_data', 'novatel_bottom/imu/data'),
            ('in_steering_report', 'raptor_dbw_interface/steering_report'),
            ('in_control_error', 'control_joint_lqr/lqr_error')
        ]
    )

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
            }
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        remappings=[
            ('in_control_cmd_prev', '/control_safety_layer/out/control_cmd'),
            ('out_control_cmd', '/control_pool/control_cmd'),
            # ('in_reference_path', '/planning/graph/path_global'),
            ('in_reference_path', '/planning/path_global'),
        ]
    )

    nif_accel_control_param = DeclareLaunchArgument(
        'accel_control_param',
        default_value=get_share_file(package_name='nif_accel_control_nodes', file_name='config/params.yaml'
        ),
        description='Path to config file for nif_accel_control_nodes'
    )

    nif_accel_control_node = Node(
        package='nif_accel_control_nodes',
        executable='nif_accel_control_nodes_exe',
        output='screen',
        parameters=[
            LaunchConfiguration('accel_control_param')
        ]
    )

# NIF LQR + CSL END ###############################################

    global_params_file = None

    if track == LOR:
        global_params_file = 'params_LOR.global.yaml'
    elif track == IMS:
        global_params_file = 'params_IMS.global.yaml'
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

    nif_system_status_manager_node = Node(
        package='nif_system_status_manager_nodes',
        executable='nif_system_status_manager_nodes_exe',
        remappings=[
            ('in_joystick_cmd', '/joystick/command'),
            ('out_system_status', '/system/status'),
            ('in_novatel_bestpos', '/novatel_bottom/bestpos'),
            ('in_novatel_insstdev', '/novatel_bottom/insstdev')
        ]
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('av21_description') + '/launch/deploy.launch.py'
        )
    )

### NIF WAYPOINT MANAGER #############################

    wpt_config_file_lor = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "lor.pit.yaml",
        ),
    )

    wpt_config_file_ims = (
        os.path.join(
            get_package_share_directory("nif_waypoint_manager_nodes"),
            "config",
            "ims.yaml",
        ),
    )

    config_file = None

    if track == LOR:
        config_file = wpt_config_file_lor
    elif track == IMS:
        config_file = wpt_config_file_ims
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
            ('topic_ego_odometry', 'localization/ekf/odom'),
            ('wpt_manager/maptrack_path/global', '/planning/path_global'),
            ('wpt_manager/maptrack_path/body', '/planning/path_body')
        ]
    )

### NIF WAYPOINT MANAGER END #############################

    # if track == LOR:
    #     globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/lor/traj_ltpl_cl.csv")
    #     graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/track_offline_graphs/lor/stored_graph.pckl")
    #     ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lor/ltpl_config_offline.ini")
    #     ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lor/ltpl_config_online.ini")
    #     log_path = get_share_file("nif_multilayer_planning_nodes", "logs/lor/graph_ltpl")
    # elif track == IMS:
    #     globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/ims/traj_ltpl_cl.csv")
    #     graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/track_offline_graphs/ims/stored_graph.pckl")
    #     ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ims/ltpl_config_offline.ini")
    #     ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/ims/ltpl_config_online.ini")
    #     log_path = get_share_file("nif_multilayer_planning_nodes", "logs/ims/graph_ltpl")
    # elif track == LG_SIM:
    #     globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/lg_sim/traj_ltpl_cl.csv")
    #     graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/track_offline_graphs/lg_sim/stored_graph.pckl")
    #     ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lg_sim/ltpl_config_offline.ini")
    #     ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lg_sim/ltpl_config_online.ini")
    #     log_path = get_share_file("nif_multilayer_planning_nodes", "logs/lg_sim/graph_ltpl")
    # elif track == LOR_NARROW:
    #     globtraj_input_path = get_share_file("nif_multilayer_planning_nodes", "inputs/traj_ltpl_cl/lg_sim/traj_ltpl_cl.csv")
    #     graph_store_path = get_share_file("nif_multilayer_planning_nodes", "inputs/track_offline_graphs/lg_sim/stored_graph.pckl")
    #     ltpl_offline_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lg_sim/ltpl_config_offline.ini")
    #     ltpl_online_param_path = get_share_file("nif_multilayer_planning_nodes", "params/lg_sim/ltpl_config_online.ini")
    #     log_path = get_share_file("nif_multilayer_planning_nodes", "logs/lg_sim/graph_ltpl")
    # else:
    #     raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    nif_multilayer_planning_node = Node(
        package='nif_multilayer_planning_nodes',
        executable='nif_multilayer_planning_nodes_exe',
        output={
            'stdout': 'log',
            'stderr': 'screen',
        },
        # parameters=[
        #     {
        #         "globtraj_input_path": globtraj_input_path,
        #         "graph_store_path": graph_store_path,
        #         "ltpl_offline_param_path": ltpl_offline_param_path,
        #         "ltpl_online_param_path": ltpl_online_param_path,
        #         "log_path": log_path,
        #         # "graph_log_id": ,
        #     }
        # ],
        remappings={
            ('out_local_maptrack_inglobal', '/planning/graph/path_global'),
            ('in_ego_odometry', '/localization/ekf/odom'),
        }
    )

### NIF MULTILAYER PLANNER END #############################

    return LaunchDescription([
        ssc_interface_param,
        nif_global_param,
        nif_csl_param,
        nif_wpt_param,
        nif_joint_lqr_param,
        nif_accel_control_param,

        ssc_interface,
        socketcan_receiver_launch,
        socketcan_sender_launch,
        raptor_node,
        telemetry_node,

        nif_global_param_node,
        nif_system_status_manager_node,
        nif_csl_node,
        nif_localization_launch,
        nif_wall_node_launch_bg,
        nif_waypoint_manager_node,
        robot_description_launch,
        nif_multilayer_planning_node,
        nif_velocity_planning_node,
        nif_joint_lqr_control_node,
        nif_accel_control_node,
    ])