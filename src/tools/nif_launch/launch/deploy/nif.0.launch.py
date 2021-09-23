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
track = LOR

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    """Launch all packages for the vehicle in IAC."""
    ssc_interface_param_file = get_share_file(
        package_name='bvs_common', file_name='param/ssc.param.yaml'
    )

    dbc_file_path = get_share_file(
        package_name='raptor_dbw_can', file_name='launch/CAN1_INDY_V3.dbc'
    )

    long_control_param_file = get_share_file(
        package_name='bvs_long_control', file_name='config/params.yaml'
    )

    long_control_param = DeclareLaunchArgument(
        'long_control_param_file',
        default_value=long_control_param_file,
        description='Path to config file for long_control'
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

    safety_node = Node(
        package='bvs_safety',
        executable='safety_executive_node',
        name='safety_executive_node',
        output='screen'
    )

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
        launch_arguments={'interface': 'can6'}.items()
    )

    socketcan_sender_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_share_file(
                package_name='ros2_socketcan',
                file_name='launch/socket_can_sender.launch.py'
            )
        ]),
        launch_arguments={'interface': 'can6'}.items()
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

    lqr_params_config = get_share_file(
        package_name='bvs_control', file_name='config/lqr_params.yaml'
    )

    # MAPs
    lor_inside_line_csv = get_share_file(
        package_name='bvs_control', file_name='config/LOR_inside_line.csv'
    )

    ims_center_line_csv = get_share_file(
        package_name='bvs_control', file_name='config/IMS_center_line.csv'
    )

    map_csv = None

    if track == LOR:
        map_csv = lor_inside_line_csv
    elif track == IMS:
        map_csv = ims_center_line_csv
    else:
        print("ERROR: invalid track provided: {}".format(track))

    lqr_control_node = Node(
        package='bvs_control',
        executable='path_follower_node',
        parameters=[{'lqr_config_file': lqr_params_config}],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    localization_node_bg = Node(
        package='bvs_localization',
        executable='localization_node',
        output='screen',
        parameters=[
            {
                "ltp_frame": "odom",
                "base_link_frame": "base_link",
                "ltp_latitude": 39.8125900071711,
                "ltp_longitude": -86.3418060783425,
            },
            {"subscribe_novatel_oem7_msgs": True },
            {"subscribe_novatel_gps_msgs": False }
        ],
        remappings=[
            ("novatel_oem7_msgs/inspva", "novatel_bottom/inspva")
        ]
    )

    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_localization_nodes') + '/launch/deploy.launch.py'
        ),
    )

    wall_node_bg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_points_preprocessor_nodes') + '/launch/deploy.launch.py'
        ),
    )

    path_publisher_node = Node(
        package='bvs_utils',
        executable='path_publisher_node',
        parameters=[{'track_line_csv': map_csv}],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        remappings=[
            ('bvs_localization/ltp_odom', '/localization/ekf/odom')
        ]
    )

    long_control_param_file = get_share_file(
        package_name='bvs_long_control', file_name='config/params.yaml'
    )

    long_control_param = DeclareLaunchArgument(
        'long_control_param_file',
        default_value=long_control_param_file,
        description='Path to config file for long_control'
    )

    long_control_node = Node(
        package='bvs_long_control',
        executable='long_control',
        output='screen',
        parameters=[LaunchConfiguration('long_control_param_file')],
    )

    control_lqr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_control_lqr_nodes') + '/launch/deploy.launch.py'
        )
    )

    csl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_control_safety_layer_nodes') + '/launch/deploy.launch.py'
        ),

    )

    global_params_file = None

    if track == LOR:
        global_params_file = 'params_LOR.global.yaml'
    elif track == IMS:
        global_params_file = 'params_IMS.global.yaml'
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    nif_global_parameters_file = os.path.join(
        get_package_share_directory('nif_launch'),
        'config',
        'deploy',
        global_params_file
    )

    global_parameters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_common_nodes') + '/launch/parameters.launch.py'
        ),
        launch_arguments={
            'nif_global_parameters_file': nif_global_parameters_file
        }.items()
    )

    system_status_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_system_status_manager_nodes') + '/launch/deploy.launch.py'
        ),
    )

    safety_node = Node(
        package='bvs_safety',
        executable='safety_executive_node',
        name='safety_executive_node',
        output='screen'
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('av21_description') + '/launch/deploy.launch.py'
        )
    )

    # multilayer_planning_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_package_share_directory('nif_multilayer_planning_nodes') + '/launch/deploy.launch.py'
    #     ),
    # )

    return LaunchDescription([
        ssc_interface_param,
        long_control_param,
        # diagnostics_node,
        ssc_interface,
        socketcan_receiver_launch,
        socketcan_sender_launch,
        raptor_node,
        global_parameters_launch,
        system_status_manager_launch,
        long_control_node,
        csl_launch,
        telemetry_node,
        control_lqr_launch,
        localization_node,
        localization_node_bg,
        wall_node_bg,
        path_publisher_node,
        safety_node,
        robot_description_launch,
        # multilayer_planning_launch
    ])