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
        package_name='bvs_common', file_name='param/ssc.param.yaml'
    )

    dbc_file_path = get_share_file(
        package_name='raptor_dbw_can', file_name='launch/CAN1_INDY_V3.dbc'
    )

    ssc_interface_param = DeclareLaunchArgument(
        'ssc_interface_param_file',
        default_value=ssc_interface_param_file,
        description='Path to config file for ssc_interface'
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
        launch_arguments={'interface': 'can1'}.items()
    )

    socketcan_sender_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_share_file(
                package_name='ros2_socketcan',
                file_name='launch/socket_can_sender.launch.py'
            )
        ]),
        launch_arguments={'interface': 'can1'}.items()
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

    return LaunchDescription([
        ssc_interface_param,
        ssc_interface,
        socketcan_receiver_launch,
        socketcan_sender_launch,
        raptor_node,
        telemetry_node,
    ])
