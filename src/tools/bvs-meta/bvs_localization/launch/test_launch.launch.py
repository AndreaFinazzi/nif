# Copyright 2020-2021, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
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


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    lqr_params_config = get_share_file(
        package_name='bvs_control', file_name='config/lqr_params.yaml'
    )
    lor_inside_line_csv = get_share_file(
        package_name='bvs_control', file_name='config/LOR_inside_line.csv'
    )

    localization_node = Node(
        package='bvs_localization',
        executable='localization_executive',
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        parameters=[
            {
                "ltp_latitude": 39.809786,
                "ltp_longitude": -86.235148,
                "sources": ["front_wheels", "novatel_bottom_inspva", "novatel_bottom_bestpos", "novatel_bottom_bestvel"],
                "source_priority": ["front_wheels", "novatel_bottom_bestpos", "novatel_bottom_bestvel", "novatel_bottom_inspva"],
                
                "sources.front_wheels.type": "front_wheel_velocity",
                "sources.front_wheels.topic": "/raptor_dbw_interface/wheel_speed_report",
                
                "sources.novatel_bottom_inspva.type": "novatel_oem7_inspva",
                "sources.novatel_bottom_inspva.topic": "/novatel_bottom/inspva",
                "sources.novatel_bottom_inspva.health_lim_age": 0.2,

                "sources.novatel_bottom_bestpos.type": "novatel_oem7_bestpos",
                "sources.novatel_bottom_bestpos.topic_bestpos": "/novatel_bottom/bestpos",
                "sources.novatel_bottom_bestpos.health_lim_stddev": 0.02,
                "sources.novatel_bottom_bestpos.health_lim_age": 0.2,

                "sources.novatel_bottom_bestvel.type": "novatel_oem7_bestvel",
                "sources.novatel_bottom_bestvel.topic_bestvel": "/novatel_bottom/bestvel",
                "sources.novatel_bottom_bestvel.topic_inspva": "/novatel_bottom/inspva",
                "sources.novatel_bottom_bestvel.orient_to_inspva": True,
                "sources.novatel_bottom_bestvel.health_lim_age": 0.2,
            }
        ]
    )


    urdf = get_share_file('iac_launch', "urdf/av21.urdf")

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Define robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'publish_frequency': 1.0,
            'ignore_timestamp': False,
            'use_tf_static': True,
            'robot_description': robot_desc,
        }])

    return LaunchDescription([
        # lqr_control_node,
        # path_publisher_node,
        localization_node,
        robot_state_publisher,
        # path_server_node
    ])
