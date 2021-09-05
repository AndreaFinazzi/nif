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

    lqr_control_node = Node(
        package='nif_localization_nodes',
        executable='nif_localization_nodes_exe',
        parameters=[],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        remappings=[
            ('in_bestpos', '/novatel_bottom/bestpos'),
            ('in_inspva', '/novatel_bottom/inspva'),
            ('in_imu', '/novatel_bottom/imu/data'),
            ('in_wheel_speed_report', '/raptor_dbw_interface/wheel_speed_report'),
            ('out_odometry_ekf_estimated', '/localization/odometry/ekf'),
            ('out_odometry_bestpos', '/localization/odometry/bestpos'),
        ]
    )

    return LaunchDescription([
        lqr_control_node,
    ])
