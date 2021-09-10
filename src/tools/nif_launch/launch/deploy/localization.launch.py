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


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    
    localization_node = Node(
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

    localization_node_bg = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_localization_nodes') + '/launch/deploy.launch.py'
        ),

    )

    return LaunchDescription([
        localization_node,
        localization_node_bg
    ])