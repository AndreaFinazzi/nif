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
        package_name='bvs_control', file_name='config/IMS_center_line.csv'
    )

    lqr_control_node = Node(
        package='bvs_control',
        executable='path_follower_node',
        parameters=[{'lqr_config_file': lqr_params_config}],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    localization_node = Node(
        package='bvs_localization',
        executable='localization_executive',
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    path_publisher_node = Node(
        package='bvs_control',
        executable='path_publisher_node',
        parameters=[{'track_line_csv': lor_inside_line_csv}],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )
    path_server_node = Node(
        package='path_server',
        executable='path_server',
        output='screen',
    )

    return LaunchDescription([
        lqr_control_node,
        path_publisher_node,
        localization_node,
        # path_server_node
    ])
