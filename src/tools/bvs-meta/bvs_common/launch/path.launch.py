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

    path_publisher_node = Node(
        package='bvs_utils',
        executable='path_publisher_node',
        parameters=[{'track_line_csv': map_csv}],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        }
    )

    return LaunchDescription([
        path_publisher_node,
    ])
