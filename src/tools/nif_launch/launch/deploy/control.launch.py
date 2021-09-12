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


    return LaunchDescription([
        long_control_param,
        long_control_node,
        control_lqr_launch,
        csl_launch
    ])
