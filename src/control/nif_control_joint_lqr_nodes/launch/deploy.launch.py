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
                'use_tire_velocity' : True,
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

    return LaunchDescription([
        nif_joint_lqr_param,
        nif_joint_lqr_control_node
    ])