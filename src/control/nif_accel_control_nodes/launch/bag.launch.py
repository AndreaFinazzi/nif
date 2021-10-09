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
        package='nif_accel_control_nodes',
        executable='nif_accel_control_nodes_exe',
        parameters=[
            {
                'engine_based_throttle_enabled': True,
                'gear.track': "IMS",
            }
        ],
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
        remappings=[
            ('/control_safety_layer/out/desired_accel', '/control_joint_lqr/accel_command'),
            ('/joystick/accelerator_cmd', '/joystick/accelerator_cmd_test'),
            ('/joystick/accelerator_cmd/raw', '/joystick/accelerator_cmd/raw_test'),
            ('/joystick/brake_cmd', '/joystick/brake_cmd_test'),
            ('/joystick/brake_cmd/raw', '/joystick/brake_cmd/raw_test'),
        ]
    )

    return LaunchDescription([
        lqr_control_node,
    ])
