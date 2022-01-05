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
IMS_SIM = 2
LVMS = 3
LVMS_SIM = 4
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS" or track_id == "ims":
    track = IMS
elif track_id == "LOR" or track_id == "lor":
    track = LOR
elif track_id == "IMS_SIM" or track_id == "ims_sim":
    track = IMS_SIM
elif track_id == "LVMS" or track_id == "lvms":
    track = LVMS
elif track_id == "LVMS_SIM" or track_id == "lvms_sim":
    track = LVMS_SIM
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    # nif_geofence_filter_radar_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_share_file("nif_geofence_filter_nodes", 'launch/deploy.launch.py')
    #     )
    # )

    # nif_aptiv_interface_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         get_share_file("nif_radar_clustering_nodes", 'launch/aptiv.launch.py')
    #     )
    # )

    nif_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("nif_objects_tracker_nodes", 'launch/ghost.launch.py')
        )
    )

    nif_prediction_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("nif_opponent_prediction_nodes", 'launch/deploy.launch.py')
        )
    )

    nif_ghost_spawner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_share_file("nif_ghost_vehicle_spawner_nodes", 'launch/deploy.launch.py')
        )
    )

    return LaunchDescription([
        # nif_telemetry_node,
        nif_ghost_spawner_launch,
        # nif_geofence_filter_radar_launch,
        # nif_aptiv_interface_launch,
        nif_tracker_launch,
        nif_prediction_launch
])