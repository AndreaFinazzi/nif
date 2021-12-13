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
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

namespace = "/camera"
# namespace = ""

exposure_value = 0.5
auto_exposure_value = "Off"
gain_value = 1.0
auto_gain_value = "On"

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():
    # Print to terminal to notify cameras are enabled
    print('CAMERAS ARE ENABLED')

    veh_id = os.environ.get('VEHICLE_ID')
    if veh_id is None or not veh_id.isnumeric():
        veh_id = '0'

    veh_id = veh_id.strip()

    ip_camera_left_arg = DeclareLaunchArgument(
        'ip_camera_left', default_value=TextSubstitution(text='10.42.{}.50'.format(veh_id))
    )
    ip_camera_left_front_arg = DeclareLaunchArgument(
        'ip_camera_left_front', default_value=TextSubstitution(text='10.42.{}.51'.format(veh_id))
    )
    ip_camera_right_front_arg = DeclareLaunchArgument(
        'ip_camera_right_front', default_value=TextSubstitution(text='10.42.{}.52'.format(veh_id))
    )
    ip_camera_right_arg = DeclareLaunchArgument(
        'ip_camera_right', default_value=TextSubstitution(text='10.42.{}.53'.format(veh_id))
    )
    ip_camera_right_rear_arg = DeclareLaunchArgument(
        'ip_camera_right_rear', default_value=TextSubstitution(text='10.42.{}.54'.format(veh_id))
    )
    ip_camera_left_rear_arg = DeclareLaunchArgument(
        'ip_camera_left_rear', default_value=TextSubstitution(text='10.42.{}.55'.format(veh_id))
    )

    """Launch all packages for the vehicle in IAC."""
    default_camera_param_file = get_share_file(
        package_name='avt_vimba_camera', file_name='config/main/default.yaml'
    )

    default_camera_param = DeclareLaunchArgument(
        'default_camera_param_file',
        default_value=default_camera_param_file,
        description='Path to config file for avt_vimba_camera'
    )

    camera_front_left = Node(
        package='avt_vimba_camera',
        namespace=namespace,
        name='front_left',
        executable='mono_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('default_camera_param_file'),
            {
                'camera_ip_addr'    :   LaunchConfiguration('ip_camera_left'),
                'frame_id'          :   'camera_front_left',
                'exposure_auto'     :   auto_exposure_value,
                'exposure'          :   exposure_value,
                'whitebalance_auto' :   'Continuous',
                'gain_auto'         :   auto_gain_value,
                'gain'              :   gain_value,
                'roi_height'        :   772,
                'roi_width'         :   1032,
                'roi_offset_x'      :   516,
                'roi_offset_y'      :   388
            }
        ],
        remappings=[
        ]
    )

    debayer_front_left = Node(
        package = "camera-utils",
        namespace=namespace,
        name="debayer_front_left",
        executable="debayer_node",
        output="screen",
        parameters = [
            {
                "topics": ["/camera/front_left/image",],
            }
        ]
    )

    camera_front_left_center = Node(
        package='avt_vimba_camera',
        namespace=namespace,
        name='front_left_center',
        executable='mono_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('default_camera_param_file'),
            {
                'camera_ip_addr'    :   LaunchConfiguration('ip_camera_left_front'),
                'frame_id'          :   'camera_front_1',
                'exposure_auto'     :   auto_exposure_value,
                'exposure'          :   exposure_value,
                'whitebalance_auto' :   'Continuous',
                'gain_auto'         :   auto_gain_value,
                'gain'              :   gain_value,
                'roi_height'        :   772,
                'roi_width'         :   1032,
                'roi_offset_x'      :   516,
                'roi_offset_y'      :   388
            }
        ],
        remappings=[
        ]
    )

    debayer_front_left_center = Node(
        package = "camera-utils",
        namespace=namespace,
        name="debayer_front_left_center",
        executable="debayer_node",
        output="screen",
        parameters = [
            {
                "topics": ["/camera/front_left_center/image",],
            }
        ]
    )

    camera_front_right_center = Node(
        package='avt_vimba_camera',
        namespace=namespace,
        name='front_right_center',
        executable='mono_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('default_camera_param_file'),
            {
                'camera_ip_addr'    :   LaunchConfiguration('ip_camera_right_front'),
                'frame_id'          :   'camera_front_2',
                'exposure_auto'     :   auto_exposure_value,
                'exposure'          :   exposure_value,
                'whitebalance_auto' :   'Continuous',
                'gain_auto'         :   auto_gain_value,
                'gain'              :   gain_value,
                'roi_height'        :   772,
                'roi_width'         :   1032,
                'roi_offset_x'      :   516,
                'roi_offset_y'      :   388
            }
        ],
        remappings=[
        ]
    )

    debayer_front_right = Node(
        package = "camera-utils",
        namespace=namespace,
        name="debayer_front_right",
        executable="debayer_node",
        output="screen",
        parameters = [
            {
                "topics": ["/camera/front_right/image",],
            }
        ]
    )

    camera_front_right = Node(
        package='avt_vimba_camera',
        namespace=namespace,
        name='front_right',
        executable='mono_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('default_camera_param_file'),
            {
                'camera_ip_addr'    :   LaunchConfiguration('ip_camera_right'),
                'frame_id'          :   'camera_front_right',
                'exposure_auto'     :   auto_exposure_value,
                'exposure'          :   exposure_value,
                'whitebalance_auto' :   'Continuous',
                'gain_auto'         :   auto_gain_value,
                'gain'              :   gain_value,
                'roi_height'        :   772,
                'roi_width'         :   1032,
                'roi_offset_x'      :   516,
                'roi_offset_y'      :   388
            }
        ],
        remappings=[
        ]
    )

    debayer_front_right_center = Node(
        package = "camera-utils",
        namespace=namespace,
        name="debayer_front_right_center",
        executable="debayer_node",
        output="screen",
        parameters = [
            {
                "topics": ["/camera/front_right_center/image",],
            }
        ]
    )

    camera_rear_right = Node(
        package='avt_vimba_camera',
        namespace=namespace,
        name='rear_right',
        executable='mono_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('default_camera_param_file'),
            {
                'camera_ip_addr'    :   LaunchConfiguration('ip_camera_right_rear'),
                'frame_id'          :   'camera_rear_right',
                'exposure_auto'     :   auto_exposure_value,
                'exposure'          :   exposure_value,
                'whitebalance_auto' :   'Continuous',
                'gain_auto'         :   auto_gain_value,
                'gain'              :   gain_value,
                'roi_height'        :   772,
                'roi_width'         :   1032,
                'roi_offset_x'      :   516,
                'roi_offset_y'      :   388
            }
        ],
        remappings=[
        ]
    )

    debayer_rear_right = Node(
        package = "camera-utils",
        namespace=namespace,
        name="debayer_rear_right",
        executable="debayer_node",
        output="screen",
        parameters = [
            {
                "topics": ["/camera/rear_right/image",],
            }
        ]
    )

    camera_rear_left = Node(
        package='avt_vimba_camera',
        namespace=namespace,
        name='rear_left',
        executable='mono_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('default_camera_param_file'),
            {
                'camera_ip_addr'    :   LaunchConfiguration('ip_camera_left_rear'),
                'frame_id'          :   'camera_rear_left',
                'exposure_auto'     :   auto_exposure_value,
                'exposure'          :   exposure_value,
                'whitebalance_auto' :   'Continuous',
                'gain_auto'         :   auto_gain_value,
                'gain'              :   gain_value,
                'roi_height'        :   772,
                'roi_width'         :   1032,
                'roi_offset_x'      :   516,
                'roi_offset_y'      :   388
            }
        ],
        remappings=[
        ]
    )

    debayer_rear_left = Node(
        package = "camera-utils",
        namespace=namespace,
        name="debayer_rear_left",
        executable="debayer_node",
        output="screen",
        parameters = [
            {
                "topics": ["/camera/rear_left/image",],
            }
        ]
    )

    return LaunchDescription([
        ip_camera_left_arg,
        ip_camera_left_front_arg,
        ip_camera_right_front_arg,
        ip_camera_right_arg,
        ip_camera_right_rear_arg,
        ip_camera_left_rear_arg,
        default_camera_param,
        camera_front_left,
        camera_front_left_center,
        camera_front_right_center,
        camera_front_right,
        camera_rear_right,
        camera_rear_left,
        debayer_front_left,
        debayer_front_left_center,
        debayer_front_right_center,
        debayer_front_right,
        debayer_rear_right,
        debayer_rear_left,
    ])
