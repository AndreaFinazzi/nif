
import os
import yaml

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_cfg_path(cfg):
    return os.path.join(get_package_share_directory("novatel_oem7_driver"), "config", cfg )


def get_params(cfg):
    p = get_cfg_path(cfg)
    with open(p, 'r') as f:
        return yaml.safe_load(f)


def arg(name, default_value, description):
    return DeclareLaunchArgument(name = name, description = description, default_value = default_value)


def generate_launch_description():

    node = Node(
        package='novatel_oem7_driver',
        namespace='novatel_bottom',
        executable='novatel_oem7_driver_exe',
        name='oem7',

        parameters=[
                    get_params("std_msg_handlers.yaml"    ),
                    get_params("std_oem7_raw_msgs.yaml"   ),
                    get_params("std_msg_topics.yaml"      ),
                    get_params("oem7_supported_imus.yaml" ),
                    # get_params("std_init_commands_noins.yaml"   ),
                    get_params("std_init_commands.yaml"   ),

                    {
                    'oem7_max_io_errors' : 100,
                    'oem7_msg_decoder': 'Oem7MessageDecoder',
                    'oem7_if'         : LaunchConfiguration('oem7_if'),
                    'oem7_ip_addr'    : LaunchConfiguration('oem7_ip_addr'),
                    'oem7_port'       : LaunchConfiguration('oem7_port')
                    }
                    ],

        output='screen',
        arguments=[('--ros-args', '--log_level', 'DEBUG')]
    )

    ip_arg   = arg('oem7_ip_addr', '10.42.4.60', 'IP Address of Oem7 Receiver, e.g. 192.168.1.2')
    port_arg = arg('oem7_port', '3001', 'TCP or UDP port, e.g. 3002')
    # if_arg   = arg('oem7_if', 'Oem7ReceiverTcp', 'Interface Type: Oem7ReceiverTcp or Oem7ReceiverUdp')
    if_arg   = arg('oem7_if', 'Oem7ReceiverUdp', 'Interface Type: Oem7ReceiverTcp or Oem7ReceiverUdp')


    return LaunchDescription([ip_arg, port_arg, if_arg, node])





