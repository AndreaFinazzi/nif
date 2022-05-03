#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    tf_prefix = LaunchConfiguration('tf_prefix', default='')
    xacro_path = PathJoinSubstitution(
        [ThisLaunchFileDir(), '../xacro/il15_red.xacro'])
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ns = LaunchConfiguration('ns', default='/')

    return LaunchDescription([
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='path to urdf.xacro file to publish'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            namespace=ns,
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path, ' ', 'prefix:=', tf_prefix]),
                'use_sim_time':use_sim_time,
            }]),
    ])
