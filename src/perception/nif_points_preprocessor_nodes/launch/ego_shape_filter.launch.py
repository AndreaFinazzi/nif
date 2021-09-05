from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():    
    return LaunchDescription(
        [   
            Node(
                package='points_preprocessor',
                executable='ego_shape_filter',
                name='ego_shape_filter',
                # remappings=[('/merged/lidar', '/header/frame_id/laser_meter_flash_a')],
                parameters=[{
                    'left_lower_distance': 0.5,
                    'right_lower_distance': 0.5,
                    'rear_lower_distance': 2.2,
                    'front_lower_distance': 1.5,
                    'left_upper_distance': 20.0,
                    'right_upper_distance': 20.0,
                    'rear_upper_distance': 50.0,
                    'front_upper_distance': 50.0,
                    'height_lower_distance': -0.7,
                    'height_upper_distance': 0.5,
                    'resolution': 0.25,
                    'count_threshold': 3.,

                }],
                output='screen',
            ),   
        ])


