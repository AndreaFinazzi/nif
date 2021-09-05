from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():    
    return LaunchDescription(
        [   
            Node(
                package='points_preprocessor',
                executable='points_concat_filter',
                name='points_concat_filter',
                parameters=[{
                    'input_topics' : '[/luminar_front_points, /luminar_left_points, /luminar_right_points]',
                    # 'input_topics' : '[/luminar_front_points, /luminar_left_points]',
                    'output_topic' : '/merged/lidar',
                    'output_frame_id' : 'center_of_gravity',                    
                }],
                output='screen',
            ),   
        ])
