import os 

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
# Name of the current URDF file
    urdf_file_name = 'av21.urdf'

    # Get path of the URDF file
    urdf = os.path.join(
        get_package_share_directory('av21_description'),
        'urdf', urdf_file_name)

    # Load URDF file for state publisher
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Define robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'publish_frequency': 1.0,
            'ignore_timestamp': False,
            'use_tf_static': True,
            'robot_description': robot_desc,
        }])

    return LaunchDescription([
        robot_state_publisher
    ])
