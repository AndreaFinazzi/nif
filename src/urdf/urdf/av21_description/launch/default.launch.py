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
        get_package_share_directory('av21_description'), 'urdf', urdf_file_name)
    # Load URDF file for state publisher
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    param_file_default = os.path.join(
        get_package_share_directory('av21_description'), 'config', 'default.yaml'
    )

    av21_description_param_file = DeclareLaunchArgument(
        'av21_description_param_file',
        default_value=param_file_default,
        description='Path to config file for nif_control_minimal'
    )

    # Shouldn't be necessary, but speeds things up
    ## tf2 - base_link to center_of_gravity
    base_link_tf_publisher = Node(
        name='base_link_tf_publisher',
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0.0', '0.0', '0.0', 'base_link', 'center_of_gravity']
    )

    # Define robot_state_publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('av21_description_param_file'),
            {
                'robot_description': robot_desc,
                'use_sim_time': 'falsamente_falso'
            }
        ])

    return LaunchDescription([
        base_link_tf_publisher,
        av21_description_param_file,
        robot_state_publisher
    ])
