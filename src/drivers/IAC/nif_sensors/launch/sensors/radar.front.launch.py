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

    # Print to terminal to notify front radar are enabled
    print('FRONT RADAR ENABLED')
    can_interface = 'can1'

    # Front Radar
    socket_can_receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_radar_front',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': can_interface,
            'interval_sec':
            LaunchConfiguration('interval_sec'),
        }],            
        remappings=[
                ('/from_can_bus', '/radar_front/from_can_bus'),
        ],
        output='screen')

    socket_can_receiver_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_receiver_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_receiver_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_receiver_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    socket_can_sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender_radar_front',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': can_interface,
            'timeout_sec':
            LaunchConfiguration('timeout_sec'),
        }],   
        remappings=[
                ('/to_can_bus', '/radar_front/to_can_bus'),
        ],
        output='screen')

    socket_can_sender_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_sender_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_sender_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_sender_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    radar_front_driver = Node(
        package='aptiv_esr_ros2',
        executable='esr_driver',
        output='screen',        
        namespace='radar_front',
    )

    radar_front_vehicle_interface = Node(
        package='aptiv_esr_ros2',
        executable='esr_vehicle_interface',
        output='screen',          
        remappings=[
                ('/esr_vehicle1', '/radar_front/esr_vehicle1'), 
                ('/esr_vehicle2', '/radar_front/esr_vehicle2'),
                ('/esr_vehicle3', '/radar_front/esr_vehicle3'),
                ('/esr_vehicle4', '/radar_front/esr_vehicle4'),
                ('/esr_vehicle5', '/radar_front/esr_vehicle5'),
                ('in_imu', '/novatel_bottom/rawimux'),
                ('in_best_vel', '/novatel_bottom/bestvel'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('interval_sec', default_value='0.01'),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        DeclareLaunchArgument('timeout_sec', default_value='0.01'),
        socket_can_receiver_node,
        socket_can_receiver_configure_event_handler,
        socket_can_receiver_activate_event_handler,
        socket_can_sender_node,
        socket_can_sender_configure_event_handler,
        socket_can_sender_activate_event_handler,
        radar_front_driver,
        radar_front_vehicle_interface
    ])
