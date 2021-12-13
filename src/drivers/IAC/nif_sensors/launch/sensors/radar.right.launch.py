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

    # Print to terminal to notify right radar are enabled
    print('RIGHT RADAR ENABLED')
    can_interface = 'can3'

    # Right Radar
    socket_can_receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_radar_right',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': can_interface,
            'interval_sec':
            LaunchConfiguration('interval_sec'),
        }],            
        remappings=[
                ('/from_can_bus', '/radar_right/from_can_bus'),
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
        name='socket_can_sender_radar_right',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': can_interface,
            'timeout_sec':
            LaunchConfiguration('timeout_sec'),
        }],   
        remappings=[
                ('/to_can_bus', '/radar_right/to_can_bus'),
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

    radar_right_driver = Node(
        package='aptiv_esr_ros2',
        executable='esr_driver',
        output='screen',        
        namespace='radar_right',
    )

    radar_right_vehicle_interface = Node(
        package='aptiv_esr_ros2',
        executable='esr_vehicle_interface',
        output='screen',          
        remappings=[
                ('/esr_vehicle1', '/radar_right/esr_vehicle1'), 
                ('/esr_vehicle2', '/radar_right/esr_vehicle2'),
                ('/esr_vehicle3', '/radar_right/esr_vehicle3'),
                ('/esr_vehicle4', '/radar_right/esr_vehicle4'),
                ('/esr_vehicle5', '/radar_right/esr_vehicle5'),
                ('corrimu', '/novatel_bottom/corrimu'),
                ('best_vel', '/novatel_bottom/bestvel'),
        ]
    )

        # Right Radar
    radar_right_driver_mrr = Node(
        package="aptiv_mrr_driver",
        executable="aptiv_mrr_driver_node",
        output="screen",
        namespace="radar_right",
        parameters=[{
            'can_interface': can_interface,
            'frame_id': 'radar_starboard'
        }]
    )

    return LaunchDescription([
        radar_right_driver_mrr
    ])
