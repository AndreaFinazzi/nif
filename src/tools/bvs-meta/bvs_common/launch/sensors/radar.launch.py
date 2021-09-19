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

    # Print to terminal to notify cameras are enabled
    print('RADARS ARE ENABLED')

    # Front Radar
    socket_can_receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_radar_front',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': 'can0',
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
            'interface': 'can0',
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

    radar_front_vehicle_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('bvs_common') + '/launch/front_radar.launch.py'
        )
    )




    # rear Radar
    socket_can_receiver_node_rear = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver_radar_rear',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': 'can4',
            'interval_sec':
            LaunchConfiguration('interval_sec'),
        }],            
        remappings=[
                ('/from_can_bus', '/radar_rear/from_can_bus'),
        ],
        output='screen')

    socket_can_receiver_configure_event_handler_rear = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_receiver_node_rear,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node_rear),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_receiver_activate_event_handler_rear = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_receiver_node_rear,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node_rear),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    socket_can_sender_node_rear = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender_radar_rear',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': 'can4',
            'timeout_sec':
            LaunchConfiguration('timeout_sec'),
        }],   
        remappings=[
                ('/to_can_bus', '/radar_rear/to_can_bus'),
        ],
        output='screen')

    socket_can_sender_configure_event_handler_rear = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_sender_node_rear,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node_rear),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_sender_activate_event_handler_rear = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_sender_node_rear,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node_rear),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    radar_rear_driver = Node(
        package='aptiv_esr_ros2',
        executable='esr_driver',
        output='screen',
        namespace='radar_rear',
    )

    radar_rear_vehicle_interface = Node(
        package='aptiv_esr_ros2',
        executable='esr_vehicle_interface',
        output='screen',
        remappings=[
                ('/esr_vehicle1', '/radar_rear/esr_vehicle1'),
                ('/esr_vehicle2', '/radar_rear/esr_vehicle2'),
                ('/esr_vehicle3', '/radar_rear/esr_vehicle3'),
                ('/esr_vehicle4', '/radar_rear/esr_vehicle4'),
                ('/esr_vehicle5', '/radar_rear/esr_vehicle5'),
        ]
    )

    # Right Radar
    radar_right_driver = Node(
        package="aptiv_mrr_driver",
        executable="aptiv_mrr_driver_node",
        output="screen",
        namespace="radar_right",
        parameters=[{
            'can_interface': 'can2',
            'frame_id': 'radar_front'
        }]
    )

    # Left Radar
    radar_left_driver = Node(
        package="aptiv_mrr_driver",
        executable="aptiv_mrr_driver_node",
        output="screen",
        namespace="radar_left",
        parameters=[{
            'can_interface': 'can5',
            'frame_id': 'radar_front'
        }]
    )

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
        DeclareLaunchArgument('interval_sec', default_value='0.01'),
        DeclareLaunchArgument('auto_configure', default_value='true'),
        DeclareLaunchArgument('auto_activate', default_value='true'),
        DeclareLaunchArgument('timeout_sec', default_value='0.01'),
        socket_can_receiver_node,
        socket_can_receiver_node_rear,
        socket_can_receiver_configure_event_handler,
        socket_can_receiver_configure_event_handler_rear,
        socket_can_receiver_activate_event_handler,
        socket_can_receiver_activate_event_handler_rear,
        socket_can_sender_node,
        socket_can_sender_node_rear,
        socket_can_sender_configure_event_handler,
        socket_can_sender_configure_event_handler_rear,
        socket_can_sender_activate_event_handler,
        socket_can_sender_activate_event_handler_rear,
        radar_front_driver,
        radar_front_vehicle_interface,
        radar_rear_driver,
        radar_rear_vehicle_interface,
        radar_right_driver,
        radar_left_driver,
        robot_state_publisher,
    ])
