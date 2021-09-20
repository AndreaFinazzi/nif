"""Launch an example driver that communicates using TCP"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():


    gps_device_bottom_arg = DeclareLaunchArgument(
        'gps_device_bottom', default_value=TextSubstitution(text='10.42.3.60:3001')
    )
    gps_device_top_arg = DeclareLaunchArgument(
        'gps_device_top', default_value=TextSubstitution(text='10.42.3.61:3001')
    )

    container = launch_ros.actions.ComposableNodeContainer(
        node_name='novatel_gps_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            launch_ros.descriptions.ComposableNode(
                package='novatel_gps_driver',
                node_plugin='novatel_gps_driver::NovatelGpsNode',
                node_name='novatel_gps',
                node_namespace='/novatel_bottom',
                parameters=[{
                    'connection_type': 'tcp',
                    'device': LaunchConfiguration('gps_device_bottom'),
                    'verbose': False,
                    'imu_sample_rate': -1.0,
                    'use_binary_messages': True,
                    'publish_novatel_positions': True,
                    'publish_imu_messages': True,
                    'publish_novatel_velocity': True,
                    'publish_novatel_psrdop2': True,
                    'imu_frame_id': '/imu_bottom',
                    'frame_id': '/gps_bottom',
                    # 'wait_for_sync': 'false'
                }]
            ),
            launch_ros.descriptions.ComposableNode(
                package='novatel_gps_driver',
                node_plugin='novatel_gps_driver::NovatelGpsNode',
                node_name='novatel_gps',
                node_namespace='/novatel_top',
                parameters=[{
                    'connection_type': 'tcp',
                    'device': LaunchConfiguration('gps_device_top'),
                    'verbose': False,
                    'imu_sample_rate': -1.0,
                    'use_binary_messages': True,
                    'publish_novatel_positions': True,
                    'publish_imu_messages': True,
                    'publish_novatel_velocity': True,
                    'publish_novatel_psrdop2': True,
                    'imu_frame_id': '/imu_top',
                    'frame_id': '/gps_top',
                    # 'wait_for_sync': 'false'
                }]
            )
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'error']

    )

    param_declarations = [
        DeclareLaunchArgument(
            name='connection_type', default_value='tcp',
            description='Connection type. tcp, serial, udp, ...'
        )]

    return LaunchDescription([
        gps_device_bottom_arg,
        gps_device_top_arg,
        container]) #, param_declarations])

