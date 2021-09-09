from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from ament_index_python import get_package_share_directory



def generate_launch_description():
    # params_file = LaunchConfiguration(
    #     'params',
    #     default=[ThisLaunchFileDir(), '/launch_params.yaml'])

    # make sure the dbc file gets installed with the launch file
    # some_file = get_package_share_directory('nif_localization_gtsam_nodes') + \
    #                 ""

    return LaunchDescription(
        [
            Node(
                package='nif_localization_gtsam_nodes',
                executable='localization_node_exe',
                output='screen',
                # namespace='nif',
                parameters=[
                    {"body_frame_id": "/rear_axis_middle",
                     "global_frame_id": "/map"}
                ],
                remappings=[
                    ('gnss_01', '/novatel_top/inspva'),
                    ('gnss_02', '/novatel_bottom/inspva'),
                ],
            ),

            # Node(
            #     package='robot_localization',
            #     executable='navsat_transform_node',
            #     name='navsat_transform_node',
            #     output='screen',
            #     parameters=[
            #         {
            #             'broadcast_utm_transform': True
            #         }
            #     ],
            #     remappings=[
            #         ('odometry/filtered', '/nif/localization/ego_odom'),
            #         ('gps/fix', '/novatel_bottom/fix'),
            #         ('imu/data', '/novatel_bottom/imu')
            #     ],
            # )
            # LifecycleNode(
            #     package='kvaser_interface',
            #     node_executable='kvaser_can_bridge',
            #     node_name = "kvaser_interface",
            #     output='screen',
            #     namespace='',
            #     parameters=[params_file]),
        ])
