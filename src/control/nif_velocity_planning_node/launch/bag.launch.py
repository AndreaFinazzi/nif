import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

def generate_launch_description():
    nif_velocity_planning_node = Node(
        package='nif_velocity_planning_node',
        executable='nif_velocity_planning_node_exe',
        output='screen',
        remappings=[
            ('out_desired_velocity', 'velocity_planner/des_vel_test'),
            ('/velocity_planner/diagnostic', '/velocity_planner/diagnostic_test'),
            # ('in_reference_path', 'planning/graph/path_global'),
            ('in_reference_path', 'planning/path_global'),
            ('in_ego_odometry', '/aw_localization/ekf/odom'),
            ('in_wheel_speed_report', 'raptor_dbw_interface/wheel_speed_report'),
            ('in_imu_data', 'novatel_bottom/imu/data'),
            ('in_steering_report', 'raptor_dbw_interface/steering_report'),
            ('in_control_error', 'control_joint_lqr/lqr_error_test'),
        ],
        parameters=[{
                'odometry_timeout_sec' : 0.5,
                'path_timeout_sec' : 1.0,
                'use_mission_max_vel': True,
                'lateral_tire_model_factor' : 1.0,
            }]
    )
    return LaunchDescription([
        nif_velocity_planning_node,
    ])
