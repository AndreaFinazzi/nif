import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    tf_prefix = LaunchConfiguration('tf_prefix', default='')
    # xacro_path = PathJoinSubstitution([ThisLaunchFileDir(), '../xacro/il15.xacro'])
    xacro_path = os.path.join(
        get_package_share_directory('il15_description'), 'xacro', "il15.xacro")
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ns = LaunchConfiguration('ns', default='/')
    # Load URDF file for state publisher
    with open(xacro_path, 'r') as f:
        robot_desc = f.read()

    param_file_default = os.path.join(
        get_package_share_directory('av21_description'), 'config', 'default.yaml'
    )

    il21_description_param_file = DeclareLaunchArgument(
            default_value=param_file_default,
            description='path to urdf.xacro file to publish')

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
            LaunchConfiguration('il21_description_param_file'),
            {
                'robot_description': robot_desc,
                'use_sim_time': 'falsamente_falso'
            }
        ])

    return LaunchDescription([
        il21_description_param_file,
        robot_state_publisher
    ])
