from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_dir = get_package_share_directory('nif_launch')
    pkg_dir_iac = get_package_share_directory('iac_launch')

    enable_cameras = DeclareLaunchArgument(
        'enable_cameras',
        default_value='true',
        description='Enable iac_launch/cameras.launch.py launch file.'
    )
    enable_radar = DeclareLaunchArgument(
        'enable_radars',
        default_value='false',
        description='Enable iac_launch/radar.launch.py launch file.'
    )

    novatel_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_iac + '/launch/novatel.launch.py'
        ),
    )

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_iac + '/launch/cameras.launch.py'
        ),
        condition=IfCondition(LaunchConfiguration('enable_cameras'))
    )

    radar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_iac + '/launch/radar.launch.py'
        ),
        condition=IfCondition(LaunchConfiguration('enable_radars'))

    )

    launch_description = [
        enable_cameras,
        enable_radar,
        novatel_launch,
        cameras_launch,
        radar_launch
    ]

    return LaunchDescription(launch_description)  # , param_declarations])
