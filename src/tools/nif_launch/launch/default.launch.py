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
    pkg_dir_localization = get_package_share_directory('nif_localization_nodes')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_iac + '/launch/robot_description.launch.py'
        )
    )

    # vehicle_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         pkg_dir_iac + '/launch/vehicle.launch.py'
    #     )
    # )

    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/autonomy.launch.py'
        ),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/perception.launch.py'
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_localization + '/launch/default.launch.py'
        )
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir + '/launch/control.launch.py'
        )
    )

    launch_description = [
        robot_description_launch,
        autonomy_launch,
        perception_launch,
        localization_launch,
        control_launch
    ]

    return LaunchDescription(launch_description)  # , param_declarations])
