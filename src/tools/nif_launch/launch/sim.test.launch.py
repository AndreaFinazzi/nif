from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    pkg_dir = get_package_share_directory('nif_launch')
    pkg_dir_robot_description = get_package_share_directory('av21_description')
    pkg_dir_localization = get_package_share_directory('nif_localization_nodes')
    pkg_dir_waypoint_manager = get_package_share_directory('nif_waypoint_manager_nodes')
    pgk_dir_control_pure_pursuit = get_package_share_directory('nif_control_pure_pursuit_nodes')
    pgk_dir_lgsvl_simulation = get_package_share_directory('nif_lgsvl_simulation')

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_robot_description + '/launch/default.launch.py'
        )
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_localization + '/launch/default.launch.py'
        )
    )

    waypoint_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pkg_dir_waypoint_manager + '/launch/default.launch.py'
        )
    )

    control_pure_pursuit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pgk_dir_control_pure_pursuit + '/launch/default.launch.py'
        )
    )

    lgsvl_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pgk_dir_lgsvl_simulation + '/default.launch.py'
        )
    )

    # control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         pkg_dir + '/launch/control.launch.py'
    #     )
    # )

    launch_description = [
        robot_description_launch,
        localization_launch,
        waypoint_manager_launch,
        control_pure_pursuit_launch,
        lgsvl_simulation_launch
        # control_launch
    ]

    return LaunchDescription(launch_description)  # , param_declarations])
