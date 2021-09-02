import os
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

    nif_global_parameters_file = os.path.join(
        get_package_share_directory('nif_launch'),
        'config',
        'params.global.yaml'
    )

    global_parameters_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_common_nodes') + '/launch/parameters.launch.py'
        ),
        launch_arguments={
            'nif_global_parameters_file': nif_global_parameters_file
        }.items()
    )

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

    # waypoint_manager_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         pkg_dir_waypoint_manager + '/launch/default.launch.py'
    #     )
    # )

    multilayer_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_multilayer_planning_nodes') + '/launch/default.launch.py'
        ),

    )

    control_lqr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_control_lqr_nodes') + '/launch/default.launch.py'
        )
    )

    csl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('nif_control_safety_layer_nodes') + '/launch/default.launch.py'
        ),

    )

    lgsvl_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            pgk_dir_lgsvl_simulation + '/launch/default.launch.py'
        )
    )

    # control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         pkg_dir + '/launch/control.launch.py'
    #     )
    # )

    launch_description = [
        global_parameters_launch,
        robot_description_launch,
        localization_launch,
        # waypoint_manager_launch,
        multilayer_planning_launch,
        control_lqr_launch,
        csl_launch,
        lgsvl_simulation_launch
        # control_launch
    ]

    return LaunchDescription(launch_description)
