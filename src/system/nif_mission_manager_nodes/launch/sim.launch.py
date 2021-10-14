import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    mm_param_default_file = os.path.join(
        get_package_share_directory('nif_mission_manager_nodes'),
        'config',
        'transitions.sim.yaml'
    )
    
    mission_manager_node = Node(
        package='nif_mission_manager_nodes',
        executable='nif_mission_manager_nodes_exe',
        parameters=[{
            "missions_file_path": mm_param_default_file,
            "velocity.zero": 0.0,
            "velocity.max": 67.0,
            "velocity.avoidance": 20.0,
            "velocity.pit_in": 25.0,
            "velocity.pit_out": 25.0,
            "velocity.slow_drive": 25.0,
            "safeloc.threshold_stop": 40.0,
            "safeloc.threshold_slow_down": 20.0,
            "safeloc.velocity_slow_down_max": 22.2,
            "safeloc.velocity_slow_down_min": 8.0,

            # Mission avoidance auto switch
            "mission.avoidance.auto_switch": True,
            "mission.avoidance.lap_count": 0,
            "mission.avoidance.previous_track_flag": 1,
            "mission.avoidance.lap_distance_min": 0,
            "mission.avoidance.lap_distance_max": 0,

        }],
        remappings=[
            ('out_mission_status', '/system/mission'),
        ]
    )

    return LaunchDescription([
        mission_manager_node
    ])
