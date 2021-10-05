import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():
    config = (
        os.path.join(
            get_package_share_directory("nif_points_preprocessor_nodes"),
            "config",
            "config.yaml",
        ),
        )

    return LaunchDescription(
        [
            Node(
                package="nif_points_preprocessor_nodes",
                executable="nif_points_preprocessor_nodes_exe",
                output={
                    'stdout': 'screen',
                    'stderr': 'screen',
                },
                emulate_tty=True,
                parameters=[config],
                remappings=[
                    # ("ins_gps", "novatel_bottom/inspva_"),
                ]
            ),
        ]
    )

