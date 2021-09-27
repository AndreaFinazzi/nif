import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

def generate_launch_description():

    params_file = os.path.join(
            get_package_share_directory("nif_aw_localization_nodes"),
            "config",
            "config.yaml"
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        'nif_aw_localization_param_file',
        default_value=params_file,
        description='nif_aw_localization_param'
    )

    aw_loaclization_node =  Node(
                package="nif_aw_localization_nodes",
                executable="nif_aw_localization_nodes_exe",
                output="screen",
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('nif_aw_localization_param_file')
                ],
                remappings=[
                    # Current set : Bottom INS Disabled // Top INS Enabled
                    # /novatel_bottom/bestvel is used to back-up solution when novatel_top/inspva heading is not published.  
                    ("in_inspva", "novatel_bottom/inspva"), # NOT USED
                    ("in_top_inspva", "novatel_top/inspva_nouse"), # HEADING
                    ("in_bestpos", "novatel_bottom/bestpos"), # POSE (X,Y)
                    ("in_imu", "novatel_bottom/imu/data"), # YAW RATE
                    ("in_bestvel", "novatel_bottom/bestvel"), #HEADING BACK UP SOLUTION
                    ("in_insstdev", "novatel_bottom/insstdev"), #INS STANDARD DEVIATION
                    ("in_wheel_speed_report", "raptor_dbw_interface/wheel_speed_report"), # WHEEL SPEED

                    ("out_odometry_ekf_estimated", "/aw_localization/ekf/odom"),
                    ("out_odometry_bestpos", "/aw_localization/ekf/odom_bestpos")
                ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            aw_loaclization_node
        ])
    

