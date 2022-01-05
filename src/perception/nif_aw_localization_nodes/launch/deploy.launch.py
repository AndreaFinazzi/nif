import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


IMS = 0
LOR = 1
IMS_SIM = 2
LVMS = 3
LVMS_SIM = 4
track = None

# get which track we are at
track_id = os.environ.get('TRACK').strip()

if track_id == "IMS" or track_id == "ims":
    track = IMS
elif track_id == "LOR" or track_id == "lor":
    track = LOR
elif track_id == "IMS_SIM" or track_id == "ims_sim":
    track = IMS_SIM
elif track_id == "LVMS" or track_id == "lvms":
    track = LVMS
elif track_id == "LVMS_SIM" or track_id == "lvms_sim":
    track = LVMS_SIM
else:
    raise RuntimeError("ERROR: Invalid track {}".format(track_id))

def generate_launch_description():

    config_file = None

    if track == LOR:
        config_file = 'config_lor.yaml'
    elif track == IMS:
        config_file = 'config_ims.yaml'
    elif track == IMS_SIM:
        config_file = 'config_lgsim.yaml'
    elif track == LVMS:
        config_file = 'config_lvms.yaml'
    elif track == LVMS_SIM:
        config_file = 'config_lvms_sim.yaml'
    else:
        raise RuntimeError("ERROR: invalid track provided: {}".format(track))

    params_file = os.path.join(
            get_package_share_directory("nif_aw_localization_nodes"),
            "config",
            config_file
        )
        
    param_file_launch_arg = DeclareLaunchArgument(
        'nif_aw_localization_param_file',
        default_value=params_file,
        description='nif_aw_localization_param'
    )

    aw_localization_node =  Node(
                package="nif_aw_localization_nodes",
                executable="nif_aw_localization_nodes_exe",
                output={
                    "stderr": "screen",
                    "stdout": "screen"
                },
                emulate_tty=True,
                parameters=[
                    LaunchConfiguration('nif_aw_localization_param_file'),
                    {
                        "bestvel_heading_update_velocity_thres" : 3.0,
                        "heading_initial_guess_enabled" : False,
                        "heading_initial_guess_deg"     : -215.0,
                        "heading_heading2_offset_deg"   : +90.0
                    }
                ],
                remappings=[
                    # Current set : Bottom INS Disabled // Top INS Enabled
                    # /novatel_bottom/bestvel is used to back-up solution when novatel_top/inspva heading is not published.  
                    ("in_inspva", "novatel_bottom/inspva"), # HEADING PRIORITY 1
                    ("in_top_inspva", "novatel_top/inspva"), # HEADING PRIORITY 2
                    ("in_bestpos", "novatel_bottom/bestgnsspos"), # POSE (X,Y)
                    ("in_top_bestpos", "novatel_top/bestgnsspos"), # POSE (X,Y)
                    ("in_imu", "novatel_bottom/rawimux"), # YAW RATE
                    ("in_bestvel", "novatel_bottom/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION)
                    ("in_bestvel_top", "novatel_top/bestvel"), #HEADING PRIORITY 3(BACK UP SOLUTION)
                    ("in_heading2", "novatel_bottom/heading2"), #HEADING PRIORITY 3(BACK UP SOLUTION)
                    ("in_top_heading2", "novatel_top/heading2"), #HEADING PRIORITY 3(BACK UP SOLUTION)
                    ("in_insstdev", "novatel_bottom/insstdev"), #INS STANDARD DEVIATION
                    ("in_top_insstdev", "novatel_top/insstdev"), #INS STANDARD DEVIATION
                    ("in_wheel_speed_report", "raptor_dbw_interface/wheel_speed_report"), # WHEEL SPEED

                    ("out_odometry_ekf_estimated", "/aw_localization/ekf/odom"),
                    ("out_odometry_bestpos", "/aw_localization/ekf/odom_bestpos"),
                    ("out_top_odometry_bestpos", "/aw_localization/ekf/top_bestpos"),
                    ('out_localization_error', '/aw_localization/ekf/error'),
                    ('out_localization_status', '/aw_localization/ekf/status'),
                    ('/debug', '/aw_localization/debug'),
                    ('/debug/measured_pose', '/aw_localization/debug/measured_pose'),
                    ('/estimated_yaw_bias', '/aw_localization/estimated_yaw_bias'),
                ]
    )

    return LaunchDescription(
        [
            param_file_launch_arg,
            aw_localization_node    
        ])
    

