from genericpath import isdir
import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import csv
import argparse
from itertools import zip_longest
import os
from os.path import isfile, join
import numpy
# from nif_msgs.msg import MissionStatus

# /aw_localization/ekf/odom
# /aw_localization/ekf/odom_bestpos
# /aw_localization/ekf/top_bestpos
# /control_pool/control_cmd
# /detected_inner_distance
# /detected_outer_distance
# /joystick/steering_cmd
# /node_status/localization_resilient_node
# /wall_following_steering_cmd


class BagFileParser():
    def __init__(self, bag_file_path):
        self.conn = sqlite3.connect(bag_file_path)
        self.cursor = self.conn.cursor()

        # create a message type map
        topics_data = self.cursor.execute(
            "SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of,
                           name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of,
                         name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(
            type_of) for id_of, name_of, type_of in topics_data}

    def __del__(self):
        self.conn.close()

    # Return [(timestamp0, message0), (timestamp1, message1), ...]
    def get_messages(self, topic_name):

        topic_id = self.topic_id[topic_name]
        # Get from the db
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)).fetchall()
        # Deserialise all and timestamp them
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp, data in rows]


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("-b", "--bag-directory", type=str, default="/home/seong/Oct")
    # parser.add_argument("-b", "--bag-directory", type=str, default="bagfile")
    parser.add_argument("-o", "--output-path", type=str, default="result")

    args = parser.parse_args()

    bag_directory = args.bag_directory
    output_path = args.output_path

    bag_directory = os.path.realpath(bag_directory)
    assert os.path.isdir(bag_directory)

    print(bag_directory)

    # Create the output path if it doesnt exist
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    filename = (
        os.path.join(
            output_path, bag_directory + ".csv"
        )
    )

    VEH4_dir_list = [
        f for f in os.listdir(bag_directory)
        if isdir(join(bag_directory, f)) and "VEH4" in f
    ]

    write_target = []
    # track_id = []
    file_name = []
    # is_lidar = []
    # is_radar = []
    # is_camera = []
    # is_pylon = []
    # is_attacker = []
    # file_size = []
    # odom_pose_pose_x = []
    # odom_pose_pose_y = []
    # odom_pose_pose_z = []

    # odom_best_pose_pose_x = []
    # odom_best_pose_pose_y = []
    # odom_best_pose_pose_z = []

    # odom_top_pose_pose_x = []
    # odom_top_pose_pose_y = []
    # odom_top_pose_pose_z = []

    ##########
    # speed_list = []
    # operation_time = []
    # drive_distance = []
    # total_time = []

    for VEH4_dir in VEH4_dir_list:
        VEH4_dir_path = os.path.realpath(bag_directory + "/" + VEH4_dir)
        print(VEH4_dir_path)
        # distance = 0
        # oper_time = 0
        past_max_speed = -1
        # total_distance = 0
        size = 0

        for path, dirs, files in os.walk(VEH4_dir_path):
            for f in files:
                fp = os.path.join(path, f)
                size += os.path.getsize(fp)

        # file_size.append(size/(10**9))

        # for sensors....
        # sensors_dir_list = [
        #     f for f in os.listdir(VEH4_dir_path)
        #     if isdir(join(VEH4_dir_path, f)) and "sensors" in f
        # ]

        # if len(sensors_dir_list) != 0:
        #     for sensor_dir in sensors_dir_list:
        #         sensor_dir_path = os.path.realpath(
        #             VEH4_dir_path + "/" + sensor_dir)

        #         tmp_db3_file_list_sensor = [
        #             f for f in os.listdir(sensor_dir_path)
        #             if isfile(join(sensor_dir_path, f)) and "db3" in f
        #         ]

        #         db3_file_list_sensor = [
        #             file for file in tmp_db3_file_list_sensor if file.endswith(".db3")]

        #         # print(db3_file_list_sensor)

        #     longitudes = []
        #     lidars = []
        #     radars = []
        #     cameras = []
        #     rear_left = []
        #     rear_right = []
        #     drive_sec_time = []
        #     drive_nano_time = []
        #     drive_time = []

        #     for db3_file in db3_file_list_sensor:
        #         db3_file_path = os.path.realpath(
        #             sensor_dir_path + "/" + db3_file)

        #         # print(db3_file_path)
        #         parser = BagFileParser(db3_file_path)

        #         # print(parser.topic_msg_message)

        #         # with open(filename, mode='w') as file:
        #         # gps_writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        #         if "/novatel_bottom/bestpos" in parser.topic_msg_message:
        #             gps_msgs = parser.get_messages("/novatel_bottom/bestpos")
        #             for i in range(len(gps_msgs)):
        #                 longitudes.append(gps_msgs[i][1].lon)

        #         if "/luminar_front_points" in parser.topic_msg_message:
        #             lidar_msgs = parser.get_messages("/luminar_front_points")
        #             for i in range(len(lidar_msgs)):
        #                 lidars.append(lidar_msgs[i][1].width)

        #         if "/radar_front/esr_track" in parser.topic_msg_message:
        #             radar_msgs = parser.get_messages("/radar_front/esr_track")
        #             for i in range(len(radar_msgs)):
        #                 radars.append(radar_msgs[i][1].track_id)

        #         if "/camera/front_right/image/compressed" in parser.topic_msg_message:
        #             camera_msgs = parser.get_messages(
        #                 "/camera/front_right/image/compressed")
        #             for i in range(len(camera_msgs)):
        #                 cameras.append(camera_msgs[i][1].data)

        # for autonomy....
        autonomy_dir_list = [
            f for f in os.listdir(VEH4_dir_path)
            if isdir(join(VEH4_dir_path, f)) and "autonomy" in f
        ]

        for autonomy_dir in autonomy_dir_list:
            autonomy_dir_path = os.path.realpath(
                VEH4_dir_path + "/" + autonomy_dir)

            tmp_db3_file_list_autonomy = [
                f for f in os.listdir(autonomy_dir_path)
                if isfile(join(autonomy_dir_path, f)) and "db3" in f
            ]

            db3_file_list_autonomy = [
                file for file in tmp_db3_file_list_autonomy if file.endswith(".db3")]

            # print(db3_file_list_autonomy)

        # missions = []
        # distance = 0
        # total_operating = 0
        # total_distance = []
        # oper_time = 0

        # p = 1

        ekf_odom_time_sec = []
        ekf_odom_time_nano_sec = []
        ekf_odom_pose_pose_x = []
        ekf_odom_pose_pose_y = []
        ekf_odom_pose_pose_z = []

        ekf_odom_best_pose_pose_x = []
        ekf_odom_best_pose_pose_y = []
        ekf_odom_best_pose_pose_z = []

        ekf_odom_top_pose_pose_x = []
        ekf_odom_top_pose_pose_y = []
        ekf_odom_top_pose_pose_z = []


        control_sec = []
        control_nano_sec = []
        braking_control_cmd = []
        accel_control_cmd = []
        steering_control_cmd = []
        gear_control_cmd = []
        desired_steer_cmd = []
        desired_vel_cmd = []
        desired_accel_cmd = []

        detected_inner_distance = []
            
        detected_outer_distance = []

        steering_cmd = []

        node_status_id = []
        node_status_code = []

        wall_following_steering_cmd = []
        wall_following_sec = []
        wall_following_nanosec= [] 

        localization_status = []
        bottom_error = []
        top_error = []

        for db3_file in db3_file_list_autonomy:  # each db3
            db3_file_path_autonomy = os.path.realpath(
                autonomy_dir_path + "/" + db3_file)

            # average_speed = []
            # rear_left = []
            # rear_right = []
            # drive_time = []

            

            # print(db3_file_path_autonomy)
            parser = BagFileParser(db3_file_path_autonomy)

            # if "/system/mission" in parser.topic_msg_message:
            #     mission_msgs = parser.get_messages("/system/mission")
            #     for i in range(len(mission_msgs)):
            #         missions.append(mission_msgs[i][1].mission_status_code)

            if "/aw_localization/ekf/odom" in parser.topic_msg_message:
                ekf_odom_msgs = parser.get_messages(
                    "/aw_localization/ekf/odom")
                for i in range(len(ekf_odom_msgs)):
                    ekf_odom_pose_pose_x.append(
                        ekf_odom_msgs[i][1].pose.pose.position.x)
                    ekf_odom_pose_pose_y.append(
                        ekf_odom_msgs[i][1].pose.pose.position.y)
                    ekf_odom_pose_pose_z.append(
                        ekf_odom_msgs[i][1].pose.pose.position.z)
                    ekf_odom_time_sec.append(
                        ekf_odom_msgs[i][1].header.stamp.sec)
                    ekf_odom_time_nano_sec.append(
                        ekf_odom_msgs[i][1].header.stamp.nanosec)

            if "/aw_localization/ekf/odom_bestpos" in parser.topic_msg_message:
                ekf_odom_best_msgs = parser.get_messages(
                    "/aw_localization/ekf/odom_bestpos")
                for i in range(len(ekf_odom_best_msgs)):
                    ekf_odom_best_pose_pose_x.append(
                        ekf_odom_best_msgs[i][1].pose.pose.position.x)
                    ekf_odom_best_pose_pose_y.append(
                        ekf_odom_best_msgs[i][1].pose.pose.position.y)
                    ekf_odom_best_pose_pose_z.append(
                        ekf_odom_best_msgs[i][1].pose.pose.position.z)

            if "/aw_localization/ekf/top_bestpos" in parser.topic_msg_message:
                ekf_top_best_msgs = parser.get_messages(
                    "/aw_localization/ekf/top_bestpos")
                for i in range(len(ekf_top_best_msgs)):
                    ekf_odom_top_pose_pose_x.append(
                        ekf_top_best_msgs[i][1].pose.pose.position.x)
                    ekf_odom_top_pose_pose_y.append(
                        ekf_top_best_msgs[i][1].pose.pose.position.y)
                    ekf_odom_top_pose_pose_z.append(
                        ekf_top_best_msgs[i][1].pose.pose.position.z)

            if "/control_pool/control_cmd" in parser.topic_msg_message:
                control_command = parser.get_messages(
                    "/control_pool/control_cmd")
                for i in range(len(control_command)):

                    control_sec.append(
                        control_command[i][1].header.stamp.sec)
                    control_nano_sec.append(
                        control_command[i][1].header.stamp.nanosec)
                    braking_control_cmd.append(
                        control_command[i][1].braking_control_cmd.data)
                    accel_control_cmd.append(
                        control_command[i][1].accelerator_control_cmd.data)
                    steering_control_cmd.append(
                        control_command[i][1].steering_control_cmd.data)
                    gear_control_cmd.append(
                        control_command[i][1].gear_control_cmd.data)
                    desired_steer_cmd.append(
                        control_command[i][1].desired_steer_cmd.data)
                    desired_vel_cmd.append(
                        control_command[i][1].desired_velocity_cmd.data)
                    desired_accel_cmd.append(
                        control_command[i][1].desired_accel_cmd.data)

            if "/detected_inner_distance" in parser.topic_msg_message:
                inner_distance = parser.get_messages(
                    "/detected_inner_distance")
                for i in range(len(inner_distance)):
                    detected_inner_distance.append(
                        inner_distance[i][1].data)

            if "/detected_outer_distance" in parser.topic_msg_message:
                outer_distance = parser.get_messages(
                    "/detected_outer_distance")
                for i in range(len(outer_distance)):
                    detected_outer_distance.append(
                        outer_distance[i][1].data)

            if "/joystick/steering_cmd" in parser.topic_msg_message:
                steering = parser.get_messages(
                    "/joystick/steering_cmd")
                for i in range(len(steering)):
                    steering_cmd.append(
                        steering[i][1].data)

            if "/node_status/localization_resilient_node" in parser.topic_msg_message:
                node_status_msg = parser.get_messages(
                    "/node_status/localization_resilient_node")
                for i in range(len(node_status_msg)):
                    node_status_id.append(
                        node_status_msg[i][1].node_id)
                    node_status_code.append(
                        node_status_msg[i][1].node_status_code)

 
            # if "/wall_following_steering_cmd" in parser.topic_msg_message and "/wall_based_predictive_path" in parser.topic_msg_message:
            #     wall_steering_msg = parser.get_messages(
            #         "/wall_following_steering_cmd")
            #     wall_predictive_path_msg = parser.get_messages(
            #         "/wall_based_predictive_path")                    
            #     for i in range(len(wall_steering_msg)):
            #         print(i)
            #         wall_following_steering_cmd.append(
            #             wall_steering_msg[i][1].data)
            #         wall_following_sec.append(
            #             wall_predictive_path_msg[i][1].header.stamp.sec
            #         )
            #         wall_following_nanosec.append(
            #             wall_predictive_path_msg[i][1].header.stamp.nanosec
            #         )

            if "/wall_based_predictive_path" in parser.topic_msg_message:
                wall_predictive_path_msg = parser.get_messages(
                    "/wall_based_predictive_path")                    
                for i in range(len(wall_predictive_path_msg)):
                    wall_following_sec.append(
                        wall_predictive_path_msg[i][1].header.stamp.sec
                    )
                    wall_following_nanosec.append(
                        wall_predictive_path_msg[i][1].header.stamp.nanosec
                    )
            
            if "/wall_following_steering_cmd" in parser.topic_msg_message:
                wall_steering_msg = parser.get_messages(
                    "/wall_following_steering_cmd")                    
                for i in range(len(wall_steering_msg)):
                    wall_following_steering_cmd.append(
                        wall_steering_msg[i][1].data
                    )
          
            if "/aw_localization/ekf/status" in parser.topic_msg_message:
                ekf_status_msg = parser.get_messages(
                    "/aw_localization/ekf/status") 
                for i in range(len(ekf_status_msg)):
                    localization_status.append(
                        ekf_status_msg[i][1].localization_status_code)
                    bottom_error.append(
                        ekf_status_msg[i][1].bottom_error)
                    top_error.append(
                        ekf_status_msg[i][1].top_error)


# /aw_localization/ekf/status
# localization_code , bottom , top error


            # if "/raptor_dbw_interface/wheel_speed_report" in parser.topic_msg_message:
            #     speed_msgs = parser.get_messages("/raptor_dbw_interface/wheel_speed_report")
            #     for i in range(len(speed_msgs)):
            #         rear_left.append(speed_msgs[i][1].rear_left)
            #         rear_right.append(speed_msgs[i][1].rear_right)
            #         drive_time.append(speed_msgs[i][1].header.stamp.sec)
                    # drive_nano_time.append(speed_msgs[i][1].header.stamp.nanosec)

                # print(len(rear_left))
                # drive_nano_time = [x**-9 for x in drive_nano_time]
                # drive_time = drive_sec_time + drive_nano_time

            # print(len(drive_time))
        #     if(len(drive_time) != 0):

        #         average_speed = [(x + y) / 2 for x,y in zip(rear_left, rear_right)]  #rad/sec
        #         dt = (drive_time[-1] - drive_time[0]) / len(drive_time)
        #         # print(dt)

        #     else:
        #         continue
        #     tmp_speed = -1
        #     # print(len(average_speed))
        #     # print(len(average_speed))

        #     for current_speed in average_speed:
        #         # print(current_speed*0.381)
        #         if(current_speed * 0.27 > tmp_speed):
        #             max_speed = current_speed * 0.27
        #             tmp_speed = max_speed

        #         distance = distance + current_speed * 0.27 * dt

        #         if (current_speed * 0.27 > 3):
        #             oper_time = oper_time + dt
        #             # print(oper_time)

        #     if(max_speed > past_max_speed):
        #         final_max_speed = max_speed
        #         past_max_speed = final_max_speed

        # oper_time = oper_time / 60

        # speed_list.append(final_max_speed)
        # drive_distance.append(distance)
        # operation_time.append(oper_time)

        # if len(longitudes) != 0:
        #     average = numpy.mean(longitudes)

        #     if average > -120 and average < -110:
        #         track_id.append("LVMS")
        #     elif average > -95 and average < -75:
        #         track_id.append("IMS")
        #     else:
        #         track_id.append("Nan")

        # if len(lidars) == 0 and len(merged_lidars) == 0:
        #     is_lidar.append("X")
        # else:
        #     is_lidar.append("O")

        # if len(radars) == 0:
        #     is_radar.append("X")
        # else:
        #     is_radar.append("O")

        # if len(cameras) == 0:
        #     is_camera.append("X")
        # else:
        #     is_camera.append("O")

        # if 400 in missions:
        #     is_pylon.append("O")
        # else:
        #     is_pylon.append("X")

        # if 0 in missions:
        #     is_attacker.append("O")
        # else:
        #     is_attacker.append("X")

        file_name.append(VEH4_dir)

    with open(filename, mode='w') as file:
        data_writer = csv.writer(
            file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

        # file_name.insert(0, "Folder Name")
        # track_id.insert(0, "Track_ID")
        # is_lidar.insert(0, "Lidar(o/x)")
        # is_radar.insert(0, "Radar(o/x)")
        # is_camera.insert(0, "Camera(o/x)")
        # file_size.insert(0, "FileSize(GB)")
        # is_pylon.insert(0, "Pylon(o/x)")
        # is_attacker.insert(0, "Attacker(o/x)")

        ################################################
        ekf_odom_time_sec.insert(0, "EKF_sec")
        ekf_odom_time_nano_sec.insert(0, "EKF_nanosec")
        ekf_odom_pose_pose_x.insert(0, "EKF_x")
        ekf_odom_pose_pose_y.insert(0, "EKF_y")
        ekf_odom_pose_pose_z.insert(0, "EKF_z")

        ekf_odom_best_pose_pose_x.insert(0, "EKF_best_x")
        ekf_odom_best_pose_pose_y.insert(0, "EKF_best_Y")
        ekf_odom_best_pose_pose_z.insert(0, "EKF_best_Z")

        ekf_odom_top_pose_pose_x.insert(0, "EKF_top_x")
        ekf_odom_top_pose_pose_y.insert(0, "EKF_top_y")
        ekf_odom_top_pose_pose_z.insert(0, "EKF_top_z")


        control_sec.insert(0, "control_sec")
        control_nano_sec.insert(0, "control_nanosec")
        braking_control_cmd.insert(0, "braking_control_cmd")
        accel_control_cmd.insert(0, "accel_control_cmd")
        steering_control_cmd.insert(0, "steering_control_cmd")
        gear_control_cmd.insert(0, "gear_control_cmd")
        desired_steer_cmd.insert(0, "desired_steer_cmd")
        desired_vel_cmd.insert(0, "desired_vel_cmd")
        desired_accel_cmd.insert(0, "desired_accel_cmd")

        detected_inner_distance.insert(0, "detected_inner_distance")

        detected_outer_distance.insert(0, "detected_outer_distance")

        steering_cmd.insert(0, "steering_cmd")

        node_status_id.insert(0, "node_status_id")
        node_status_code.insert(0, "node_status_code")

        wall_following_sec.insert(0, "wall_following_sec")
        wall_following_nanosec.insert(0, "wall_following_nano_sec")
        wall_following_steering_cmd.insert(0, "wall_following_steering_cmd")        

        localization_status.insert(0, "localization_status")
        bottom_error.insert(0, "bottome_eror")
        top_error.insert(0, "top_error")

        ################################################

        # write_target.append(file_name[:len(file_name)])
        # write_target.append(track_id[:len(track_id)])
        # write_target.append(is_lidar[:len(is_lidar)])
        # write_target.append(is_radar[:len(is_radar)])
        # write_target.append(is_camera[:len(is_camera)])
        # write_target.append(file_size[:len(file_size)])
        # write_target.append(is_pylon[:len(is_pylon)])
        # write_target.append(is_attacker[:len(is_attacker)])

        ###################################################

        write_target.append(ekf_odom_time_sec[:len(ekf_odom_time_sec)])
        write_target.append(ekf_odom_time_nano_sec[:len(ekf_odom_time_nano_sec)])
        write_target.append(ekf_odom_pose_pose_x[:len(ekf_odom_pose_pose_x)])
        write_target.append(ekf_odom_pose_pose_y[:len(ekf_odom_pose_pose_y)])
        write_target.append(ekf_odom_pose_pose_z[:len(ekf_odom_pose_pose_z)])

        write_target.append(
            ekf_odom_best_pose_pose_x[:len(ekf_odom_best_pose_pose_x)])
        write_target.append(
            ekf_odom_best_pose_pose_y[:len(ekf_odom_best_pose_pose_y)])
        write_target.append(
            ekf_odom_best_pose_pose_z[:len(ekf_odom_best_pose_pose_z)])

        write_target.append(
            ekf_odom_top_pose_pose_x[:len(ekf_odom_top_pose_pose_x)])
        write_target.append(
            ekf_odom_top_pose_pose_y[:len(ekf_odom_top_pose_pose_y)])
        write_target.append(
            ekf_odom_top_pose_pose_z[:len(ekf_odom_top_pose_pose_z)])


        write_target.append(
            control_sec[:len(control_sec)])
        write_target.append(
            control_nano_sec[:len(control_nano_sec)])
        write_target.append(
            braking_control_cmd[:len(braking_control_cmd)])
        write_target.append(
            accel_control_cmd[:len(accel_control_cmd)])
        write_target.append(
            steering_control_cmd[:len(steering_control_cmd)])
        write_target.append(
            gear_control_cmd[:len(gear_control_cmd)])
        write_target.append(
            desired_steer_cmd[:len(desired_steer_cmd)])
        write_target.append(
            desired_vel_cmd[:len(desired_vel_cmd)])
        write_target.append(
            desired_accel_cmd[:len(desired_accel_cmd)])

        write_target.append(
            detected_inner_distance[:len(detected_inner_distance)])

        write_target.append(
            detected_outer_distance[:len(detected_outer_distance)])

        write_target.append(
            steering_cmd[:len(steering_cmd)])        

        write_target.append(
            node_status_id[:len(node_status_id)])
        write_target.append(
            node_status_code[:len(node_status_code)])

        write_target.append(
            wall_following_sec[:len(wall_following_sec)])
        write_target.append(
            wall_following_nanosec[:len(wall_following_nanosec)])     

        write_target.append(
            wall_following_steering_cmd[:len(wall_following_steering_cmd)])
        


        write_target.append(
            localization_status[:len(localization_status)])
        write_target.append(
            bottom_error[:len(bottom_error)])
        write_target.append(
            top_error[:len(top_error)])
        

        # write_target.append(drive_distance[:len(drive_distance)])
        # write_target.append(operation_time[:len(operation_time)])
        ###################################################

        data_writer.writerows(zip_longest(*write_target, fillvalue=''))

    # for db3_file in db3_file_list:
    #     db3_file_path = os.path.realpath(bag_directory + "/" + db3_file)
    #     db3_idx = db3_idx + 1
    #     parser = BagFileParser(db3_file_path)

    #     filename = (
    #             os.path.join(
    #                 output_path, "AC_bag_" + "{:08d}".format(db3_idx) + ".csv"
    #             )
    #         )

    #     write_target = []
    #     # num_of_ai = len(parser.topic_msg_message)

    #     # if num_of_ai != 0:
    #     with open(filename, mode='w') as file:
    #         gps_writer = csv.writer(file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    #         # for i in range(0, num_of_ai):
    #         #     target_ai_topic_name = ai_prefix_topic + str(i)

    #         #     ai_msgs = parser.get_messages(target_ai_topic_name)

    #         #     if len(ai_msgs) == 0:
    #         #         continue

    #         #     print(ai_msgs[0][1].header.stamp)
    #         gps_msgs = parser.get_messages("/novatel_bottom/bestpos")

    #         # latitudes = [gps_msgs[i][1].lat for i in range(len(gps_msgs))]  # add field name
    #         longitudes = [gps_msgs[i][1].lon for i in range(len(gps_msgs))]

    #         average = numpy.mean(longitudes)
    #         averages = []

    #         if average > -120 and average < -110:
    #             write_target.append(["track id", "lvms"])

    #         # latitudes.insert(0, "latitude")
    #         # longitudes.insert(0, "longitudes")
    #         # averages.insert(0, "Track id")

    #         # write_target.append(latitudes[:len(latitudes) - 1])
    #         # write_target.append(longitudes[:len(longitudes) - 1])
    #         # write_target.append(averages[:len(averages) - 1])

    #         for i in range(0, len(db3_file_list), 15):
    #             gps_writer.writerows(zip_longest(*write_target, fillvalue=''))

    # # bag_file_path = '/home/usrg/Downloads/IMS_Run_1/ims_first_run_0.db3'
    # # bag_file_path = '/home/usrg/Downloads/rosbag/lvms_test/autonomy_2022-01-05_10-56-18_0.db3'
    # bag_file_path = 'sensors_2022-01-06_09-12-12_0.db3'
    # parser = BagFileParser(bag_file_path)

    # output_file_name = 'result/gps_latLong_file.csv'  # output file name
    # gps_msgs = parser.get_messages("/novatel_bottom/bestpos")  # topic name
    # sampling_interval = 15  # rostopic sampling interval

    # latitudes = [gps_msgs[i][1].lat for i in range(len(gps_msgs))]  # add field name
    # longitudes = [gps_msgs[i][1].lon for i in range(len(gps_msgs))]  # add field name
    # # stamp = [gps_msgs[i][1].novatel_msg_header.percent_idle_time for i in range(len(gps_msgs))]  # add field name

    # with open(output_file_name, mode='w') as gps_latLong_file:
    #     gps_writer = csv.writer(gps_latLong_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    #     for i in range(0, len(gps_msgs), sampling_interval):
    #         # gps_writer.writerow([latitudes[i], longitudes[i], stamp[i]])  # write to csv
    #         gps_writer.writerow([latitudes[i], longitudes[i]])  # write to csv
