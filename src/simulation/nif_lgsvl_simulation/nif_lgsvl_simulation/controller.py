#!/usr/bin/env python

'''
@file   controller.py
@auther USRG UGS
@date   2021-08-11
@brief  controller node for lgsvl simulation
'''
import numpy as np
import os
import pandas as pd
import math
import threading
import rospkg
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Int8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from lgsvl_msgs.msg import VehicleOdometry



def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# Calculate distance
def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

# Normalize angle [-pi, +pi]
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Global2Local
def global2local(ego_x, ego_y, ego_yaw, x_list, y_list):
    # Translational transform
    x_list = np.array(x_list)
    y_list = np.array(y_list)
    x_list = x_list - ego_x
    y_list = y_list - ego_y

    # Rotational transform
    rot_theta = -ego_yaw - np.pi/2
    c_theta = np.cos(rot_theta)
    s_theta = np.sin(rot_theta)

    rot_mat = np.array([[c_theta, -s_theta],
                        [s_theta, c_theta]])

    output_xy_list = np.matmul(rot_mat, np.array([x_list, y_list]))
    output_x_list = output_xy_list[0,:]
    output_y_list = output_xy_list[1,:]

    return output_x_list, output_y_list

# Find nearest point
def find_nearest_point(ego_x, ego_y, x_list, y_list):
    dist = np.zeros(len(x_list))
    for i in range(len(x_list)):
        dist[i] = calc_dist(x_list[i], y_list[i], ego_x, ego_y)
    
    near_ind = np.argmin(dist)
    near_dist = dist[near_ind]

    return near_dist, near_ind

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list, wpt_ind, wpt_look_ahead=0):
    # Global to Local coordinate
    local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)

    print(local_x_list)
    print(local_y_list)
    # Calculate yaw error
    target_wpt_ind = (wpt_ind + wpt_look_ahead)%x_list.shape[0] # look ahead
    error_yaw = math.atan2(local_y_list[(target_wpt_ind+1) % len(local_x_list)] - local_y_list[target_wpt_ind], \
                            local_x_list[(target_wpt_ind+1) % len(local_x_list)] - local_x_list[target_wpt_ind])
    print(local_y_list[target_wpt_ind])
    print(local_y_list[(target_wpt_ind+1) % len(local_x_list)])
    print(local_x_list[target_wpt_ind])
    print(local_x_list[(target_wpt_ind+1) % len(local_x_list)])
    print(ego_yaw)
    # Calculate errors
    error_y   = local_y_list[target_wpt_ind]
    error_yaw = normalize_angle(error_yaw)
    print("error_y: " + str(error_y))
    print("error_yaw: " + str(error_yaw))

    return error_y, error_yaw

class WaypointFollower(Node):
    def __init__(self):
        # ROS init
        super().__init__('wpt_follwer')
        self.rate = self.create_rate(100.0)

        # Params
        #self.timer_period = 0.01 # 1/Hz
        self.target_speed = 30
        self.MAX_STEER    = np.deg2rad(17.75)

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0

        self.wpt_look_ahead = 9   # [index]
        self.namespace = ''

        # Pub/Sub
        #self.pub_command = rospy.Publisher('/control', AckermannDriveStamped, queue_size=5)
        #self.sub_odom    = rospy.Subscriber('/simulation/bodyOdom', Odometry, self.callback_odom)

        self.pub_accel= self.create_publisher(Float32, self.namespace + '/raptor_dbw_interface/accelerator_pedal_cmd', 10)
        self.pub_steer = self.create_publisher(Float32, self.namespace + '/raptor_dbw_interface/steering_cmd', 10)
        self.pub_brake = self.create_publisher(Float32, self.namespace + '/raptor_dbw_interface/brake_cmd', 10)
        self.pub_gear = self.create_publisher(Int8, self.namespace + '/raptor_dbw_interface/gear_cmd', 10)

        self.sub_gps_top = self.create_subscription(NavSatFix, self.namespace + '/novatel_top/fix', self.callback_gps_top, 10)
        self.sub_imu_top= self.create_subscription(Imu, self.namespace + '/novatel_top/imu', self.callback_imu_top, 10)
        self.sub_vehicleodometry= self.create_subscription(VehicleOdometry, self.namespace + '/vehicle/odometry', self.callback_vehicleodometry, 10)


    def callback_gps_top(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        self.ego_x = msg.longitude
        self.ego_y = msg.latitude

    def callback_vehicleodometry(self, msg):
        self.ego_vx = msg.velocity

    def callback_imu_top(self, msg):
    
        # get euler from quaternion
        q = msg.orientation
        # q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q)

    # Controller
    def steer_control(self, error_y, error_yaw):
        """
        Steering control
        TODO-1 : Tuning your steering controller (Currently, P controller is implemented.).
        TODO-2 : Implement PI controller for steering angle control.
        """
        kp_y   = -30000.0 # P gain w.r.t. cross track error
        kp_yaw = -1.0 # P gain w.r.t. yaw error
        
        steer = kp_y*error_y + kp_yaw*error_yaw
        
        # Control limit
        # steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer

    def speed_control(self, error_v):
        """
        Speed control
        TODO-3: Tuning your speed controller (Currently, P controller is implemented.).
        """
        kp_v = 10.0
                
        return kp_v * error_v

    def publish_command(self, steer, accel, gear, brake):
        

        steer_msg = Float32()
        steer_msg.data = steer
        self.pub_steer.publish(steer_msg)

        accel_msg = Float32()
        accel_msg.data = accel
        self.pub_accel.publish(accel_msg)

        gear_msg = Int8()
        gear_msg.data = gear
        self.pub_gear.publish(gear_msg)
        
        brake_msg = Float32()
        brake_msg.data = brake
        self.pub_brake.publish(brake_msg)


def main(args=None):
    # Load Waypoint

    WPT_CSV_PATH = "./src/subtest/wpt_data/wpt_from_GPS.csv"
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    wpts_x = csv_data.values[:,0]
    wpts_y = csv_data.values[:,1]

    print("loaded wpt :", wpts_x.shape, wpts_y.shape)

    # Define controller
    rclpy.init(args=args)
    wpt_control = WaypointFollower()
    thread = threading.Thread(target=rclpy.spin, args=(wpt_control, ), daemon=True)
    thread.start()

    while rclpy.ok():
        # Get current state
        ego_x = wpt_control.ego_x
        ego_y = wpt_control.ego_y
        ego_yaw = wpt_control.ego_yaw
        ego_vx = wpt_control.ego_vx

        # Find the nearest waypoint
        _, near_ind = find_nearest_point(ego_x, ego_y, wpts_x, wpts_y)
        wpt_ind = near_ind

        # Lateral error calculation (cross-track error, yaw error)
        error_y, error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts_x, wpts_y, wpt_ind, wpt_look_ahead=wpt_control.wpt_look_ahead)

        # Longitudinal error calculation (speed error)
        error_v = wpt_control.target_speed - ego_vx

        # Control
        steer_cmd = wpt_control.steer_control(error_y, error_yaw)
        throttle_cmd = wpt_control.speed_control(error_v)
        gear_cmd = 6
        braking_cmd = 0.0

        # Publish command
        wpt_control.publish_command(steer_cmd, throttle_cmd, gear_cmd, braking_cmd)

        wpt_control.get_logger().info("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrackError=%.3f, YawError=%.3f, SpeedError=%.3f)." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v))
        wpt_control.rate.sleep()
        # wpt_control.get_logger().info("Woke up!!")
    wpt_control.get_logger().info("rclpy not ok")
    rclpy.shutdown()
    thread.join()   


if __name__ == '__main__':
    main()