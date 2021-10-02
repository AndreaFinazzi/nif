#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
import math
import threading
import rospkg
import rclpy


from rclpy.node import Node
from std_msgs.msg import Int16, Float32, Int8
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import Imu, NavSatFix
from autoware_auto_msgs.msg import VehicleKinematicState
from lgsvl_msgs.msg import VehicleOdometry
# from bangbang_msgs.msg import Float64String



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
def calc_error(ego_x, ego_y, ego_yaw, x_list, y_list, wpt_ind, start, wpt_look_ahead=0):
    # Global to Local coordinate
    local_x_list, local_y_list = global2local(ego_x, ego_y, ego_yaw, x_list, y_list)
    # print(local_x_list)
    # print(local_y_list)
    # Calculate yaw error
    # print("start: " + str(start))
    inpit = start
    startpoint = 0
    width = 10
    target_wpt_ind = (wpt_ind + wpt_look_ahead)%x_list.shape[0] # look ahead
    if(target_wpt_ind- startpoint < width and target_wpt_ind- startpoint > -width):
        inpit = False
    if(inpit):
        target_wpt_ind = startpoint
    error_yaw = math.atan2(local_y_list[(target_wpt_ind+1) % len(local_x_list)] - local_y_list[target_wpt_ind], \
                           local_x_list[(target_wpt_ind+1) % len(local_x_list)] - local_x_list[target_wpt_ind])
    # print(local_y_list[target_wpt_ind])
    # print(local_y_list[(target_wpt_ind+1) % len(local_x_list)])
    # print(local_x_list[target_wpt_ind])
    # print(local_x_list[(target_wpt_ind+1) % len(local_x_list)])
    # print(ego_yaw)
    # Calculate errors
    error_y   = local_y_list[target_wpt_ind]
    error_yaw = normalize_angle(error_yaw)
    # print("error_y: " + str(error_y))
    # print("error_yaw: " + str(error_yaw))

    return error_y, error_yaw, inpit

def init_wpts():
    WPT_CSV_PATH = "./src/subtest/wpt_data/raceline.csv"
    csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    pwpts_x = csv_data.values[:,0]
    pwpts_y = csv_data.values[:,1]
    lwpts_x = []
    lwpts_y = []
    for i in range(len(pwpts_x)):
        lwpts_x.append(pwpts_x[i]) #directly obtained from E
        lwpts_y.append(pwpts_y[i]) #directly obtained from N
    return np.array(lwpts_x ), np.array(lwpts_y )


class WaypointFollower(Node):
    def __init__(self, name = 'wpt_follwer', namespace = ''):
        # ROS init
        super().__init__(name)
        self.rate = self.create_rate(100.0)
        self.wpts_x, self.wpts_y = init_wpts()

        # Params
        #self.timer_period = 0.01 # 1/Hz
        self.target_speed_kmph = 110
        if(namespace== ''):
            self.target_speed_kmph = 100
        self.target_speed = self.target_speed_kmph * 1000 / 3600 # meter / second
        self.MAX_STEER    = np.deg2rad(17.75)

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0

        self.wpt_look_ahead = 15   # [index]
        self.namespace = namespace
        self.start = True
        self.action_plan = 'straight'

        #some constants
        self.lap_curr_num = 0
        if(self.namespace == '/car1'):
            self.lap_curr_num = self.lap_curr_num-1
        self.lap_obj_num = 1
        self.wpt_ind = 0

        self.safespeed = 1.0


        # Pub/Sub
        #self.pub_command = rospy.Publisher('/control', AckermannDriveStamped, queue_size=5)
        #self.sub_odom    = rospy.Subscriber('/simulation/bodyOdom', Odometry, self.callback_odom)

        # self.pub_accel= self.create_publisher(Float32, self.namespace + '/raptor_dbw_interface/accelerator_pedal_cmd', 10)
        # self.pub_steer = self.create_publisher(Float32, self.namespace + '/raptor_dbw_interface/steering_cmd', 10)
        # self.pub_brake = self.create_publisher(Float32, self.namespace + '/raptor_dbw_interface/brake_cmd', 10)
        # self.pub_gear = self.create_publisher(Int8, self.namespace + '/raptor_dbw_interface/gear_cmd', 10)
        self.pub_accel= self.create_publisher(Float32, self.namespace + '/joystick/accelerator_cmd', rclpy.qos.qos_profile_sensor_data)
        self.pub_steer = self.create_publisher(Float32, self.namespace + '/joystick/steering_cmd', rclpy.qos.qos_profile_sensor_data)
        self.pub_brake = self.create_publisher(Float32, self.namespace + '/joystick/brake_cmd', rclpy.qos.qos_profile_sensor_data)
        self.pub_gear = self.create_publisher(Int8, self.namespace + '/joystick/gear_cmd', rclpy.qos.qos_profile_sensor_data)

        self.sub_gps_top = self.create_subscription(NavSatFix, self.namespace + '/novatel_top/fix', self.callback_gps_top, 10)
        self.sub_imu_top= self.create_subscription(Imu, self.namespace + '/novatel_top/imu', self.callback_imu_top, 10)
        self.sub_vehicleodometry= self.create_subscription(VehicleOdometry, self.namespace + '/vehicle/odometry', self.callback_vehicleodometry, 10)

        self.sub_gps_only_localization= self.create_subscription(VehicleKinematicState, self.namespace + "/converted_data", self.callback_gps_only_localization, 10)
        self.sub_gps_odometry = self.create_subscription(Odometry, self.namespace + "/sensor/odom_ground_truth", self.callback_gps_odom, 10 )

        # subscribe map from planner
        self.sub_plan_path = self.create_subscription(Path, self.namespace + "/planning/graph/path_global", self.callback_plan_path, 10 )

        # self.sub_plan_mode = self.create_subscription(Float64String, self.namespace+ '/plan_mode', self.callback_plan_mode, 10)

    def callback_plan_mode(self, msg):
        if(not self.namespace== ''):
            return
        self.action_plan = msg.str
        if(self.action_plan == 'follow'):
            print(str(self.namespace) + "follow mode")  # speed to: "+ str(msg.speed - self.safespeed))
            self.target_speed = msg.speed - self.safespeed
        else:
            print(str(self.action_plan) + " mode")
            self.target_speed = self.target_speed_kmph * 1000 / 3600


    def callback_plan_path(self, msg):
        # print("new plan!")
        if(not self.namespace== ''):
            return
        for i in range(len(msg.poses)):
            self.wpts_x[i] = msg.poses[i].pose.position.x #directly obtained from E
            self.wpts_y[i] = msg.poses[i].pose.position.y #directly obtained from N


    def callback_gps_odom(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y

    def callback_gps_top(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        # self.ego_x = msg.longitude
        # self.ego_y = msg.latitude

    def callback_gps_only_localization(self, msg):
        pass
        # self.ego_x = msg.state.x
        # self.ego_y = msg.state.y

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
        kp_y   = -0.2 # P gain w.r.t. cross track error
        kp_yaw = -0.5 # P gain w.r.t. yaw error

        steer = kp_y*error_y + kp_yaw*error_yaw

        # Control limit
        # steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer
    ######################################################################
    def speed_control(self, error_v):
        """
        Speed control
        TODO-3: Tuning your speed controller (Currently, P controller is implemented.).
        """
        kp_v = 1000.0
        if error_v > 0.0:
            throttle = kp_v * error_v
        else:
            # print("giving 0 throttle error v: " + str(error_v))
            throttle = 0.0
            # throttle = 100.0
        return throttle

    def brake_control(self, error_v):

        db = 1
        if error_v < -db:
            # print(str(self.namespace)+" brake!!!!!!!!!!!!!1\n")
            brake = 3447379.0 #error_v*500000.0 #3447379.0  # insert proper value
        else:
            brake = 0.0
        return  brake
    #######################################################################
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

def main_func(wpt_control):
    ego_x = wpt_control.ego_x
    ego_y = wpt_control.ego_y
    ego_yaw = wpt_control.ego_yaw
    ego_vx = wpt_control.ego_vx
    # Find the nearest waypoint
    old_wpt_ind = wpt_control.wpt_ind
    _, near_ind = find_nearest_point(ego_x, ego_y, wpt_control.wpts_x, wpt_control.wpts_y)
    wpt_control.wpt_ind = near_ind
    # print("wpt_ind: " + str(wpt_control.wpt_ind) + " old_wpt_ind: " + str(old_wpt_ind))
    if old_wpt_ind > wpt_control.wpt_ind:
        wpt_control.lap_curr_num = wpt_control.lap_curr_num + 1
        # print("lap_curr_num: " + str(wpt_control.lap_curr_num))


    # Lateral error calculation (cross-track error, yaw error)
    error_y, error_yaw, start = calc_error(ego_x, ego_y, ego_yaw, wpt_control.wpts_x, wpt_control.wpts_y, wpt_control.wpt_ind, wpt_control.start, wpt_look_ahead=wpt_control.wpt_look_ahead )
    wpt_control.start = start
    if(not start and wpt_control.namespace== '' and wpt_control.action_plan != 'follow'):
        wpt_control.target_speed_kmph = 130
        wpt_control.target_speed = wpt_control.target_speed_kmph * 1000 / 3600 # meter / second
    # if(not start and not wpt_control.namespace== ''):
    #     wpt_control.target_speed_kmph = 90
    #     wpt_control.target_speed = wpt_control.target_speed_kmph * 1000 / 3600 # meter / second
    # Longitudinal error calculation (speed error)
    error_v = wpt_control.target_speed - ego_vx # meter / second

    # Control
    steer_cmd = wpt_control.steer_control(error_y, error_yaw)
    # if  wpt_control.lap_curr_num == wpt_control.lap_obj_num + 1 :
    #     throttle_cmd = 0.0
    #     braking_cmd = 3447379.0
    # else:
    throttle_cmd = wpt_control.speed_control(error_v)
    braking_cmd = wpt_control.brake_control(error_v)
    gear_cmd = 6

    # Publish command
    wpt_control.publish_command(steer_cmd, throttle_cmd, gear_cmd, braking_cmd)

    # wpt_control.get_logger().info("Commands: (steer=%.3f, accel=%.3f). Errors: (CrossTrackError=%.3f, YawError=%.3f, SpeedError=%.3f)." %(steer_cmd, throttle_cmd, error_y, error_yaw, error_v))
    # wpt_control.get_logger().info("Woke up!!")

def main(args=None):
    # Load Waypoint

    # WPT_CSV_PATH = "./src/subtest/wpt_data/raceline.csv"
    # csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
    # pwpts_x = csv_data.values[:,0]
    # pwpts_y = csv_data.values[:,1]
    # pwpts_z = csv_data.values[:,2]
    # print(type(pwpts_x))
    # a = 6378137.000           #WGS-84 Earth semimajor axis (m)
    # b = 6356752.314245179      #Derived Earth semiminor axis (m)
    # f = (a - b) / a           #Ellipsoid Flatness
    # # f_inv = 1.0 / f         #Inverse flattening
    # e_sq = f * (2 - f)        #Square of Eccentricity
    # lwpts_x = []
    # lwpts_y = []



    # lat0 = 39.7933495407008
    # lon0 = -86.238915265548
    # h0 = 0.0436621457338333

    # for i in range(len(pwpts_x)):
    ##lla2ecef conversion
    # lat_rad = np.radians(pwpts_x[i])
    # lon_rad = np.radians(pwpts_y[i])
    # s = math.sin(lat_rad)
    # N = a / math.sqrt(1 - e_sq* s * s)

    # sin_lat = math.sin(lat_rad)
    # cos_lat = math.cos(lat_rad)
    # cos_lon = math.cos(lon_rad)
    # sin_lon = math.sin(lon_rad)

    # ecef_x = (pwpts_z[i] + N) * cos_lat * cos_lon          #change to altitude for testing with lgsvl
    # ecef_y = (pwpts_z[i] + N) * cos_lat * sin_lon
    # ecef_z = (pwpts_z[i] + (1 - e_sq) * N) * sin_lat

    # ##ecef2enu conversion
    # lamb = math.radians(lat0)
    # phi = math.radians(lon0)
    # s = math.sin(lamb)
    # N = a / math.sqrt(1 - e_sq * s * s)

    # sin_lambda = math.sin(lamb)
    # cos_lambda = math.cos(lamb)
    # sin_phi = math.sin(phi)
    # cos_phi = math.cos(phi)

    # x0 = (h0 + N) * cos_lambda * cos_phi
    # y0 = (h0 + N) * cos_lambda * sin_phi
    # z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    # xd = ecef_x - x0
    # yd = ecef_y - y0
    # zd = ecef_z - z0

    # lwpts_x.append(-sin_phi * xd + cos_phi * yd) #east
    # lwpts_y.append(-cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd)
    #     lwpts_x.append(pwpts_x[i]) #directly obtained from E
    #     lwpts_y.append(pwpts_y[i]) #directly obtained from N

    # global wpts_x, wpts_y
    # wpts_x = np.array(lwpts_x )
    # wpts_y = np.array(lwpts_y )

    # print("loaded wpt :", wpts_x.shape, wpts_y.shape)

    # Define controller
    rclpy.init(args=args)
    wpt_control = WaypointFollower('wpt_follwer', '')
    wpt_control2 = WaypointFollower('wpt_follwer2', '/car1')

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(wpt_control )
    executor.add_node(wpt_control2)
    # Spin in a separate thread
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()


    while rclpy.ok():
        # Get current state
        main_func(wpt_control)
        main_func(wpt_control2)
        # print(wpt_control.wpts_x[:40])
        # print(wpt_control.start)
        wpt_control.rate.sleep()
    wpt_control.get_logger().info("rclpy not ok")
    rclpy.shutdown()
    thread.join()


if __name__ == '__main__':
    main()