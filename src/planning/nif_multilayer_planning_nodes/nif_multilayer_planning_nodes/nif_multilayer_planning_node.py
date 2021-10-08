import math
import configparser
import time
import json
import datetime
from typing import Tuple
import numpy as np
import sys, os
import csv
import rclpy

from ament_index_python import get_package_share_directory

from nif_msgs.msg import Perception3DArray, SystemStatus, MissionStatus
from nav_msgs.msg import Path, Odometry, OccupancyGrid
from std_msgs.msg import Bool
from nifpy_common_nodes.base_node import BaseNode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.node import Node
from sklearn.neighbors import KDTree
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point32

from nif_multilayer_planning_nodes.quintic_polynomial_planner import quintic_polynomials_planner

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

# TODO clean this out
lib_path_default = get_share_file('nif_multilayer_planning_nodes',
                                  "lib/GraphBasedLocalTrajectoryPlanner/graph_ltpl")

sys.path.insert(0, lib_path_default)
for dir_name in os.listdir(lib_path_default):
    dir_path = os.path.join(lib_path_default, dir_name)
    if os.path.isdir(dir_path):
        sys.path.insert(0, dir_path)
os.environ['OPENBLAS_NUM_THREADS'] = str(1)

# TODO : I don't know why but currently, we have to import the graph library in this order
from Graph_LTPL import Graph_LTPL
from imp_global_traj.src import *

class GraphBasedPlanner(rclpy.node.Node):
    
    def __init__(self):
        super().__init__('graph_based_planner_node')

        self.pose_resolution = 2.5
        self.maptrack_len = 100
        self.pit_maptrack_len = 100
        
#       Pre-initialize memory
#       TODO pre-load all these info
        self.msg = Path()
        self.msg.header.frame_id = "odom" #str(self.get_global_parameter('frames.global'))
        for idx in range(self.maptrack_len):
            pose = PoseStamped()
            pose.header.frame_id = self.msg.header.frame_id
            self.msg.poses.append(pose)

        # top level path (module directory)
        path_params = get_share_file('nif_multilayer_planning_nodes', 'params')
        path_inputs = get_share_file('nif_multilayer_planning_nodes', 'inputs')
        path_logs = get_share_file('nif_multilayer_planning_nodes', 'logs')

        self.sys_var_track = os.environ.get('TRACK').strip()

        self.declare_parameter("globtraj_input_path", os.path.join(path_inputs, "traj_ltpl_cl", self.sys_var_track, "traj_ltpl_cl.csv"))
        self.declare_parameter("graph_store_path", os.path.join(path_inputs, "track_offline_graphs", self.sys_var_track, "stored_graph.pckl"))
        self.declare_parameter("ltpl_offline_param_path", os.path.join(path_params, self.sys_var_track, "ltpl_config_offline.ini"))
        self.declare_parameter("ltpl_online_param_path", os.path.join(path_params, self.sys_var_track, "ltpl_config_online.ini"))
        self.declare_parameter("log_path", os.path.join(path_logs, self.sys_var_track, "graph_ltpl"))
        self.declare_parameter("graph_log_id", self.sys_var_track + datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S"))

        self.graph_config = configparser.ConfigParser()

        # define all relevant paths
        path_dict = {
            'globtraj_input_path': self.get_parameter("globtraj_input_path").get_parameter_value().string_value,
            'graph_store_path': self.get_parameter("graph_store_path").get_parameter_value().string_value,
            'ltpl_offline_param_path': self.get_parameter("ltpl_offline_param_path").get_parameter_value().string_value,
            'ltpl_online_param_path': self.get_parameter("ltpl_online_param_path").get_parameter_value().string_value,
            'log_path': self.get_parameter("log_path").get_parameter_value().string_value,
            'graph_log_id': self.get_parameter("graph_log_id").get_parameter_value().string_value,
        }

        self.get_logger().info(path_dict['globtraj_input_path'])
        self.get_logger().info(path_dict['graph_store_path'])
        self.get_logger().info(path_dict['ltpl_offline_param_path'])
        self.get_logger().info(path_dict['ltpl_online_param_path'])
        self.get_logger().info(path_dict['log_path'])
        self.get_logger().info(path_dict['graph_log_id'])

        self.odom_first_call = True

        # TODO: Loading the pit-in waypoints
        self.pit_in_wpt_file_path = None
        self.pit_in_wpt_stitched = [] # stitched pit-in waypoint [x,y,yaw_rad]
        self.pit_in_wpt_stitched_xy = [] # stitched pit-in waypoint [x,y]
        self.pit_in_wpt = [] # static pit-in waypoint from the waypoint file [x,y,yaw_rad]
        self.pit_in_wpt_xy = [] # static pit-in waypoint from the waypoint file [x,y]
        self.pit_in_wpt_quintic = [] # the pit-in waypoint segment stitching with the static pit-in waypoint from the ego position [x,y,yaw_rad]
        self.num_pit_in_wpt = 0
        self.pit_in_maptrack_len = None

        # TODO: Loading the pit-out waypoints
        self.pit_out_wpt_file_path = None
        self.pit_out_wpt_stitched = [] # stitched pit-out waypoint [x,y,yaw_rad]
        self.pit_out_wpt_stitched_xy = [] # stitched pit-out waypoint [x,y]
        self.pit_out_wpt = [] # static pit-out waypoint from the waypoint file [x,y,yaw_rad]
        self.pit_out_wpt_xy = [] # static pit-out waypoint from the waypoint file [x,y]
        self.pit_out_wpt_quintic = [] # the pit-out waypoint segment stitching with the static pit-in waypoint from the ego position [x,y,yaw_rad]
        self.num_pit_out_wpt = 0
        self.pit_out_maptrack_len = None

        # TODO: Loading the pit waypoints (entire)
        self.pit_wpt_file_path = None
        self.pit_wpt_stitched = [] # stitched pit waypoint [x,y,yaw_rad]
        self.pit_wpt_stitched_xy = [] # stitched pit waypoint [x,y]
        self.pit_wpt = [] # static pit waypoint from the waypoint file [x,y,yaw_rad]
        self.pit_wpt_xy = [] # static pit waypoint from the waypoint file [x,y]
        self.pit_wpt_local = [] # Locally planned waypoint segment in body frame [x,y,yaw_rad]
        self.num_pit_wpt = 0

        self.pit_in_wpt_gen_first_call = True
        self.pit_out_wpt_gen_first_call = True

        # TODO : file should be changed
        if self.sys_var_track == 'LOR':
            if not self.graph_config.read(path_dict['ltpl_offline_param_path']):
                raise ValueError(
                    'Specified graph config file does not exist or is empty!')
            self.pit_wpt_file_path = self.graph_config.get('PIT',"pit_wpt_file")
            self.pit_wpt_file_path = get_share_file('nif_multilayer_planning_nodes', self.pit_wpt_file_path)
            self.pit_wpt_maptrack_len = json.loads(self.graph_config.get('PIT','pit_wpt_maptrack_len'))
            self.max_wpt_switch_dist = json.loads(self.graph_config.get('PIT','max_wpt_switch_dist'))
            
        elif self.sys_var_track == 'IMS':
            if not self.graph_config.read(path_dict['ltpl_offline_param_path']):
                raise ValueError(
                    'Specified graph config file does not exist or is empty!')
            self.pit_wpt_file_path = self.graph_config.get('PIT',"pit_wpt_file")
            self.pit_wpt_file_path = get_share_file('nif_multilayer_planning_nodes', self.pit_wpt_file_path)
            self.pit_wpt_maptrack_len = json.loads(self.graph_config.get('PIT','pit_wpt_maptrack_len'))
            self.max_wpt_switch_dist = json.loads(self.graph_config.get('PIT','max_wpt_switch_dist'))

        elif self.sys_var_track == 'LG_SVL':
            if not self.graph_config.read(path_dict['ltpl_offline_param_path']):
                raise ValueError(
                    'Specified graph config file does not exist or is empty!')
            self.pit_wpt_file_path = self.graph_config.get('PIT',"pit_wpt_file")
            self.pit_wpt_file_path = get_share_file('nif_multilayer_planning_nodes', self.pit_wpt_file_path)
            self.pit_wpt_maptrack_len = json.loads(self.graph_config.get('PIT','pit_wpt_maptrack_len'))
            self.max_wpt_switch_dist = json.loads(self.graph_config.get('PIT','max_wpt_switch_dist'))

        else:
            raise ValueError('[nif_multilayer_planning_nodes] Track specification in driving_task.ini is wrong!')


        self.load_pit_waypoint()
        self.pit_tree = KDTree(self.pit_wpt_xy)

        self.mission_code = None
        self.mission_is_changed = True

        # TODO pre-load all these info
        self.pit_in_wpt_msg = Path()
        self.pit_in_wpt_msg.header.frame_id = "odom" #str(self.get_global_parameter('frames.global'))
        for idx in range(len(self.pit_in_wpt)):
            pose = PoseStamped()
            pose.header.frame_id = self.pit_in_wpt_msg.header.frame_id
            self.pit_in_wpt_msg.poses.append(pose)

        # TODO pre-load all these info
        self.pit_out_wpt_msg = Path()
        self.pit_out_wpt_msg.header.frame_id = "odom" #str(self.get_global_parameter('frames.global'))
        for idx in range(len(self.pit_out_wpt)):
            pose = PoseStamped()
            pose.header.frame_id = self.pit_out_wpt_msg.header.frame_id
            self.pit_out_wpt_msg.poses.append(pose)

        # Subscribers and Publisher
        self.local_maptrack_inglobal_pub = self.create_publisher(Path, 'out_local_maptrack_inglobal', rclpy.qos.qos_profile_sensor_data)
        self.pit_in_entire_inglobal_pub = self.create_publisher(Path, 'pit_in_entire_inglobal', rclpy.qos.qos_profile_sensor_data)
        self.pit_out_entire_inglobal_pub = self.create_publisher(Path, 'pit_out_entire_inglobal', rclpy.qos.qos_profile_sensor_data)
        self.pit_wpt_inglobal_pub = self.create_publisher(Path, 'pit_wpt_inglobal', rclpy.qos.qos_profile_sensor_data)
        self.pit_wpt_inbody_pub = self.create_publisher(Path, 'pit_wpt_inbody', rclpy.qos.qos_profile_sensor_data)
        self.veh_odom_sub = self.create_subscription(Odometry, 'in_ego_odometry', self.veh_odom_callback, rclpy.qos.qos_profile_sensor_data)
        self.perception_result_sub = self.create_subscription(Perception3DArray, 'in_perception_result', self.perception_result_callback, rclpy.qos.qos_profile_sensor_data)
        self.system_status_sub = self.create_subscription(SystemStatus, 'in_system_status', self.system_status_callback, 10)

        # For visualization
        self.pub_on_track = self.create_publisher(Bool, 'flg_on_track', rclpy.qos.qos_profile_sensor_data)
        self.pub_out_bound_path = self.create_publisher(Path, 'outter_bound_path', rclpy.qos.qos_profile_sensor_data)
        self.pub_in_bound_path = self.create_publisher(Path, 'inner_bound_path', rclpy.qos.qos_profile_sensor_data)
        self.pub_refline_path = self.create_publisher(Path, 'ref_path', rclpy.qos.qos_profile_sensor_data)
        self.pub_graph_node = self.create_publisher(PointCloud2, 'graph_nodes', rclpy.qos.qos_profile_sensor_data)

        self.out_of_track = None
        self.out_of_track_graph = None
        self.current_veh_odom = None
        self.current_veh_odom_yaw_rad = None

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        # only for the visualization
        timer_period_slow = 0.25  # seconds
        self.timer_slow = self.create_timer(timer_period_slow, self.timer_callback_slow)

        # ----------------------------------------------------------------------------------------------------------------------
        # INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------
        # intialize graph_ltpl-class
        self.ltpl_obj = Graph_LTPL(path_dict=path_dict, visual_mode=False, log_to_file=False)

        # calculate offline graph
        self.ltpl_obj.graph_init()

        # get track info
        self.on_track_msg = Bool
        self.out_bound_msg = Path()
        self.out_bound_msg.header.frame_id = "odom"
        # self.out_bound = (self.ltpl_obj.__graph_base.refline + self.ltpl_obj.__graph_base.normvec_normalized
                        # * np.expand_dims(self.ltpl_obj.__graph_base.track_width_right, 1))
        self.out_bound = self.ltpl_obj.right_bound()

        self.in_bound_msg = Path()
        self.in_bound_msg.header.frame_id = "odom"
        # self.in_bound = (self.ltpl_obj.__graph_base.refline + self.ltpl_obj.__graph_base.normvec_normalized
        #                 * np.expand_dims(self.ltpl_obj.__graph_base.track_width_left, 1))
        self.in_bound = self.ltpl_obj.left_bound()
        self.ref_line_msg = Path()
        self.ref_line_msg.header.frame_id = "odom"
        self.refline = self.ltpl_obj.ref_line()

        self.nodes_pos_cloud_msg = PointCloud2()
        self.nodes_pos_cloud_msg.header.frame_id = "odom"
        # node_poses = self.ltpl_obj.get_nodes()
        # for i, pos in enumerate(node_poses):
        #     self.nodes_pos_cloud_msg.points.append(Point32(pos[0], pos[1], 0.0))

        for i in range(len(self.out_bound)):
            pose = PoseStamped()
            pose.pose.position.x = self.out_bound[i][0]
            pose.pose.position.y = self.out_bound[i][1]
            self.out_bound_msg.poses.append(pose)
        for i in range(len(self.in_bound)):
            pose = PoseStamped()
            pose.pose.position.x = self.in_bound[i][0]
            pose.pose.position.y = self.in_bound[i][1]
            self.in_bound_msg.poses.append(pose)
        for i in range(len(self.refline)):
            pose = PoseStamped()
            pose.pose.position.x = self.refline[i][0]
            pose.pose.position.y = self.refline[i][1]
            self.ref_line_msg.poses.append(pose)

        self.pos_est = self.refline[0, :]
        self.heading_est = np.arctan2(np.diff(self.refline[0:2, 1]), np.diff(self.refline[0:2, 0])) - np.pi/2
        self.vel_est = 0.0

        self.last_pose = PoseStamped()
        self.last_pose.pose.position.x = 0.
        self.last_pose.pose.position.y = 0.
        self.last_pose.pose.position.z = 0.

        # ----------------------------------------------------------------------------------------------------------------------
        # ONLINE LOOP ----------------------------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------
        self.traj_set = {'straight': None}
        self.obj_list = []
        tic = time.time()

    def goal_pt_to_body(self, cur_odom_x, cur_odom_y, cur_odom_yaw_rad, 
                        global_pt_x, global_pt_y, global_pt_yaw_rad):
        body_x = (math.cos(-1 * cur_odom_yaw_rad) * (global_pt_x - cur_odom_x) -
                math.sin(-1 * cur_odom_yaw_rad) * (global_pt_y - cur_odom_y))
        body_y = (math.sin(-1 * cur_odom_yaw_rad) * (global_pt_x - cur_odom_x) +
                math.cos(-1 * cur_odom_yaw_rad) * (global_pt_y - cur_odom_y))
        body_yaw_rad = global_pt_yaw_rad - cur_odom_yaw_rad

        return body_x, body_y, body_yaw_rad

    def gen_local_path_using_global_goal_pt(self, cur_ego_pose_x, cur_ego_pose_y, cur_ego_pose_yaw_rad, cur_speed_mps, cur_acc_mpss,
                                                goal_pose_x_global, goal_pose_y_global, goal_pose_yaw_rad_global, goal_speed_mps, goal_acc_mpss,
                                                max_acc_mpss = 1.0, max_jerk = 1.0, dt = 0.1):
        # global goal point to local point
        goal_pose_x_local, goal_pose_y_local, goal_pose_yaw_rad_local = self.goal_pt_to_body(cur_ego_pose_x, cur_ego_pose_y, cur_ego_pose_yaw_rad,
                                                                                            goal_pose_x_global, goal_pose_y_global, goal_pose_yaw_rad_global)
        time, rx, ry, ryaw, rv, ra, rj = quintic_polynomials_planner(0.0, 0.0, 0.0, cur_speed_mps, cur_acc_mpss,
                                                                    goal_pose_x_local, goal_pose_y_local, goal_pose_yaw_rad_local, goal_speed_mps, goal_acc_mpss,
                                                                    max_acc_mpss, max_jerk, dt)
        self.pit_wpt_local = np.column_stack((np.array(rx),np.array(ry),np.array(ryaw))).tolist()

    def stitch_pit_in_waypoint(self, cur_ego_pose_x, cur_ego_pose_y, cur_ego_pose_yaw_rad, cur_speed_mps, cur_acc_mpss,
                               goal_pose_x, goal_pose_y, goal_pose_yaw_rad, goal_speed_mps, goal_acc_mpss,
                               max_acc_mpss = 1.0, max_jerk = 1.0, dt = 0.1):
        time, rx, ry, ryaw, rv, ra, rj = quintic_polynomials_planner(cur_ego_pose_x, cur_ego_pose_y, cur_ego_pose_yaw_rad, cur_speed_mps, cur_acc_mpss,
                                    goal_pose_x, goal_pose_y, goal_pose_yaw_rad, goal_speed_mps, goal_acc_mpss,
                                    max_acc_mpss, max_jerk, dt)
        self.pit_in_wpt_quintic = np.column_stack((np.array(rx),np.array(ry),np.array(ryaw))).tolist()
        self.pit_in_wpt_stitched = np.column_stack((np.array(rx + self.pit_in_wpt[:,0]),
                                                    np.array(ry + self.pit_in_wpt[:,1]),
                                                    np.array(ryaw + self.pit_in_wpt[:,2]))).tolist()
        self.pit_in_wpt_stitched_xy = np.column_stack((np.array(rx + self.pit_in_wpt[:,0]),
                                                       np.array(ry + self.pit_in_wpt[:,1]))).tolist()
        self.pit_in_maptrack_len = len(self.pit_in_wpt_stitched)

    def stitch_pit_out_waypoint(self, cur_ego_pose_x, cur_ego_pose_y, cur_ego_pose_yaw_rad, cur_speed_mps, cur_acc_mpss,
                               goal_pose_x, goal_pose_y, goal_pose_yaw_rad, goal_speed_mps, goal_acc_mpss,
                               max_acc_mpss = 1.0, max_jerk = 1.0, dt = 0.1):
        time, rx, ry, ryaw, rv, ra, rj = quintic_polynomials_planner(cur_ego_pose_x, cur_ego_pose_y, cur_ego_pose_yaw_rad, cur_speed_mps, cur_acc_mpss,
                                                                     goal_pose_x, goal_pose_y, goal_pose_yaw_rad, goal_speed_mps, goal_acc_mpss,
                                                                     max_acc_mpss, max_jerk, dt)
        self.pit_out_wpt_quintic = np.column_stack((np.array(rx),np.array(ry),np.array(ryaw))).tolist()
        self.pit_out_wpt_stitched = np.column_stack((np.array(rx + self.pit_out_wpt[:,0]),
                                                    np.array(ry + self.pit_out_wpt[:,1]),
                                                    np.array(ryaw + self.pit_out_wpt[:,2]))).tolist()
        self.pit_out_wpt_stitched_xy = np.column_stack((np.array(rx + self.pit_out_wpt[:,0]),
                                                       np.array(ry + self.pit_out_wpt[:,1]))).tolist()
        self.pit_out_maptrack_len = len(self.pit_out_wpt_stitched)

    def euler_to_quaternion(self, r):
        (yaw, pitch, roll) = (r[0], r[1], r[2])
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def system_status_callback(self, msg):
        if self.mission_code != msg.mission_status.mission_status_code:
            self.mission_is_changed = True
        self.mission_code = msg.mission_status.mission_status_code

    def track_inout_callback(self, msg):
        self.out_of_track = not msg.data

    def load_pit_waypoint(self):
        with open(self.pit_wpt_file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if math.isnan(float(row[0])) or math.isnan(float(row[1])) or math.isnan(float(row[2])) :
                    raise ValueError('[nif_multilayer_planning_nodes] Track specification in driving_task.ini is wrong!')
                self.pit_wpt.append([float(row[0]),float(row[1]),float(row[2])]) # order of x,y
                self.pit_wpt_xy.append([float(row[0]),float(row[1])])
                line_count += 1
        if len(self.pit_wpt) == 0:
            raise ValueError('[nif_multilayer_planning_nodes] Pit-in waypoint file is empty!')
        self.num_pit_wpt = len(self.pit_wpt)

        print("Pit waypoints are laoded.")

    def load_pit_in_waypoint(self):
        with open(self.pit_in_wpt_file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if math.isnan(float(row[0])) or math.isnan(float(row[1])) or math.isnan(float(row[2])) :
                    raise ValueError('[nif_multilayer_planning_nodes] Track specification in driving_task.ini is wrong!')
                self.pit_in_wpt.append([float(row[0]),float(row[1]),float(row[2])]) # order of x,y
                self.pit_in_wpt_xy.append([float(row[0]),float(row[1])])
                line_count += 1
        if len(self.pit_in_wpt) == 0:
            raise ValueError('[nif_multilayer_planning_nodes] Pit-in waypoint file is empty!')
        self.num_pit_in_wpt = len(self.pit_in_wpt)

        print("Pit in waypoints are laoded.")

    def load_pit_out_waypoint(self):
        with open(self.pit_out_wpt_file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if math.isnan(float(row[0])) or math.isnan(float(row[1])) or math.isnan(float(row[2])) :
                    raise ValueError('[nif_multilayer_planning_nodes] Track specification in driving_task.ini is wrong!')
                self.pit_out_wpt.append([float(row[0]),float(row[1]),float(row[2])]) # order of x,y
                self.pit_out_wpt_xy.append([float(row[0]),float(row[1])])
                line_count += 1
        if len(self.pit_out_wpt) == 0:
            raise ValueError('[nif_multilayer_planning_nodes] Pit-out waypoint file is empty!')
        self.num_pit_out_wpt = len(self.pit_out_wpt)

        print("Pit out waypoints are laoded.")

    def yaw_from_ros_quaternion(self, quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t3 = +2.0 * (quat.w * quat.z + quat.x * quat.y)
        t4 = +1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def veh_odom_callback(self, msg):
        self.current_veh_odom = msg

        self.pos_est = np.array([
            self.current_veh_odom.pose.pose.position.x,
            self.current_veh_odom.pose.pose.position.y
        ])

        self.current_veh_odom_yaw_rad = self.yaw_from_ros_quaternion(self.current_veh_odom.pose.pose.orientation)
        self.heading_est = self.current_veh_odom_yaw_rad - np.pi/2
        # TODO : could use perception instead of this, we gain both accuracy and performance
        self.vel_est = math.sqrt(pow(self.current_veh_odom.twist.twist.linear.x, 2)
                                 + pow(self.current_veh_odom.twist.twist.linear.y, 2))

        if self.odom_first_call is True:
            self.odom_first_call = self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                                        heading_est=self.heading_est)

    def perception_result_callback(self, msg):
        self.obj_list.clear()

        # TODO : From perception result, heading and velocity information should be added or extracted.
        # TODO : Check whether the heading is in global or body coordinate (Most likely, velocity is global but check once more)
        for perception_result in msg.perception_list:
            template_dict = {'X': perception_result.detection_result_3d.center.position.x,
                             'Y': perception_result.detection_result_3d.center.position.y,
                             'theta': self.yaw_from_ros_quaternion(perception_result.detection_result_3d.center.orientation),
                             'type': 'physical', 'id': perception_result.id, 'length': 5.0,
                             'v': perception_result.tracking_result_velocity_body_x_mps + self.vel_est}

            self.obj_list.append(template_dict)

    def timer_callback_slow(self):
        self.pub_out_bound_path.publish(self.out_bound_msg)
        self.pub_in_bound_path.publish(self.in_bound_msg)
        self.pub_refline_path.publish(self.ref_line_msg)
        self.pub_graph_node.publish(self.nodes_pos_cloud_msg)

    def timer_callback(self):

        # self.on_track_msg.data = self.ltpl_obj.check_out_of_track()
        # self.pub_on_track.publish(self.on_track_msg)
        # self.obj_list.clear()
        # template_dict = {'X': -1.27,
        #                      'Y': 2.58,
        #                      'theta': np.pi,
        #                      'type': 'physical', 'id': 0, 'length': 5.0,
        #                      'v': 0.01}

        # self.obj_list.append(template_dict)

        if self.odom_first_call is True:
            planning_graph = False
        else:
            planning_graph = True

        if self.mission_code == MissionStatus.MISSION_PIT_IN:
            # ---------------------------------------------------
            # ---------------------------------------------------
            # V3 : Using costmap, pass a collision free trajectory only in the pit area
            # ---------------------------------------------------
            # ---------------------------------------------------
            nearest_dist_list, nearest_ind_list = self.pit_tree.query([[self.current_veh_odom.pose.pose.position.x,
                                                                     self.current_veh_odom.pose.pose.position.y]], k=1)

            # TODO : just for safety but not very smart
            if nearest_dist_list[0][0] > self.max_wpt_switch_dist:
                return

            nearest_ind = nearest_ind_list[0][0]
            pit_in_path_msg = Path()
            pit_in_path_msg.header.stamp = self.get_clock().now().to_msg()
            pit_in_path_msg.header.frame_id = "odom"

            for i in range(self.pit_wpt_maptrack_len):
                idx = None
                if nearest_ind + i < len(self.pit_wpt):
                    idx = nearest_ind + i
                else:
                    idx = nearest_ind + i - len(self.pit_wpt)
                goal_pt_inglobal_x = self.pit_wpt[idx][0]
                goal_pt_inglobal_y = self.pit_wpt[idx][1]
                goal_pt_inglobal_yaw_rad = self.pit_wpt[idx][2]

                pose = PoseStamped()
                pose.pose.position.x = goal_pt_inglobal_x
                pose.pose.position.y = goal_pt_inglobal_y
                quat = self.euler_to_quaternion([goal_pt_inglobal_yaw_rad,0.0,0.0])
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                pit_in_path_msg.poses.append(pose)
            self.local_maptrack_inglobal_pub.publish(pit_in_path_msg)
            return

        elif self.mission_code == MissionStatus.MISSION_STANDBY:

            self.out_of_track_graph = self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                                        heading_est=self.heading_est)

            if self.out_of_track_graph is True:
                nearest_dist_list, nearest_ind_list = self.pit_tree.query([[self.current_veh_odom.pose.pose.position.x,
                                                                        self.current_veh_odom.pose.pose.position.y]], k=1)

                # TODO : just for safety but not very smart
                if nearest_dist_list[0][0] > self.max_wpt_switch_dist:
                    return

                nearest_ind = nearest_ind_list[0][0]
                pit_in_path_msg = Path()
                pit_in_path_msg.header.stamp = self.get_clock().now().to_msg()
                pit_in_path_msg.header.frame_id = "odom"

                for i in range(self.pit_wpt_maptrack_len):
                    idx = None
                    if nearest_ind + i < len(self.pit_wpt):
                        idx = nearest_ind + i
                    else:
                        idx = nearest_ind + i - len(self.pit_wpt)
                    goal_pt_inglobal_x = self.pit_wpt[idx][0]
                    goal_pt_inglobal_y = self.pit_wpt[idx][1]
                    goal_pt_inglobal_yaw_rad = self.pit_wpt[idx][2]

                    quat = self.euler_to_quaternion([goal_pt_inglobal_yaw_rad,0.0,0.0])

                    pose = PoseStamped()
                    pose.pose.position.x = goal_pt_inglobal_x
                    pose.pose.position.y = goal_pt_inglobal_y
                    pose.pose.orientation.x = quat[0]
                    pose.pose.orientation.y = quat[1]
                    pose.pose.orientation.z = quat[2]
                    pose.pose.orientation.w = quat[3]
                    pit_in_path_msg.poses.append(pose)
                self.local_maptrack_inglobal_pub.publish(pit_in_path_msg)
                return

            else:
                if planning_graph == False:
                    return
                # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
                # (here: brute-force, replace by sophisticated behavior planner)
                for sel_action_prev in ["straight", "follow"]:  # try to force 'right', else try next in list
                    if sel_action_prev in self.traj_set.keys():
                        break

                # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
                self.ltpl_obj.calc_paths(prev_action_id=sel_action_prev,
                                        object_list=self.obj_list)

                self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                                            vel_est=self.vel_est)[0]

                for sel_action_current in ["straight", "follow"]:  # try to force 'right', else try next in list
                    if sel_action_current in self.traj_set.keys():
                        break

                maptrack_inglobal = self.traj_set.get(sel_action_current)
                mp_len = len(maptrack_inglobal[0])
                if mp_len < len(self.msg.poses):
                    for idx in range(mp_len, len(self.msg.poses)):
                        self.msg.poses.pop()
                    self.maptrack_len = mp_len

                self.msg.header.stamp = self.get_clock().now().to_msg()

                # TODO pre-load all these info
                for idx in range(mp_len):
                    if idx < len(self.msg.poses):
                        pose = self.msg.poses[idx]
                    else:
                        pose = PoseStamped()
                        pose.header.frame_id = self.pit_in_wpt_msg.header.frame_id
                        self.msg.poses.append(pose)
                        self.maptrack_len += 1

                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = maptrack_inglobal[0][idx][1]  # for x
                    pose.pose.position.y = maptrack_inglobal[0][idx][2]  # for y

                    # TODO implement orientation comp in offline part
                    y_dot = (pose.pose.position.y - self.last_pose.pose.position.y) / self.pose_resolution
                    x_dot = (pose.pose.position.x - self.last_pose.pose.position.x) / self.pose_resolution
                    yaw = math.atan2(y_dot, x_dot)
                    pose.pose.orientation.x = 0.
                    pose.pose.orientation.z = math.sin(yaw / 2.)
                    pose.pose.orientation.y = 0.
                    pose.pose.orientation.w = math.cos(yaw / 2.)

                    # NOTE : Graph planner uses different heading coordinate
                    # quat = self.euler_to_quaternion([maptrack_inglobal[0][idx][3] - np.pi/2,0.0,0.0])
                    # pose.pose.orientation.x = quat[0]
                    # pose.pose.orientation.y = quat[1]
                    # pose.pose.orientation.z = quat[2]
                    # pose.pose.orientation.w = quat[3]

                    # self.get_logger().debug("%f, %f" % (pose.pose.position.x, pose.pose.position.y))
                    self.last_pose = pose

                self.msg.poses[0].pose.orientation = self.msg.poses[1].pose.orientation
                self.local_maptrack_inglobal_pub.publish(self.msg)



        elif self.mission_code == MissionStatus.MISSION_PIT_OUT or self.mission_code == MissionStatus.MISSION_PIT_STANDBY:
            # ---------------------------------------------------
            # ---------------------------------------------------
            # V3 : Using costmap, pass a collision free trajectory only in the pit area
            # ---------------------------------------------------
            # ---------------------------------------------------

            self.out_of_track_graph = self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                                        heading_est=self.heading_est)

            nearest_dist_list, nearest_ind_list = self.pit_tree.query([[self.current_veh_odom.pose.pose.position.x,
                                                                     self.current_veh_odom.pose.pose.position.y]], k=1)

            # TODO : just for safety but not very smart
            if nearest_dist_list[0][0] > self.max_wpt_switch_dist:
                return

            nearest_ind = nearest_ind_list[0][0]
            pit_in_path_msg = Path()
            pit_in_path_msg.header.stamp = self.get_clock().now().to_msg()
            pit_in_path_msg.header.frame_id = "odom"

            for i in range(self.pit_wpt_maptrack_len):
                idx = None
                if nearest_ind + i < len(self.pit_wpt):
                    idx = nearest_ind + i
                else:
                    idx = nearest_ind + i - len(self.pit_wpt)
                goal_pt_inglobal_x = self.pit_wpt[idx][0]
                goal_pt_inglobal_y = self.pit_wpt[idx][1]
                goal_pt_inglobal_yaw_rad = self.pit_wpt[idx][2]

                quat = self.euler_to_quaternion([goal_pt_inglobal_yaw_rad,0.0,0.0])

                pose = PoseStamped()
                pose.pose.position.x = goal_pt_inglobal_x
                pose.pose.position.y = goal_pt_inglobal_y
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                pit_in_path_msg.poses.append(pose)
            self.local_maptrack_inglobal_pub.publish(pit_in_path_msg)

        # TODO : Coordinate driving which means that there is no overtaking others. Maximum speed should be handeled in the velocity planner side as well.
        elif self.mission_code == MissionStatus.MISSION_SLOW_DRIVE:

            if planning_graph == False:
                return
            # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
            # (here: brute-force, replace by sophisticated behavior planner)
            for sel_action_prev in ["straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action_prev in self.traj_set.keys():
                    break

            # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
            try: 
                if self.out_of_track_graph is False:
                    self.ltpl_obj.calc_paths(prev_action_id=sel_action_prev,
                                            object_list=self.obj_list)

                    self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                                                vel_est=self.vel_est)[0]
                else:
                    return
            except: 
                self.out_of_track_graph = self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                                   heading_est=self.heading_est)

            for sel_action_current in ["straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action_current in self.traj_set.keys():
                    break

            maptrack_inglobal = self.traj_set.get(sel_action_current)
            mp_len = len(maptrack_inglobal[0])
            if mp_len < len(self.msg.poses):
                for idx in range(mp_len, len(self.msg.poses)):
                    self.msg.poses.pop()
                self.maptrack_len = mp_len

            self.msg.header.stamp = self.get_clock().now().to_msg()

            # TODO pre-load all these info
            for idx in range(mp_len):
                if idx < len(self.msg.poses):
                    pose = self.msg.poses[idx]
                else:
                    pose = PoseStamped()
                    pose.header.frame_id = self.pit_in_wpt_msg.header.frame_id
                    self.msg.poses.append(pose)
                    self.maptrack_len += 1

                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = maptrack_inglobal[0][idx][1]  # for x
                pose.pose.position.y = maptrack_inglobal[0][idx][2]  # for y
                # NOTE : Graph planner uses different heading coordinate
                # quat = self.euler_to_quaternion([maptrack_inglobal[0][idx][3]- np.pi/2,0.0,0.0])
                # pose.pose.orientation.x = quat[0]
                # pose.pose.orientation.y = quat[1]
                # pose.pose.orientation.z = quat[2]
                # pose.pose.orientation.w = quat[3]

                # TODO implement orientation comp in offline part
                y_dot = (pose.pose.position.y - self.last_pose.pose.position.y) / self.pose_resolution
                x_dot = (pose.pose.position.x - self.last_pose.pose.position.x) / self.pose_resolution
                yaw = math.atan2(y_dot, x_dot)
                pose.pose.orientation.x = 0.
                pose.pose.orientation.z = math.sin(yaw / 2.)
                pose.pose.orientation.y = 0.
                pose.pose.orientation.w = math.cos(yaw / 2.)

                # self.get_logger().debug("%f, %f" % (pose.pose.position.x, pose.pose.position.y))
                self.last_pose = pose

            self.msg.poses[0].pose.orientation = self.msg.poses[1].pose.orientation
            self.local_maptrack_inglobal_pub.publish(self.msg)
        else:
            if planning_graph == False:
                return
            # NOMINAL CASE
            # ------------------------------------------------
            # GREEN FLAG / EGO VEHICLE IS RUNNING ON THE TRACK
            # ------------------------------------------------

            # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
            # (here: brute-force, replace by sophisticated behavior planner)
            for sel_action_prev in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action_prev in self.traj_set.keys():
                    break

            # # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
            # self.ltpl_obj.calc_paths(prev_action_id=sel_action_prev,
            #                         object_list=self.obj_list)

            # self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
            #                                             vel_est=self.vel_est)[0]

            try: 
                if self.out_of_track_graph is False:
                    self.ltpl_obj.calc_paths(prev_action_id=sel_action_prev,
                                            object_list=self.obj_list)

                    self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                                                vel_est=self.vel_est)[0]
                else:
                    return
            except: 
                self.out_of_track_graph = self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                                   heading_est=self.heading_est)

            for sel_action_current in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action_current in self.traj_set.keys():
                    break

            maptrack_inglobal = self.traj_set.get(sel_action_current)
            mp_len = len(maptrack_inglobal[0])
            if mp_len < len(self.msg.poses):
                for idx in range(mp_len, len(self.msg.poses)):
                    self.msg.poses.pop()
                self.maptrack_len = mp_len

            self.msg.header.stamp = self.get_clock().now().to_msg()

    #       TODO pre-load all these info
            for idx in range(mp_len):
                if idx < len(self.msg.poses):
                    pose = self.msg.poses[idx]
                else:
                    pose = PoseStamped()
                    pose.header.frame_id = self.pit_in_wpt_msg.header.frame_id
                    self.msg.poses.append(pose)
                    self.maptrack_len += 1

                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = maptrack_inglobal[0][idx][1]  # for x
                pose.pose.position.y = maptrack_inglobal[0][idx][2]  # for y
                # NOTE : Graph planner uses different heading coordinate
                # quat = self.euler_to_quaternion([maptrack_inglobal[0][idx][3] + np.pi/2,0.0,0.0])
                # pose.pose.orientation.x = quat[0]
                # pose.pose.orientation.y = quat[1]
                # pose.pose.orientation.z = quat[2]
                # pose.pose.orientation.w = quat[3]

                # TODO implement orientation comp in offline part
                y_dot = (pose.pose.position.y - self.last_pose.pose.position.y) / self.pose_resolution
                x_dot = (pose.pose.position.x - self.last_pose.pose.position.x) / self.pose_resolution
                yaw = math.atan2(y_dot, x_dot)
                pose.pose.orientation.x = 0.
                pose.pose.orientation.z = math.sin(yaw / 2.)
                pose.pose.orientation.y = 0.
                pose.pose.orientation.w = math.cos(yaw / 2.)

                # self.get_logger().debug("%f, %f" % (pose.pose.position.x, pose.pose.position.y))
                self.last_pose = pose

            self.msg.poses[0].pose.orientation = self.msg.poses[1].pose.orientation
            self.local_maptrack_inglobal_pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    graph_based_planner_node = GraphBasedPlanner()
    rclpy.spin(graph_based_planner_node)
    graph_based_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
