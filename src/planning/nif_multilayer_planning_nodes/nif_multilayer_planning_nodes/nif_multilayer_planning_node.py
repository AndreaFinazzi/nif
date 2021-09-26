import math
import configparser
import time
import json
import datetime
import numpy as np
import sys, os

from ament_index_python import get_package_share_directory

from nif_msgs.msg import Perception3DArray, SystemStatus, MissionStatus
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from nifpy_common_nodes.base_node import BaseNode
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import csv
import math
from sklearn.neighbors import KDTree
from geometry_msgs.msg import Quaternion

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
        
#       Pre-initialize memory
#       TODO pre-load all these info
        self.msg = Path()
        self.msg.header.frame_id = "odom" #str(self.get_global_parameter('frames.global'))
        for idx in range(self.maptrack_len):
            pose = PoseStamped()
            pose.header.frame_id = self.msg.header.frame_id
            self.msg.poses.append(pose)

        # top level path (module directory)
        path_assets = get_share_file('nif_multilayer_planning_nodes', 'assets')
        path_params = get_share_file('nif_multilayer_planning_nodes', 'params')
        path_inputs = get_share_file('nif_multilayer_planning_nodes', 'inputs')
        path_logs = get_share_file('nif_multilayer_planning_nodes', 'logs')

        self.sys_var_track = os.getenv('TRACK')
        track_param = configparser.ConfigParser()
        if not track_param.read(os.path.join(path_params, "driving_task.ini")):
            raise ValueError('Specified online parameter config file does not exist or is empty!')
        track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

        self.declare_parameter("globtraj_input_path", os.path.join(path_inputs, "traj_ltpl_cl", self.sys_var_track, "traj_ltpl_cl.csv"))
        self.declare_parameter("graph_store_path", os.path.join(path_inputs, "track_offline_graphs", self.sys_var_track, "stored_graph.pckl"))
        self.declare_parameter("ltpl_offline_param_path", os.path.join(path_params, self.sys_var_track, "ltpl_config_offline.ini"))
        self.declare_parameter("ltpl_online_param_path", os.path.join(path_params, self.sys_var_track, "ltpl_config_online.ini"))
        self.declare_parameter("log_path", os.path.join(path_logs, self.sys_var_track, "graph_ltpl"))
        self.declare_parameter("graph_log_id", datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S"))

        self.graph_config = configparser.ConfigParser()

        # TODO: Loading the pit-in waypoints
        self.pit_in_wpt_file_path = None
        self.pit_in_wpt = None
        self.pit_in_allowed_zone_min_x = None
        self.pit_in_allowed_zone_max_x = None
        self.pit_in_allowed_zone_min_y = None
        self.pit_in_allowed_zone_max_y = None
        self.pit_in_blocked_zone = None
        self.num_pit_in_wpt = 0
        self.pit_in_maptrack_len = 100
        self.pit_in_flg = False
        self.pit_in_wpt_maximum_vel = 0.0 # m/s
        self.pit_in_first_call = False
        self.pit_in_available_flg = False
        self.odom_first_call = True

        # TODO : file should be changed
        if track_specifier == 'LOR':
            if not self.graph_config.read("ltpl_offline_param_path"):
                raise ValueError(
                    'Specified graph config file does not exist or is empty!')
            self.pit_in_wpt_file_path = self.graph_config.get("PIT","pit_in_wpt_file")
            self.pit_in_wpt_maximum_vel = self.graph_config.getfloat("pit_in_wpt_maximum_vel")
            self.pit_in_allowed_zone_min_x = self.graph_config.getfloat("pit_in_allowed_x_min")
            self.pit_in_allowed_zone_max_x = self.graph_config.getfloat("pit_in_allowed_x_max")
            self.pit_in_allowed_zone_min_y = self.graph_config.getfloat("pit_in_allowed_y_min")
            self.pit_in_allowed_zone_max_y = self.graph_config.getfloat("pit_in_allowed_y_max")
            self.pit_in_blocked_zone = {'blocked_zone_for_pitIn': 
                                [[64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66],
                                [0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6],
                                np.array([[-20.54, 227.56], [23.80, 186.64]]),
                                np.array([[-23.80, 224.06], [20.17, 183.60]])]}
        elif track_specifier == 'IMS':
            if not self.graph_config.read("ltpl_offline_param_path"):
                raise ValueError(
                    'Specified graph config file does not exist or is empty!')
            # self.pit_in_wpt_file_path = '/home/usrg/Downloads/LOR_pit_lane_new_wpt.csv'
            self.pit_in_wpt_file_path = self.graph_config.get("PIT","pit_in_wpt_file")
            self.pit_in_wpt_maximum_vel = self.graph_config.getfloat("pit_in_wpt_maximum_vel")
            self.pit_in_allowed_zone_min_x = self.graph_config.getfloat("pit_in_allowed_x_min")
            self.pit_in_allowed_zone_max_x = self.graph_config.getfloat("pit_in_allowed_x_max")
            self.pit_in_allowed_zone_min_y = self.graph_config.getfloat("pit_in_allowed_y_min")
            self.pit_in_allowed_zone_max_y = self.graph_config.getfloat("pit_in_allowed_y_max")
            self.pit_in_blocked_zone = {'blocked_zone_for_pitIn':
                                [[64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66],
                                [0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6],
                                np.array([[-20.54, 227.56], [23.80, 186.64]]),
                                np.array([[-23.80, 224.06], [20.17, 183.60]])]}
        elif track_specifier == 'LG_SIM':
            if not self.graph_config.read("ltpl_offline_param_path"):
                raise ValueError(
                    'Specified graph config file does not exist or is empty!')
            self.pit_in_wpt_file_path = self.graph_config.get("PIT","pit_in_wpt_file")
            self.pit_in_wpt_maximum_vel = self.graph_config.getfloat("pit_in_wpt_maximum_vel")
            self.pit_in_allowed_zone_min_x = self.graph_config.getfloat("pit_in_allowed_x_min")
            self.pit_in_allowed_zone_max_x = self.graph_config.getfloat("pit_in_allowed_x_max")
            self.pit_in_allowed_zone_min_y = self.graph_config.getfloat("pit_in_allowed_y_min")
            self.pit_in_allowed_zone_max_y = self.graph_config.getfloat("pit_in_allowed_y_max")
            self.pit_in_blocked_zone = {'blocked_zone_for_pitIn': 
                                [[64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66],
                                [0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5, 6],
                                np.array([[-20.54, 227.56], [23.80, 186.64]]),
                                np.array([[-23.80, 224.06], [20.17, 183.60]])]}
        else:
            raise ValueError('[nif_multilayer_planning_nodes] Track specification in driving_task.ini is wrong!')

        if(self.pit_in_wpt_file_path == None):
            raise ValueError('[nif_multilayer_planning_nodes] Pit-in waypoint is not configured')
        if(self.pit_in_allowed_zone_min_x == None or self.pit_in_allowed_zone_max_x == None or
                    self.pit_in_allowed_zone_min_y == None or self.pit_in_allowed_zone_max_y == None):
            raise ValueError('[nif_multilayer_planning_nodes] Pit-in allowed zone is not configured')
        if(self.pit_in_allowed_zone_min_x >= self.pit_in_allowed_zone_max_x or
                self.pit_in_allowed_zone_min_y >= self.pit_in_allowed_zone_max_y):
            raise ValueError('[nif_multilayer_planning_nodes] Pit-in allowed zone is not properly configured!')

        self.load_pit_in_waypoint()
        self.pit_in_tree = KDTree(self.pit_in_wpt)

        # TODO pre-load all these info
        self.pit_in_wpt_msg = Path()
        self.pit_in_wpt_msg.header.frame_id = "odom" #str(self.get_global_parameter('frames.global'))
        for idx in range(self.pit_in_maptrack_len):
            pose = PoseStamped()
            pose.header.frame_id = self.pit_in_wpt_msg.header.frame_id
            self.pit_in_wpt_msg.poses.append(pose)

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

        # Subscribers and Publisher
        self.local_maptrack_inglobal_pub = self.create_publisher(Path, 'out_local_maptrack_inglobal', rclpy.qos.qos_profile_sensor_data)
        self.veh_odom_sub = self.create_subscription(Odometry, 'in_ego_odometry', self.veh_odom_callback, rclpy.qos.qos_profile_sensor_data)
        self.perception_result_sub = self.create_subscription(Perception3DArray, 'in_perception_result', self.perception_result_callback, rclpy.qos.qos_profile_sensor_data)
        # TODO---------------------------------------------------
        # TODO : Change the topic name / QOS for inout from deagyu
        # TODO----------------------------------------------------
        self.system_status_sub = self.create_subscription(SystemStatus, '/system/status', self.system_status_callback, 10)
        self.track_inout_bool_sub = self.create_subscription(Bool, 'todo', self.track_inout_callback, 10)

        self.out_of_track = None
        self.current_veh_odom = None

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # ----------------------------------------------------------------------------------------------------------------------
        # INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------
        # intialize graph_ltpl-class
        self.ltpl_obj = Graph_LTPL(path_dict=path_dict, visual_mode=False, log_to_file=False)

        # calculate offline graph
        self.ltpl_obj.graph_init()

        # set start pose based on first point in provided reference-line
        self.refline = graph_ltpl.imp_global_traj.src. \
            import_globtraj_csv.import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]

        self.pos_est = self.refline[0, :]
        self.heading_est = np.arctan2(np.diff(self.refline[0:2, 1]), np.diff(self.refline[0:2, 0])) - np.pi / 2
        self.vel_est = 0.0

        # set start pos
        # self.ltpl_obj.set_startpos(pos_est=self.pos_est,
        #                            heading_est=self.heading_est)
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

        self.cnt = 0

    def system_status_callback(self, msg):
        if msg.mission_status == msg.mission_status.MISSION_PIT_IN:
            self.pit_in_flg = True
        else:
            self.pit_in_flg = False

    def track_inout_callback(self,msg):
        self.out_of_track = not msg.data

    def load_pit_in_waypoint(self):
        with open(self.pit_in_wpt_file_path) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                if math.isnan(row[0]) or math.isnan(row[1]):
                    raise ValueError('[nif_multilayer_planning_nodes] Track specification in driving_task.ini is wrong!')
                self.pit_in_wpt.append([row[0],row[1]]) # order of x,y
                line_count += 1
        if len(self.pit_in_wpt) == 0:
            raise ValueError('[nif_multilayer_planning_nodes] Pit-in waypoint file is empty!')

        print("Pit in waypoints are laoded.")


    def yaw_from_ros_quaternion(self, quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
        t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        roll_x = math.atan2(t0, t1)
        return roll_x  # in radians

    def veh_odom_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.current_veh_odom = msg

        self.pos_est = np.array([
            self.current_veh_odom.pose.pose.position.x,
            self.current_veh_odom.pose.pose.position.y
        ])
        # TODO : could use perception instead of this, we gain both accuracy and performance
        self.vel_est = math.sqrt(pow(self.current_veh_odom.twist.twist.linear.x, 2)
                                 + pow(self.current_veh_odom.twist.twist.linear.y, 2)
                                 + pow(self.current_veh_odom.twist.twist.linear.z, 2))

        if self.odom_first_call == True:
            self.out_of_track = self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                                        heading_est=self.heading_est)
            self.odom_first_call = False

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

    def timer_callback(self):
        # TODO: temporal setup for lg sim
        self.out_of_track = False

        if self.pit_in_race_flg == False:
            # Green flag
            self.pit_in_first_call = False
            self.cnt = self.cnt +1
            if self.out_of_track == True:
                nearest_dist, nearest_ind = self.pit_in_tree.query((self.current_veh_odom.pose.pose.position.x,
                                                                    self.current_veh_odom.pose.pose.position.y), k=1)
                if nearest_dist < 1.0:
                    self.pit_in_wpt_msg.header.stamp = self.get_clock().now().to_msg()
                    for i in range(self.pit_in_maptrack_len):
                        if nearest_ind + i < self.num_pit_in_wpt:
                            self.pit_in_wpt_msg.poses[i].pose.position.x = self.pit_in_wpt[nearest_ind + i][0]
                            self.pit_in_wpt_msg.poses[i].pose.position.y = self.pit_in_wpt[nearest_ind + i][1]
                        else:
                            self.pit_in_wpt_msg.poses[i].pose.position.x = self.pit_in_wpt[nearest_ind + i - self.pit_in_maptrack_len][0]
                            self.pit_in_wpt_msg.poses[i].pose.position.y = self.pit_in_wpt[nearest_ind + i - self.pit_in_maptrack_len][1]
                    self.local_maptrack_inglobal_pub.publish(self.pit_in_wpt_msg)
                else:
                    return

            # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
            # (here: brute-force, replace by sophisticated behavior planner)
            for sel_action_prev in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action_prev in self.traj_set.keys():
                    break

            # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
            self.ltpl_obj.calc_paths(prev_action_id=sel_action_prev,
                                    object_list=self.obj_list)

            self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                                        vel_est=self.vel_est)[0]

            for sel_action_current in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                if sel_action_current in self.traj_set.keys():
                    break

            maptrack_inglobal = self.traj_set.get(sel_action_current)
            if len(maptrack_inglobal[0]) < self.maptrack_len:
                mp_len = len(maptrack_inglobal[0])
            else:
                mp_len = self.maptrack_len

            self.msg.header.stamp = self.get_clock().now().to_msg()

    #       TODO pre-load all these info
            for idx in range(mp_len):
                pose = self.msg.poses[idx]
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = maptrack_inglobal[0][idx][1]  # for x
                pose.pose.position.y = maptrack_inglobal[0][idx][2]  # for x
                
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
            # Vehicle should pit-in
            # TODO : Decide whether we can change the waypoint for the pit-in
            # Considering items :
            # 1. Switching zone
            # 2. Current velocity
            # 3. What else?
            # TODO : not layer, should be zone to be more simple
            pit_in_allowed_zone_flg = (self.pit_in_allowed_zone_min_x < self.current_veh_odom.pose.pose.position.x < self.pit_in_allowed_zone_max_x) \
                                        and (self.pit_in_allowed_zone_min_y < self.current_veh_odom.pose.pose.position.y < self.pit_in_allowed_zone_max_y)

            if (self.vel_est < self.pit_in_wpt_maximum_vel and pit_in_allowed_zone_flg == True and self.pit_in_first_call == False):
                self.pit_in_first_call = True
                nearest_dist, nearest_ind = self.pit_in_tree.query((self.current_veh_odom.pose.pose.position.x,
                                                                    self.current_veh_odom.pose.pose.position.y), k=1)
                self.pit_in_wpt_msg.header.stamp = self.get_clock().now().to_msg()
                for i in range(self.pit_in_maptrack_len):
                    if nearest_ind + i < self.num_pit_in_wpt:
                        self.pit_in_wpt_msg.poses[i].pose.position.x = self.pit_in_wpt[nearest_ind + i][0]
                        self.pit_in_wpt_msg.poses[i].pose.position.y = self.pit_in_wpt[nearest_ind + i][1]
                    else:
                        self.pit_in_wpt_msg.poses[i].pose.position.x = self.pit_in_wpt[nearest_ind + i - self.pit_in_maptrack_len][0]
                        self.pit_in_wpt_msg.poses[i].pose.position.y = self.pit_in_wpt[nearest_ind + i - self.pit_in_maptrack_len][1]
                self.local_maptrack_inglobal_pub.publish(self.pit_in_wpt_msg)

            elif self.pit_in_first_call == True:
                nearest_dist, nearest_ind = self.pit_in_tree.query((self.current_veh_odom.pose.pose.position.x,
                                                    self.current_veh_odom.pose.pose.position.y), k=1)
                self.pit_in_wpt_msg.header.stamp = self.get_clock().now().to_msg()
                for i in range(self.pit_in_maptrack_len):
                    if nearest_ind + i < self.num_pit_in_wpt:
                        self.pit_in_wpt_msg.poses[i].pose.position.x = self.pit_in_wpt[nearest_ind + i][0]
                        self.pit_in_wpt_msg.poses[i].pose.position.y = self.pit_in_wpt[nearest_ind + i][1]
                    else:
                        self.pit_in_wpt_msg.poses[i].pose.position.x = self.pit_in_wpt[nearest_ind + i - self.pit_in_maptrack_len][0]
                        self.pit_in_wpt_msg.poses[i].pose.position.y = self.pit_in_wpt[nearest_ind + i - self.pit_in_maptrack_len][1]
                self.local_maptrack_inglobal_pub.publish(self.pit_in_wpt_msg)

            else:
                # Can not change the waypoint, wait until the conditions are all satistied.
                if self.out_of_track == True:
                    return
                # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
                # (here: brute-force, replace by sophisticated behavior planner)
                # for sel_action_prev in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                for sel_action_prev in ["straight", "follow"]:  # prevent from the overtaking
                    if sel_action_prev in self.traj_set.keys():
                        break

                # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
                self.ltpl_obj.calc_paths(prev_action_id=sel_action_prev,
                                        object_list=self.obj_list)

                self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                                            vel_est=self.vel_est)[0]

                # for sel_action_current in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
                for sel_action_current in ["straight", "follow"]:  # prevent from the overtaking
                    if sel_action_current in self.traj_set.keys():
                        break

                maptrack_inglobal = self.traj_set.get(sel_action_current)
                if len(maptrack_inglobal[0]) < self.maptrack_len:
                    mp_len = len(maptrack_inglobal[0])
                else:
                    mp_len = self.maptrack_len

                self.msg.header.stamp = self.get_clock().now().to_msg()

        #       TODO pre-load all these info
                for idx in range(mp_len):
                    pose = self.msg.poses[idx]
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = maptrack_inglobal[0][idx][1]  # for x
                    pose.pose.position.y = maptrack_inglobal[0][idx][2]  # for x
                    
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
    # import cProfile, pstats
    # profiler = cProfile.Profile()
    
    rclpy.init(args=args)
    graph_based_planner_node = GraphBasedPlanner()

    # profiler.enable()
    rclpy.spin(graph_based_planner_node)
    # profiler.disable()
    
    graph_based_planner_node.destroy_node()
    
    # stats = pstats.Stats(profiler).sort_stats('tottime')
    # stats.dump_stats('multilayer_planner.prof')
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()
