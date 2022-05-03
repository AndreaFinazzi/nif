'''Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.'''


from sqlite3 import Time
from matplotlib.pyplot import grid
from sympy import false
import rclpy
import torch
import numpy as np
import sys
from nav_msgs.msg import Odometry
from rclpy.node import Node
import os
import time
import math
from ament_index_python import get_package_share_directory
from std_msgs.msg import Float64MultiArray

home_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),'.'))
sys.path.append(home_dir)
home_dir = os.path.abspath(os.path.join(os.path.dirname(__file__),'..','ac_track_db'))
sys.path.append(home_dir)

from oatomobile.baselines.torch.dim.model_ac import ImitativeModel
import pandas as pd
from collections import defaultdict

from nav_msgs.msg import Path
from scipy import interpolate, spatial
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from geometry_msgs.msg import PoseStamped
from nif_msgs.msg import ACTelemetryCarStatus

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

track_db_path = get_share_file('nif_imitative_planning_nodes', 'ac_track_db')

class ImitativePlanningNode(Node):

    def __init__(self):
        super().__init__('imitative_planning_node')

        self.verbose = True

        '''
        Testing purpose
        '''
        self.cnt = 0
        # timer = self.create_timer(0.05, self.timer_callback)
        self.traj_lib_pub = self.create_publisher(Path, 'imitative/traj_lib', rclpy.qos.qos_profile_sensor_data)
        self.prob_arr_pub = self.create_publisher(Float64MultiArray, 'imitative/prob', rclpy.qos.qos_profile_sensor_data)
        self.path_pub = self.create_publisher(Path, 'imitative/highest_imitation_prior', rclpy.qos.qos_profile_sensor_data)
        self.track_bound_l_pub = self.create_publisher(Path, 'imitative/input/track_bound_l_path', rclpy.qos.qos_profile_sensor_data)
        self.track_bound_r_pub = self.create_publisher(Path, 'imitative/input/track_bound_r_path', rclpy.qos.qos_profile_sensor_data)
        self.raceline_pub = self.create_publisher(Path, 'imitative/input/raceline_path', rclpy.qos.qos_profile_sensor_data)
        self.player_past_pub = self.create_publisher(Path, 'imitative/input/past_path', rclpy.qos.qos_profile_sensor_data)

        # Create a subscribers
        self.ego_odom_subscription = self.create_subscription(ACTelemetryCarStatus, '/ac/car_status_0', self.ego_veh_status_callback, rclpy.qos.qos_profile_sensor_data)
        self.opponent_1_subscription = self.create_subscription(Odometry, 'image', self.opponent_1_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_2_subscription = self.create_subscription(Odometry, 'image', self.opponent_2_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_3_subscription = self.create_subscription(Odometry, 'image', self.opponent_3_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_4_subscription = self.create_subscription(Odometry, 'image', self.opponent_4_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_5_subscription = self.create_subscription(Odometry, 'image', self.opponent_5_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_6_subscription = self.create_subscription(Odometry, 'image', self.opponent_6_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_7_subscription = self.create_subscription(Odometry, 'image', self.opponent_7_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_8_subscription = self.create_subscription(Odometry, 'image', self.opponent_8_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_9_subscription = self.create_subscription(Odometry, 'image', self.opponent_9_callback, rclpy.qos.qos_profile_sensor_data) 
        self.opponent_10_subscription = self.create_subscription(Odometry, 'image', self.opponent_10_callback, rclpy.qos.qos_profile_sensor_data) 

        self.opponent_1_last_update_time = 0.0
        self.opponent_2_last_update_time = 0.0
        self.opponent_3_last_update_time = 0.0
        self.opponent_4_last_update_time = 0.0
        self.opponent_5_last_update_time = 0.0
        self.opponent_6_last_update_time = 0.0
        self.opponent_7_last_update_time = 0.0
        self.opponent_8_last_update_time = 0.0
        self.opponent_9_last_update_time = 0.0
        self.opponent_1_last_update_time = 0.0

        self.opponent_1_odom_buffer = []
        self.opponent_2_odom_buffer = []
        self.opponent_3_odom_buffer = []
        self.opponent_4_odom_buffer = []
        self.opponent_5_odom_buffer = []
        self.opponent_6_odom_buffer = []
        self.opponent_7_odom_buffer = []
        self.opponent_8_odom_buffer = []
        self.opponent_9_odom_buffer = []
        self.opponent_10_odom_buffer = []

        self.opponent_1_odom_buffer_global_np = np.empty((0,3))
        self.opponent_2_odom_buffer_global_np = np.empty((0,3))
        self.opponent_3_odom_buffer_global_np = np.empty((0,3))
        self.opponent_4_odom_buffer_global_np = np.empty((0,3))
        self.opponent_5_odom_buffer_global_np = np.empty((0,3))
        self.opponent_6_odom_buffer_global_np = np.empty((0,3))
        self.opponent_7_odom_buffer_global_np = np.empty((0,3))
        self.opponent_8_odom_buffer_global_np = np.empty((0,3))
        self.opponent_9_odom_buffer_global_np = np.empty((0,3))
        self.opponent_10_odom_buffer_global_np = np.empty((0,3))

        self.opponent_1_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_2_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_3_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_4_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_5_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_6_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_7_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_8_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_9_odom_buffer_img_grid_np = np.empty((0,2))
        self.opponent_10_odom_buffer_img_grid_np = np.empty((0,2))
        
        self.cur_odom = Odometry()
        self.odom_buffer = []
        self.odom_buffer_np = np.empty((0,3), dtype=float)
        self.odom_r = None
        self.last_odom_update = self.get_clock().now()
        self.past_traj_path_body = Path()
        self.past_traj_path_body.header.frame_id = "base_link"

        '''
        Load static information
        '''
        inner_bound_file_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/lvms_inner_line.csv"
        outer_bound_file_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/lvms_outer_line.csv"
        raceline_file_path =    "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/race_line_w_field.csv"
        self.inner_bound_data = defaultdict(dict)
        self.outer_bound_data = defaultdict(dict)
        self.raceline_data = defaultdict(dict)
        self.inner_bound_data = pd.read_csv(inner_bound_file_path)
        self.outer_bound_data = pd.read_csv(outer_bound_file_path)
        self.race_line_data = pd.read_csv(raceline_file_path)

        self.raceline_field_name_x = " x_m"
        self.raceline_field_name_y = " y_m"
        self.raceline_data[self.raceline_field_name_x] = pd.DataFrame(self.race_line_data, columns=[self.raceline_field_name_x]).values
        # self.raceline_data[self.raceline_field_name_y] = pd.DataFrame(self.race_line_data, columns=[self.raceline_field_name_y]).multiply(-1).values
        self.raceline_data[self.raceline_field_name_y] = pd.DataFrame(self.race_line_data, columns=[self.raceline_field_name_y]).values
        self.raceline_global = np.array(np.column_stack((self.raceline_data[self.raceline_field_name_x],
                                                    self.raceline_data[self.raceline_field_name_y],
                                                    [0] * len(self.raceline_data[self.raceline_field_name_y]))))

        self.bound_field_name_x = "interpolated_x"
        self.bound_field_name_y = "interpolated_y"
        self.bound_field_name_z = "interpolated_z"

        self.inner_bound_data[self.bound_field_name_x] = pd.DataFrame(self.inner_bound_data, columns=[self.bound_field_name_x]).values
        # self.inner_bound_data[self.bound_field_name_y] = pd.DataFrame(self.inner_bound_data, columns=[self.bound_field_name_y]).multiply(-1).values
        self.inner_bound_data[self.bound_field_name_y] = pd.DataFrame(self.inner_bound_data, columns=[self.bound_field_name_y]).values
        self.inner_bound_data[self.bound_field_name_z] = pd.DataFrame(self.inner_bound_data, columns=[self.bound_field_name_z]).values
        self.outer_bound_data[self.bound_field_name_x] = pd.DataFrame(self.outer_bound_data, columns=[self.bound_field_name_x]).values
        # self.outer_bound_data[self.bound_field_name_y] = pd.DataFrame(self.outer_bound_data, columns=[self.bound_field_name_y]).multiply(-1).values
        self.outer_bound_data[self.bound_field_name_y] = pd.DataFrame(self.outer_bound_data, columns=[self.bound_field_name_y]).values
        self.outer_bound_data[self.bound_field_name_z] = pd.DataFrame(self.outer_bound_data, columns=[self.bound_field_name_z]).values

        self.track_bound_l_global = np.column_stack([self.inner_bound_data[self.bound_field_name_x],
                                                    self.inner_bound_data[self.bound_field_name_y],
                                                    [0] * len(self.inner_bound_data[self.bound_field_name_y])])
        self.track_bound_r_global = np.column_stack([self.outer_bound_data[self.bound_field_name_x],
                                                    self.outer_bound_data[self.bound_field_name_y],
                                                    [0] * len(self.outer_bound_data[self.bound_field_name_y])])

        self.raceline_tree = spatial.KDTree(self.raceline_global)
        self.track_bound_l_tree = spatial.KDTree(self.track_bound_l_global)
        self.track_bound_r_tree = spatial.KDTree(self.track_bound_r_global)
        self.racenline_idx = None
        self.track_bound_l_idx = None
        self.track_bound_r_idx = None

        self.track_bound_l_path_global = Path()
        self.track_bound_r_path_global = Path()
        self.race_line_path_global = Path()

        self.track_bound_l_path_global.header.frame_id = "odom"
        for i in range(self.track_bound_l_global.shape[0]):
            pt = PoseStamped()
            pt.header.frame_id = "odom"
            pt.pose.position.x = self.track_bound_l_global[i][0]
            pt.pose.position.y = self.track_bound_l_global[i][1]
            pt.pose.position.z = 0.0
            self.track_bound_l_path_global.poses.append(pt)
        self.track_bound_l_path_global_len = len(self.track_bound_l_path_global.poses)

        self.track_bound_r_path_global.header.frame_id = "odom"
        for i in range(self.track_bound_l_global.shape[0]):
            pt = PoseStamped()
            pt.header.frame_id = "odom"
            pt.pose.position.x = self.track_bound_r_global[i][0]
            pt.pose.position.y = self.track_bound_r_global[i][1]
            pt.pose.position.z = 0.0
            self.track_bound_r_path_global.poses.append(pt)
        self.track_bound_r_path_global_len = len(self.track_bound_r_path_global.poses)

        self.race_line_path_global.header.frame_id = "odom"
        for i in range(self.raceline_global.shape[0]):
            pt = PoseStamped()
            pt.header.frame_id = "odom"
            pt.pose.position.x = self.raceline_global[i][0]
            pt.pose.position.y = self.raceline_global[i][1]
            pt.pose.position.z = 0.0
            self.race_line_path_global.poses.append(pt)
        self.race_line_path_global_len = len(self.race_line_path_global.poses)

        '''
        inference configuration
        '''
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if self.verbose:
            print("Device : ", self.device)
            
        self.odom_buffer_length = 20
        self.oppo_buffer_length = 20
        self.grid_size = 200
        self.UNIT_GRID_DIST = 0.5
        self.num_pos_dim = 3
        self.future_lenth = 20
        self.past_lenth = 20
        self.output_shape = [self.future_lenth,self.num_pos_dim]
        self.input_shape = [self.past_lenth, self.num_pos_dim]
        self.use_depth = True
        self.use_lidar = False
        self.use_rgb = False
        self.dt = 0.1
        self.PRESET_X_GRID_SIZE = 200
        self.PRESET_Y_GRID_SIZE = 200
        self.CENTER_X_GRID = self.PRESET_X_GRID_SIZE / 2
        self.CENTER_Y_GRID = self.PRESET_Y_GRID_SIZE / 2
        self.CHANNLE = 1
        self.OVERLAY_BOUNDARY_VALUE = 50
        self.OVERLAY_RACELINE_VALUE = 125

        self.input_visual_feature = np.full((1,self.CHANNLE, self.PRESET_X_GRID_SIZE, self.PRESET_Y_GRID_SIZE), 0)
     
        '''
        load model
        '''
        self.model_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_weight_files/model-496.pt"
        self.model = ImitativeModel(
            output_shape=self.output_shape,
            num_pos_dim=self.num_pos_dim,
            input_shape=self.input_shape
        ).to(self.device)
        self.model.load_state_dict(
            torch.load(
                self.model_path,
                map_location=self.device,
            )
        )

        '''
        load trajectory lib
        '''
        self.trajectory_lib_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_trajectory_lib/solo.npy"
        self.arr = np.load(self.trajectory_lib_path)
        # Trajectory lip reshaping
        reshaped_traj_lib = np.reshape(self.arr, (self.arr.shape[0], self.output_shape[0], self.output_shape[1]))
        # print(reshaped_traj_lib)
        # print(reshaped_traj_lib.shape)
        # print(reshaped_traj_lib[:,:,0])
        # print(reshaped_traj_lib[:,:,0].shape)
        # print(reshaped_traj_lib[:,2])
        # print(reshaped_traj_lib[:,2] * 50)
        # reshaped_traj_lib[:,:,0] = reshaped_traj_lib[:,:,0] * 1
        # reshaped_traj_lib[:,:,1] = reshaped_traj_lib[:,:,1] * 1
        self.traj_lib = (
            torch.from_numpy(reshaped_traj_lib)
            .type(torch.FloatTensor)
            .to(self.device)
        )

        self.trajs_candidates = np.reshape(self.arr, (self.arr.shape[0], self.output_shape[0], self.output_shape[1]))

        '''
        SPEED
        '''
        self.slice_length_full = 150
        
        ps_global = PoseStamped()
        ps_global.header.frame_id = "odom"
        ps_body = PoseStamped()
        ps_body.header.frame_id = "base_link"

        self.sliced_track_bound_l = Path()
        self.sliced_track_bound_l.header.frame_id = "odom"
        self.sliced_track_bound_l.header.stamp = self.get_clock().now().to_msg()
        self.sliced_track_bound_l.poses = [ps_global] * self.slice_length_full

        self.sliced_track_bound_r = Path()
        self.sliced_track_bound_r.header.frame_id = "odom"
        self.sliced_track_bound_r.header.stamp = self.get_clock().now().to_msg()
        self.sliced_track_bound_r.poses = [ps_global] * self.slice_length_full

        self.sliced_raceline = Path()
        self.sliced_raceline.header.frame_id = "odom"
        self.sliced_raceline.header.stamp = self.get_clock().now().to_msg()
        self.sliced_raceline.poses = [ps_global] * self.slice_length_full

        self.sliced_track_bound_l_body = Path()
        self.sliced_track_bound_l_body.header.frame_id = "base_link"
        self.sliced_track_bound_l_body.header.stamp = self.get_clock().now().to_msg()
        self.sliced_track_bound_l_body.poses = [ps_body] * self.slice_length_full

        self.sliced_track_bound_r_body = Path()
        self.sliced_track_bound_r_body.header.frame_id = "base_link"
        self.sliced_track_bound_r_body.header.stamp = self.get_clock().now().to_msg()
        self.sliced_track_bound_r_body.poses = [ps_body] * self.slice_length_full

        self.sliced_race_line_body = Path()
        self.sliced_race_line_body.header.frame_id = "base_link"
        self.sliced_race_line_body.header.stamp = self.get_clock().now().to_msg()
        self.sliced_race_line_body.poses = [ps_body] * self.slice_length_full
    
    def goal_pt_to_body(self, cur_odom_x, cur_odom_y, cur_odom_yaw_rad, 
                        global_pt_x, global_pt_y, global_pt_yaw_rad):
        body_x = (math.cos(-1 * cur_odom_yaw_rad) * (global_pt_x - cur_odom_x) -
                math.sin(-1 * cur_odom_yaw_rad) * (global_pt_y - cur_odom_y))
        body_y = (math.sin(-1 * cur_odom_yaw_rad) * (global_pt_x - cur_odom_x) +
                math.cos(-1 * cur_odom_yaw_rad) * (global_pt_y - cur_odom_y))
        body_yaw_rad = global_pt_yaw_rad - cur_odom_yaw_rad

        return body_x, body_y, body_yaw_rad

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def overlay_lines(self,ego_odom):
        self.input_visual_feature.fill(0)
        self.odom_r = R.from_quat((ego_odom.pose.pose.orientation.x,
                                    ego_odom.pose.pose.orientation.y,
                                    ego_odom.pose.pose.orientation.z,
                                    ego_odom.pose.pose.orientation.w))

        ego_roll, ego_pitch, ego_yaw = self.euler_from_quaternion(ego_odom.pose.pose.orientation.x,
                                                             ego_odom.pose.pose.orientation.y,
                                                             ego_odom.pose.pose.orientation.z,
                                                             ego_odom.pose.pose.orientation.w)
        if self.odom_r == None:
            return -1

        ego_pt = [ego_odom.pose.pose.position.x,
                  ego_odom.pose.pose.position.y,
                  ego_odom.pose.pose.position.z]

        tic = time.time()

        # self.racenline_idx
        _,self.track_bound_l_idx = self.track_bound_l_tree.query(ego_pt)
        _,self.track_bound_r_idx = self.track_bound_r_tree.query(ego_pt)
        _,self.racenline_idx = self.raceline_tree.query(ego_pt)

        # print("self.track_bound_l_idx : ", self.track_bound_l_idx)
        # print("self.track_bound_r_idx : ", self.track_bound_r_idx)
        # print("self.racenline_idx : ", self.racenline_idx)

        toc = time.time()
        # print("cur idx : ", toc - tic)

        tic = time.time()

        if self.track_bound_l_idx < (self.slice_length_full/2):
            self.sliced_track_bound_l.poses[0:int(self.slice_length_full/2 - self.track_bound_l_idx)] = self.track_bound_l_path_global.poses[int(-(self.slice_length_full/2 - self.track_bound_l_idx)):]
            self.sliced_track_bound_l.poses[int(self.slice_length_full/2 - self.track_bound_l_idx):] = self.track_bound_l_path_global.poses[0:int(self.track_bound_l_idx+(self.slice_length_full/2))]
        elif self.track_bound_l_idx > self.track_bound_l_path_global_len - (self.slice_length_full/2):
            self.sliced_track_bound_l.poses[0:int(self.track_bound_l_path_global_len - self.track_bound_l_idx + self.slice_length_full/2)] = self.track_bound_l_path_global.poses[int(self.track_bound_l_idx-(self.slice_length_full/2)):]
            self.sliced_track_bound_l.poses[int(self.track_bound_l_path_global_len - self.track_bound_l_idx + self.slice_length_full/2):] = self.track_bound_l_path_global.poses[0:int(self.track_bound_l_idx+(self.slice_length_full/2) - self.track_bound_l_path_global_len -1)]
        else:
            self.sliced_track_bound_l.poses = self.track_bound_l_path_global.poses[int(self.track_bound_l_idx-(self.slice_length_full/2)):int(self.track_bound_l_idx+(self.slice_length_full/2))]

        if self.track_bound_r_idx < (self.slice_length_full/2):
            self.sliced_track_bound_r.poses[0:int(self.slice_length_full/2 - self.track_bound_r_idx)] = self.track_bound_r_path_global.poses[int(-(self.slice_length_full/2 - self.track_bound_r_idx)):]
            self.sliced_track_bound_r.poses[int(self.slice_length_full/2 - self.track_bound_r_idx):] = self.track_bound_r_path_global.poses[0:int(self.track_bound_r_idx+(self.slice_length_full/2))]
        elif self.track_bound_r_idx > self.track_bound_r_path_global_len - (self.slice_length_full/2):
            self.sliced_track_bound_r.poses[0:int(self.track_bound_r_path_global_len - self.track_bound_r_idx + self.slice_length_full/2)] = self.track_bound_r_path_global.poses[int(self.track_bound_r_idx-(self.slice_length_full/2)):]
            self.sliced_track_bound_r.poses[int(self.track_bound_r_path_global_len - self.track_bound_r_idx + self.slice_length_full/2):] = self.track_bound_r_path_global.poses[0:int(self.track_bound_r_idx+(self.slice_length_full/2) - self.track_bound_r_path_global_len -1)]
        else:
            self.sliced_track_bound_r.poses = self.track_bound_r_path_global.poses[int(self.track_bound_r_idx-(self.slice_length_full/2)):int(self.track_bound_r_idx+(self.slice_length_full/2))]

        if self.racenline_idx < (self.slice_length_full/2):
            self.sliced_raceline.poses[0:int(self.slice_length_full/2 - self.racenline_idx)] = self.race_line_path_global.poses[int(-(self.slice_length_full/2 - self.racenline_idx)):]
            self.sliced_raceline.poses[int(self.slice_length_full/2 - self.racenline_idx):] = self.race_line_path_global.poses[0:int(self.racenline_idx+(self.slice_length_full/2))]
        elif self.racenline_idx > self.race_line_path_global_len - (self.slice_length_full/2):
            self.sliced_raceline.poses[0:int(self.race_line_path_global_len - self.racenline_idx + self.slice_length_full/2)] = self.race_line_path_global.poses[int(self.racenline_idx-(self.slice_length_full/2)):]
            self.sliced_raceline.poses[int(self.race_line_path_global_len - self.racenline_idx + self.slice_length_full/2):] = self.race_line_path_global.poses[0:int(self.racenline_idx+(self.slice_length_full/2) - self.race_line_path_global_len -1)]
        else:
            self.sliced_raceline.poses = self.race_line_path_global.poses[int(self.racenline_idx-(self.slice_length_full/2)):int(self.racenline_idx+(self.slice_length_full/2))]

        # if(len(self.sliced_raceline.poses) != self.slice_length_full or len(self.sliced_track_bound_r.poses) != self.slice_length_full or len(self.sliced_track_bound_l.poses) != self.slice_length_full):
        #     print("1 : ",self.slice_length_full)
        #     print("2 : ",len(self.sliced_track_bound_l.poses))
        #     print("3 : ",len(self.sliced_track_bound_r.poses))
        #     print("4 : ",len(self.sliced_raceline.poses))

        # print("sliced_raceline.poses len : ", len(self.sliced_raceline.poses))
        # print("sliced_track_bound_r.poses len : ", len(self.sliced_track_bound_r.poses))
        # print("sliced_track_bound_l.poses len : ", len(self.sliced_track_bound_l.poses))

        toc = time.time()
        # print("slice : ", toc - tic)

        # buffer = tf2_ros.Buffer()
        # transform = buffer.lookup_transform("base_link","camera_link", rclpy.time.Time()) # Blocking
        # transform = await buffer.lookup_transform_async("base_link","camera_link",rclpy.time.Time())  #Non-Blocking

        # for local_coor_xyz in list((self.odom_r.apply(np.array((ego_pt))- sliced_track_bound_l))):

        tic = time.time()

        for global_pose_idx in range(min(len(self.sliced_track_bound_l.poses),len(self.sliced_track_bound_l_body.poses))):
            global_pose = self.sliced_track_bound_l.poses[global_pose_idx]
            body_x,body_y,_ = self.goal_pt_to_body(ego_odom.pose.pose.position.x,ego_odom.pose.pose.position.y,ego_yaw,
                                                global_pose.pose.position.x, global_pose.pose.position.y, 0.0)
            pt_local = PoseStamped()
            pt_local.header.frame_id = "base_link"
            pt_local.header.stamp = self.get_clock().now().to_msg()
            pt_local.pose.position.x = body_x
            pt_local.pose.position.y = body_y
            pt_local.pose.position.z = 0.0
            # if(len(self.sliced_track_bound_l_body.poses) != len(self.sliced_track_bound_l.poses)):
            #     print(len(self.sliced_track_bound_l_body.poses))
            #     print(len(self.sliced_track_bound_l.poses))
            #     print(global_pose_idx)
            #     # assert false
            self.sliced_track_bound_l_body.poses[global_pose_idx] = pt_local

            grid_x_idx = int(self.CENTER_X_GRID - int(body_x/(self.UNIT_GRID_DIST/self.PRESET_X_GRID_SIZE)))
            grid_y_idx = int(self.CENTER_Y_GRID - int(body_y/(self.UNIT_GRID_DIST/self.PRESET_Y_GRID_SIZE)))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.input_visual_feature[0,0,grid_x_idx,grid_y_idx] = self.OVERLAY_BOUNDARY_VALUE

        for global_pose_idx_r in range(min(len(self.sliced_track_bound_r.poses), len(self.sliced_track_bound_r_body.poses))):
            global_pose = self.sliced_track_bound_r.poses[global_pose_idx_r]
            body_x,body_y,_ = self.goal_pt_to_body(ego_odom.pose.pose.position.x,ego_odom.pose.pose.position.y,ego_yaw,
                                                global_pose.pose.position.x, global_pose.pose.position.y, 0.0)
            pt_local = PoseStamped()
            pt_local.header.frame_id = "base_link"
            pt_local.header.stamp = self.get_clock().now().to_msg()                                                
            pt_local.pose.position.x = body_x
            pt_local.pose.position.y = body_y
            pt_local.pose.position.z = 0.0
            # if(len(self.sliced_track_bound_r_body.poses) != len(self.sliced_track_bound_r.poses)):
            #     print(len(self.sliced_track_bound_r_body.poses))
            #     print(len(self.sliced_track_bound_r.poses))
            #     print(global_pose_idx_r)
            #     # assert false
            self.sliced_track_bound_r_body.poses[global_pose_idx_r] = pt_local

            grid_x_idx = int(self.CENTER_X_GRID - int(body_x/(self.UNIT_GRID_DIST/self.PRESET_X_GRID_SIZE)))
            grid_y_idx = int(self.CENTER_Y_GRID - int(body_y/(self.UNIT_GRID_DIST/self.PRESET_Y_GRID_SIZE)))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.input_visual_feature[0,0,grid_x_idx,grid_y_idx] = self.OVERLAY_BOUNDARY_VALUE

        for global_pose_idx_r in range(min(len(self.sliced_raceline.poses), len(self.sliced_race_line_body.poses))):
            global_pose = self.sliced_raceline.poses[global_pose_idx_r]    
            body_x,body_y,_ = self.goal_pt_to_body(ego_odom.pose.pose.position.x,ego_odom.pose.pose.position.y,ego_yaw,
                                                global_pose.pose.position.x, global_pose.pose.position.y, 0.0)
            pt_local = PoseStamped()
            pt_local.header.frame_id = "base_link"
            pt_local.header.stamp = self.get_clock().now().to_msg()
            pt_local.pose.position.x = body_x
            pt_local.pose.position.y = body_y
            pt_local.pose.position.z = 0.0
            # if(len(self.sliced_race_line_body.poses) != len(self.sliced_raceline.poses)):
            #     print(len(self.sliced_race_line_body.poses))
            #     print(len(self.sliced_raceline.poses))
            #     print(global_pose_idx_r)
            #     # assert false
            self.sliced_race_line_body.poses[global_pose_idx_r] = pt_local

            grid_x_idx = int(self.CENTER_X_GRID - int(body_x/(self.UNIT_GRID_DIST/self.PRESET_X_GRID_SIZE)))
            grid_y_idx = int(self.CENTER_Y_GRID - int(body_y/(self.UNIT_GRID_DIST/self.PRESET_Y_GRID_SIZE)))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.input_visual_feature[0,0,grid_x_idx,grid_y_idx] = self.OVERLAY_BOUNDARY_VALUE

        toc = time.time()
        # print("to body : ", toc - tic)

        self.sliced_track_bound_l_body.header.stamp = self.get_clock().now().to_msg()
        self.sliced_track_bound_r_body.header.stamp = self.get_clock().now().to_msg()
        self.sliced_race_line_body.header.stamp = self.get_clock().now().to_msg()

        # print(len(self.sliced_raceline.poses))
        # print(len(self.sliced_track_bound_r.poses))
        # print(len(self.sliced_track_bound_l.poses))

        self.raceline_pub.publish(self.sliced_race_line_body)
        self.track_bound_r_pub.publish(self.sliced_track_bound_r_body)
        self.track_bound_l_pub.publish(self.sliced_track_bound_l_body)

    def timer_callback(self):
        '''
        Inference testing
        '''
        tic = time.time()
        test_odom_msg = Odometry()
        test_odom_msg.header.frame_id = "odom"
        test_odom_msg.header.stamp = self.get_clock().now().to_msg()

        test_odom_msg.pose.pose.position.x = -3.0
        test_odom_msg.pose.pose.position.y = -8.0
        test_odom_msg.pose.pose.position.z = -0.0

        self.overlay_lines(test_odom_msg)
        # past trajectory create
        player_past = np.arange(60).reshape(20,3,1)
        player_past.fill(0)

        batch = {}
        batch["player_past"] = torch.from_numpy(player_past).unsqueeze(dim=0).type(torch.FloatTensor).to(self.device)
        batch["visual_features"] = torch.from_numpy(self.input_visual_feature).type(torch.FloatTensor).to(self.device)

        z = self.model._params(
                visual_features=batch["visual_features"], player_past=batch["player_past"]
            ).repeat((self.arr.shape[0], 1))

        traj, prb = self.model.trajectory_library_plan(self.traj_lib, z, costmap=None, phi=1, costmap_only=False)

        toc = time.time()
        # print(toc-tic)

        traj_lib_np = self.traj_lib.detach().cpu().numpy().astype(np.float64)
        prb_np = prb.detach().cpu().numpy().astype(np.float64)
        traj_np = traj.detach().cpu().numpy().astype(np.float64)

        vis_path = Path()
        vis_path.header.frame_id = "odom"
        for i in range(traj_np.shape[0]):
            pt = PoseStamped()
            pt.header.frame_id = "odom"
            pt.pose.position.x = traj_np[i,0]
            pt.pose.position.y = traj_np[i,1]
            vis_path.poses.append(pt)
        self.path_pub.publish(vis_path)

    def opponent_1_callback(self,msg):
        # parse msg
        self.opponent_1_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_1_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_1_odom_buffer.append(msg)
            self.opponent_1_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_1_odom_buffer) > self.oppo_buffer_length):
                self.opponent_1_odom_buffer = self.opponent_1_odom_buffer[-self.oppo_buffer_length]
                self.opponent_1_odom_buffer_global_np = self.opponent_1_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_1_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_1_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_1_odom_buffer_img_grid_np = np.append(self.opponent_1_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_2_callback(self,msg):
        # parse msg
        self.opponent_2_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_2_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_2_odom_buffer.append(msg)
            self.opponent_2_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_2_odom_buffer) > self.oppo_buffer_length):
                self.opponent_2_odom_buffer = self.opponent_2_odom_buffer[-self.oppo_buffer_length]
                self.opponent_2_odom_buffer_global_np = self.opponent_2_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_2_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_2_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_2_odom_buffer_img_grid_np = np.append(self.opponent_2_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_3_callback(self,msg):
        # parse msg     
        self.opponent_3_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_3_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_3_odom_buffer.append(msg)
            self.opponent_3_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_3_odom_buffer) > self.oppo_buffer_length):
                self.opponent_3_odom_buffer = self.opponent_3_odom_buffer[-self.oppo_buffer_length]
                self.opponent_3_odom_buffer_global_np = self.opponent_3_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_3_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_3_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_3_odom_buffer_img_grid_np = np.append(self.opponent_3_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_4_callback(self,msg):
        # parse msg
        self.opponent_4_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_4_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_4_odom_buffer.append(msg)
            self.opponent_4_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_4_odom_buffer) > self.oppo_buffer_length):
                self.opponent_4_odom_buffer = self.opponent_4_odom_buffer[-self.oppo_buffer_length]
                self.opponent_4_odom_buffer_global_np = self.opponent_4_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_4_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_4_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_4_odom_buffer_img_grid_np = np.append(self.opponent_4_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_5_callback(self,msg):
        # parse msg
        self.opponent_5_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_5_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_5_odom_buffer.append(msg)
            self.opponent_5_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_5_odom_buffer) > self.oppo_buffer_length):
                self.opponent_5_odom_buffer = self.opponent_5_odom_buffer[-self.oppo_buffer_length]
                self.opponent_5_odom_buffer_global_np = self.opponent_5_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_5_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_5_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_5_odom_buffer_img_grid_np = np.append(self.opponent_5_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_6_callback(self,msg):
        # parse msg
        self.opponent_6_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_6_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_6_odom_buffer.append(msg)
            self.opponent_6_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_6_odom_buffer) > self.oppo_buffer_length):
                self.opponent_6_odom_buffer = self.opponent_6_odom_buffer[-self.oppo_buffer_length]
                self.opponent_6_odom_buffer_global_np = self.opponent_6_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_6_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_6_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_6_odom_buffer_img_grid_np = np.append(self.opponent_6_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_7_callback(self,msg):
        # parse msg
        self.opponent_7_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_7_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_7_odom_buffer.append(msg)
            self.opponent_7_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_7_odom_buffer) > self.oppo_buffer_length):
                self.opponent_7_odom_buffer = self.opponent_7_odom_buffer[-self.oppo_buffer_length]
                self.opponent_7_odom_buffer_global_np = self.opponent_7_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_7_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_7_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_7_odom_buffer_img_grid_np = np.append(self.opponent_7_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_8_callback(self,msg):
        # parse msg
        self.opponent_8_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_8_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_8_odom_buffer.append(msg)
            self.opponent_8_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_8_odom_buffer) > self.oppo_buffer_length):
                self.opponent_8_odom_buffer = self.opponent_8_odom_buffer[-self.oppo_buffer_length]
                self.opponent_8_odom_buffer_global_np = self.opponent_8_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_8_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_8_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_8_odom_buffer_img_grid_np = np.append(self.opponent_8_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_9_callback(self,msg):
        # parse msg
        self.opponent_9_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_9_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_9_odom_buffer.append(msg)
            self.opponent_9_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_9_odom_buffer) > self.oppo_buffer_length):
                self.opponent_9_odom_buffer = self.opponent_9_odom_buffer[-self.oppo_buffer_length]
                self.opponent_9_odom_buffer_global_np = self.opponent_9_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_9_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_9_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_9_odom_buffer_img_grid_np = np.append(self.opponent_9_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    def opponent_10_callback(self,msg):
        # parse msg
        self.opponent_10_last_update_time = self.get_clock().now() 
        if(abs(self.opponent_10_last_update_time - self.get_clock().now().sec) > 0.1):
            self.opponent_1_odom_buffer.append(msg)
            self.opponent_10_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_1_odom_buffer) > self.oppo_buffer_length):
                self.opponent_1_odom_buffer = self.opponent_1_odom_buffer[-self.oppo_buffer_length]
                self.opponent_10_odom_buffer_global_np = self.opponent_10_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_10_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_10_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_10_odom_buffer_img_grid_np = np.append(self.opponent_10_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    # def ego_odom_callback(self,msg):
    #     print("Odom callback")
    #     self.cur_odom = msg
    #     self.odom_r = R.from_quat(np.array([msg.pose.pose.orientation.x,
    #                                         msg.pose.pose.orientation.y,
    #                                         msg.pose.pose.orientation.z,
    #                                         msg.pose.pose.orientation.w]))
    #     odom_time = msg.header.stamp.sec
    #     time_now = rclpy.clock.Clock().now()

    #     if(abs(self.last_odom_update - time_now) > 0.1):
    #         # add odom msg to buffer
    #         self.odom_buffer.append(msg)
    #         self.last_odom_update = odom_time

    #         if len(self.odom_buffer) > self.odom_buffer_length:
    #             self.odom_buffer = self.odom_buffer[-self.odom_buffer_length:]

    #     if(len(self.odom_buffer) >= self.odom_buffer_length):
    #         '''
    #         Gen input 1: past trajectory
    #         '''
    #         t_odom = np.array([o.header.stamp.sec for o in self.odom_buffer])
    #         t_odom = t_odom - np.min(t_odom)
    #         sort_idx = np.argsort(t_odom)
    #         t_odom = t_odom[sort_idx]
    #         t_odom, unique_idx = np.unique(t_odom, return_index=True)
    #         sort_idx = sort_idx[unique_idx]

    #         odom_buffer = [self.odom_buffer[i] for i in sort_idx]
    #         odom_pos = np.array(
    #             [
    #                 [
    #                     o.pose.pose.position.x,
    #                     o.pose.pose.position.y,
    #                     o.pose.pose.position.z,
    #                 ]
    #                 for o in odom_buffer
    #             ]
    #         )
    #         odom_quat = np.array(
    #             [
    #                 [
    #                     o.pose.pose.orientation.x,
    #                     o.pose.pose.orientation.y,
    #                     o.pose.pose.orientation.z,
    #                     o.pose.pose.orientation.w,
    #                 ]
    #                 for o in odom_buffer
    #             ]
    #         )
    #         interpolated_t_odom = np.arange(np.min(t_odom), np.max(t_odom), self.dt)
    #         interpolated_odom_pos = np.zeros((interpolated_t_odom.shape[0], 3))
    #         interp_obj_x = interpolate.interp1d(t_odom, odom_pos[:, 0], kind="linear")
    #         interpolated_odom_pos[:, 0] = interp_obj_x(interpolated_t_odom)
    #         interp_obj_y = interpolate.interp1d(t_odom, odom_pos[:, 1], kind="linear")
    #         interpolated_odom_pos[:, 1] = interp_obj_y(interpolated_t_odom)
    #         interp_obj_z = interpolate.interp1d(t_odom, odom_pos[:, 2], kind="linear")
    #         interpolated_odom_pos[:, 2] = interp_obj_z(interpolated_t_odom)

    #         # interpolate rotations (using SLERP)
    #         odom_r = R.from_quat(odom_quat)
    #         interp_rots = Slerp(t_odom, odom_r)
    #         interpolated_odom_r = interp_rots(interpolated_t_odom)

    #         # transform to baselink
    #         local_odom = interpolated_odom_pos[-self.past_lenth:, :]
    #         local_odom_ego = (
    #             interpolated_odom_r[-1]
    #             .inv()
    #             .apply(local_odom - interpolated_odom_pos[-1, :])
    #         )
    #         if self.flip_y_axis:
    #             local_odom_ego[:, 1] *= -1.0
    #         # if self.forward_only and np.any(local_odom_ego[:, 0] < 0):
    #         #     rospy.logerr_throttle(1, "Skipping because path is moving backwards!")
    #         #     return

    #         past_traj_mat = local_odom_ego

    #         # Overlay opponents' traj
    #         oppos_traj = np.concatenate((self.opponent_1_odom_buffer_img_grid_np, 
    #                                     self.opponent_2_odom_buffer_img_grid_np,
    #                                     self.opponent_3_odom_buffer_img_grid_np,
    #                                     self.opponent_4_odom_buffer_img_grid_np,
    #                                     self.opponent_5_odom_buffer_img_grid_np,
    #                                     self.opponent_6_odom_buffer_img_grid_np,
    #                                     self.opponent_7_odom_buffer_img_grid_np,
    #                                     self.opponent_8_odom_buffer_img_grid_np,
    #                                     self.opponent_9_odom_buffer_img_grid_np,
    #                                     self.opponent_10_odom_buffer_img_grid_np,
    #                                     ), axis=0)

    #         for i in range(len(oppos_traj)):
    #             grid_x_idx = oppos_traj[i][0]
    #             grid_y_idx = oppos_traj[i][1]
    #             self.input_visual_feature[0, grid_x_idx, grid_y_idx] = 255

    #         '''
    #         Gen input 2: static information
    #         '''
    #         for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
    #                                     - self.track_bound_l_global))):
    #             grid_x_idx = int(self.CEplayer_pastNTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
    #             grid_y_idx = int(self.CENTER_y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))
                
    #             if (grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
    #                 and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE):
    #                 self.input_visual_feature[0, grid_x_idx, grid_y_idx] = 50
                    
    #         for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
    #                                     - self.track_bound_r_global))):
    #             grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
    #             grid_y_idx = int(self.CENTER_y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))
                
    #             if (grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
    #                 and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE):
    #                 self.input_visual_feature[0, grid_x_idx, grid_y_idx] = 50

    #         for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
    #                                     - self.raceline_global))):
    #             grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
    #             grid_y_idx = int(self.CENTER_y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))
                
    #             if (grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
    #                 and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE):
    #                 self.input_visual_feature[0, grid_x_idx, grid_y_idx] = 125

    #         '''
    #         visualization
    #         '''
    #         self.vis_past_traj = Path()
    #         self.vis_past_traj.header.frame_id = "base_link"
    #         self.vis_past_traj.header.stamp = time_now
    #         self.vis_output_traj = Path()
    #         self.vis_output_traj.header.frame_id = "base_link"
    #         self.vis_output_traj.header.stamp = time_now

    #         for i in range(self.past_lenth):
    #             pt = PoseStamped()
    #             pt.header.frame_id = "base_link"
    #             pt.header.stamp = time_now
    #             pt.pose.position.z = 0
    #             pt.pose.orientation.x = 0
    #             pt.pose.orientation.y = 0
    #             pt.pose.orientation.z = 0
    #             pt.pose.orientation.w = 1
    #             pt.pose.position.x = local_odom_ego[i, 0]
    #             pt.pose.position.y = local_odom_ego[i, 1]
    #             self.vis_past_traj.poses.append(pt)
    #         # TODO: publish

    #         batch = {}
    #         batch["player_past"] = torch.from_numpy(player_past).unsqueeze(dim=0).type(torch.FloatTensor).to(self.device)
    #         batch["visual_features"] = torch.from_numpy(self.input_visual_feature).type(torch.FloatTensor).to(self.device)

    #         z = self.model._params(
    #             visual_features=batch["visual_features"], player_past=batch["player_past"]
    #         ).repeat((self.arr.shape[0], 1))

    #         traj, prb = self.model.trajectory_library_plan(self.traj_lib, z, costmap=None, phi=1, costmap_only=False)

    def output_visualization(self,traj_cpu,prb_cpu_np):
        normed_prb = np.linalg.norm(prb_cpu_np)


    # def ego_veh_status_callback_deprecated(self,msg):
    #     self.cur_odom.header.frame_id = "odom"
    #     self.cur_odom.header.stamp = msg.header.stamp
    #     self.cur_odom.pose.pose = msg.odometry.pose
    #     self.odom_r = R.from_quat(np.array([self.cur_odom.pose.pose.orientation.x,
    #                                         self.cur_odom.pose.pose.orientation.y,
    #                                         self.cur_odom.pose.pose.orientation.z,
    #                                         self.cur_odom.pose.pose.orientation.w]))
    #     odom_time = msg.header.stamp.sec
    #     time_now = odom_time

    #     if(abs(self.last_odom_update - time_now) > 0.1):
    #         # add odom msg to buffer
    #         self.odom_buffer.append(self.cur_odom.pose.pose)
    #         self.odom_buffer_np = np.append(self.odom_buffer_np,
    #                                         np.array([[self.cur_odom.pose.pose.position.x,
    #                                                     self.cur_odom.pose.pose.position.y,
    #                                                     self.cur_odom.pose.pose.position.z]]), axis=0)
            
    #         self.last_odom_update = odom_time

    #         if len(self.odom_buffer) > self.odom_buffer_length:
    #             self.odom_buffer = self.odom_buffer[-self.odom_buffer_length:]
    #             self.odom_buffer_np = np.delete(self.odom_buffer_np,0,axis=0)

    #     batch = {}
    #     if(len(self.odom_buffer)+1 > self.odom_buffer_length):
    #         '''
    #         Gen input 1: Past trajectory (to body coordinate)
    #         '''
    #         player_past = np.empty((0,3), dtype=float)
    #         for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.pose.position.x,
    #                                                                self.cur_odom.pose.pose.position.y,
    #                                                                self.cur_odom.pose.pose.position.z))
    #                                                                - self.odom_buffer_np))):
    #             player_past = np.append(player_past,np.array([[self.cur_odom.pose.pose.position.x,
    #                                                             self.cur_odom.pose.pose.position.y,
    #                                                             self.cur_odom.pose.pose.position.z]]), axis=0)
    #         '''
    #         Gen input 2: Visual feature (Gen self.input_visual_feature)
    #         '''
    #         self.overlay_lines(self.cur_odom)
    #         batch["player_past"] = torch.from_numpy(player_past).unsqueeze(dim=0).type(torch.FloatTensor).to(self.device)
    #         batch["visual_features"] = torch.from_numpy(self.input_visual_feature).type(torch.FloatTensor).to(self.device)

    #         z = self.model._params(visual_features=batch["visual_features"], 
    #                                 player_past=batch["player_past"]).repeat((self.arr.shape[0], 1))

    #         traj, prb = self.model.trajectory_library_plan(self.traj_lib, z, costmap=None, phi=1, costmap_only=False)

    #         # GPU to CPU
    #         traj_cpu_np = traj.detach().cpu().numpy().astype(np.float64)
    #         prb_cpu_np = prb.detach().cpu().numpy().astype(np.float64)

    #         # Publish result
    #         vis_path = Path()
    #         vis_path.header.frame_id = "base_link"
    #         for i in range(traj_cpu_np.shape[0]):
    #             pt = PoseStamped()
    #             pt.header.frame_id = "base_link"
    #             pt.pose.position.x = traj_cpu_np[i,0]
    #             pt.pose.position.y = traj_cpu_np[i,1]
    #             vis_path.poses.append(pt)
    #         self.path_pub.publish(vis_path)

    #     else:
    #         # wait for the next callback
    #         return -1

    def ego_veh_status_callback(self,msg):
        self.cur_odom.header.frame_id = "odom"
        self.cur_odom.header.stamp = msg.header.stamp
        self.cur_odom.pose.pose = msg.odometry.pose
        odom_time = self.get_clock().now()

        ego_roll, ego_pitch, ego_yaw = self.euler_from_quaternion(self.cur_odom.pose.pose.orientation.x,
                                                             self.cur_odom.pose.pose.orientation.y,
                                                             self.cur_odom.pose.pose.orientation.z,
                                                             self.cur_odom.pose.pose.orientation.w)

        if((odom_time.nanoseconds - self.last_odom_update.nanoseconds) > 0.1 * 10**9):
            # add odom msg to buffer
            self.odom_buffer.append(self.cur_odom.pose.pose)
            self.odom_buffer_np = np.append(self.odom_buffer_np,
                                            np.array([[self.cur_odom.pose.pose.position.x,
                                                        self.cur_odom.pose.pose.position.y,
                                                        self.cur_odom.pose.pose.position.z]]), axis=0)

            self.last_odom_update = odom_time

            if len(self.odom_buffer) > self.odom_buffer_length:
                self.odom_buffer = self.odom_buffer[-self.odom_buffer_length:]
                self.odom_buffer_np = np.delete(self.odom_buffer_np,0,axis=0)

        batch = {}
        if(len(self.odom_buffer)+1 > self.odom_buffer_length):
            '''
            Gen input 1: Past trajectory (to body coordinate)
            '''
            player_past = np.empty((0,3), dtype=float)
            self.past_traj_path_body.poses = []
            self.past_traj_path_body.header.frame_id = "base_link"

            for global_pose in (self.odom_buffer):

                body_x,body_y,_ = self.goal_pt_to_body(self.cur_odom.pose.pose.position.x,self.cur_odom.pose.pose.position.y,ego_yaw,
                                                    global_pose.position.x, global_pose.position.y, 0.0)
                pt_local = PoseStamped()
                pt_local.header.frame_id = "base_link"
                pt_local.header.stamp = self.get_clock().now().to_msg()
                pt_local.pose.position.x = body_x
                pt_local.pose.position.y = body_y
                pt_local.pose.position.z = 0.0
                self.past_traj_path_body.poses.append(pt_local)

                player_past = np.insert(player_past,0,np.array([[body_x,body_y,0.0]]), axis=0)
            
            self.player_past_pub.publish(self.past_traj_path_body)
            
            '''
            Gen input 2: Visual feature (Gen self.input_visual_feature)
            '''
            tic = time.time()

            self.overlay_lines(self.cur_odom)
            
            batch["player_past"] = torch.from_numpy(player_past).unsqueeze(dim=0).type(torch.FloatTensor).to(self.device)
            batch["visual_features"] = torch.from_numpy(self.input_visual_feature).type(torch.FloatTensor).to(self.device)

            num_samps = 1
            z = self.model._params(visual_features=batch["visual_features"], 
                                    player_past=batch["player_past"]).repeat((self.arr.shape[0], 1))
            z = z.repeat((num_samps, 1))
            samples = self.model._decoder(z).reshape(50, num_samps, 
                                                    self.output_shape[0],
                                                    self.output_shape[1])
            

            traj, prb = self.model.trajectory_library_plan(self.traj_lib, z, costmap=None, phi=1, costmap_only=False)

            toc = time.time()
            # print("infer : ", toc - tic)

            tic = time.time()
            # GPU to CPU
            traj_cpu_np = traj.detach().cpu().numpy().astype(np.float64)
            prb_cpu_np = prb.detach().cpu().numpy().astype(np.float64)
            predictions = samples.detach().cpu().numpy()

            pb_arr = Float64MultiArray()
            for i in range(prb_cpu_np.shape[0]):
                pb_arr.data.append(prb_cpu_np[i])
            self.prob_arr_pub.publish(pb_arr)


            toc = time.time()
            # Publish result
            # traj_lib_path = Path()
            # traj_lib_path.header.frame_id = "base_link"
            # for i in range(self.trajs_candidates.shape[0]):
            #     for j in range(self.trajs_candidates.shape[1]):
            #         pt = PoseStamped()
            #         pt.header.frame_id = "base_link"
            #         pt.pose.position.x = self.trajs_candidates[i][j][0]
            #         pt.pose.position.y = self.trajs_candidates[i][j][1]
            #         traj_lib_path.poses.append(pt)
            # self.traj_lib_pub.publish(traj_lib_path)

            # vis_path = Path()
            # vis_path.header.frame_id = "base_link"
            # for i in range(traj_cpu_np.shape[0]):
            #     pt = PoseStamped()
            #     pt.header.frame_id = "base_link"
            #     pt.pose.position.x = traj_cpu_np[i,0]
            #     pt.pose.position.y = traj_cpu_np[i,1]
            #     vis_path.poses.append(pt)
            # self.path_pub.publish(vis_path)

            vis_path = Path()
            vis_path.header.frame_id = "base_link"
            for i in range(predictions[prb_cpu_np.argmax()].shape[1]):
                pt = PoseStamped()
                pt.header.frame_id = "base_link"
                pt.pose.position.x = predictions[prb_cpu_np.argmax()][0][i][0].astype(float)
                pt.pose.position.y = predictions[prb_cpu_np.argmax()][0][i][1].astype(float)
                vis_path.poses.append(pt)
            self.path_pub.publish(vis_path)

        else:
            # wait for the next callback
            return -1
                



def main(args=None):
    rclpy.init(args=args)

    planning_node = ImitativePlanningNode()

    try:
        rclpy.spin(planning_node)
    finally:
        planning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
