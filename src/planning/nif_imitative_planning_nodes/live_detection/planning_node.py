'''Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.'''


import rclpy
import torch
import numpy as np
import sys
from live_detection.live_detection_helper import DetectionNode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from oatomobile.baselines.torch.dim.model_ac import ImitativeModel
import pandas as pd

from nav_msgs.msg import Path
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from geometry_msgs.msg import PoseStamped

class ImitativePlanningNode(Node):

    def __init__(self):
        super().__init__('imitative_planning_node')

        self.verbose = True

        # Create a subscribers
        self.ego_odom_subscription = self.create_subscription(Odometry, 'image', self.ego_odom_callback, 10)
        self.opponent_1_subscription = self.create_subscription(Odometry, 'image', self.opponent_1_callback, 10) 
        self.opponent_2_subscription = self.create_subscription(Odometry, 'image', self.opponent_2_callback, 10) 
        self.opponent_3_subscription = self.create_subscription(Odometry, 'image', self.opponent_3_callback, 10) 
        self.opponent_4_subscription = self.create_subscription(Odometry, 'image', self.opponent_4_callback, 10) 
        self.opponent_5_subscription = self.create_subscription(Odometry, 'image', self.opponent_5_callback, 10) 
        self.opponent_6_subscription = self.create_subscription(Odometry, 'image', self.opponent_6_callback, 10) 
        self.opponent_7_subscription = self.create_subscription(Odometry, 'image', self.opponent_7_callback, 10) 
        self.opponent_8_subscription = self.create_subscription(Odometry, 'image', self.opponent_8_callback, 10) 
        self.opponent_9_subscription = self.create_subscription(Odometry, 'image', self.opponent_9_callback, 10) 
        self.opponent_10_subscription = self.create_subscription(Odometry, 'image', self.opponent_10_callback, 10) 

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
        
        self.odom_buffer = []
        self.odom_r = None
        self.last_odom_update = 0

        '''
        Load static information
        '''
        inner_bound_file_path = "/home/usrg/workspace/iac/AC_track_database/LVMS/linear_interpolated_trackbound/interpolated_lvms_inner_line.csv"
        outer_bound_file_path = "/home/usrg/workspace/iac/AC_track_database/LVMS/linear_interpolated_trackbound/interpolated_lvms_outer_line.csv"
        raceline_file_path = "/home/usrg/workspace/iac/AC_track_database/LVMS/raceline/raceline.csv"
        self.inner_bound_data = defaultdict(dict)
        self.outer_bound_data = defaultdict(dict)
        self.race_line_data = defaultdict(dict)
        self.inner_bound_data = pd.read_csv(inner_bound_file_path)
        self.outer_bound_data = pd.read_csv(outer_bound_file_path)
        self.race_line_data = pd.read_csv(raceline_file_path)

        raceline_field_name_x = " x_m"
        raceline_field_name_y = " y_m"
        self.raceline_data[raceline_field_name_x] = pd.DataFrame(raceline, columns=[raceline_field_name_x]).values.tolist()
        self.raceline_data[raceline_field_name_y] = pd.DataFrame(raceline, columns=[raceline_field_name_y]).multiply(-1).values.tolist()
        self.raceline_global = np.array(np.column_stack((self.raceline_data[raceline_field_name_x],
                                                    self.raceline_data[raceline_field_name_y],
                                                    [0] * len(self.raceline_data[raceline_field_name_y]))))

        bound_field_name_x = "interpolated_x"
        bound_field_name_y = "interpolated_y"
        bound_field_name_z = "interpolated_z"

        track_boud_l_data[bound_field_name_x] = pd.DataFrame(self.inner_bound_data, columns=[bound_field_name_x]).values.tolist()
        track_boud_l_data[bound_field_name_y] = pd.DataFrame(self.inner_bound_data, columns=[bound_field_name_y]).multiply(-1).values.tolist()
        track_boud_l_data[bound_field_name_z] = pd.DataFrame(self.inner_bound_data, columns=[bound_field_name_z]).values.tolist()

        track_boud_r_data[bound_field_name_x] = pd.DataFrame(r_data, columns=[bound_field_name_x]).values.tolist()
        track_boud_r_data[bound_field_name_y] = pd.DataFrame(r_data, columns=[bound_field_name_y]).multiply(-1).values.tolist()
        track_boud_r_data[bound_field_name_z] = pd.DataFrame(r_data, columns=[bound_field_name_z]).values.tolist()

        self.track_bound_l_global = np.array(np.column_stack((track_boud_l_data[bound_field_name_x],
                                                         track_boud_l_data[bound_field_name_y],
                                                         [0] * len(track_boud_l_data[bound_field_name_y]))))
        
        self.track_bound_r_global = np.array(np.column_stack((track_boud_r_data[bound_field_name_x],
                                                         track_boud_r_data[bound_field_name_y],
                                                         [0] * len(track_boud_r_data[bound_field_name_y]))))

        '''
        inference configuration
        '''
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if verbose:
            print("Device : ", self.device)
        self.odom_buffer_length = 20
        self.oppo_buffer_length = 20
        self.grid_size = 200
        self.unit_dist = 1.0
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

        self.input_2d = np.full((self.CHANNLE, self.PRESET_X_GRID_SIZE, self.PRESET_Y_GRID_SIZE), 0)
     
        '''
        load model
        '''
        self.model_path = ""
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
        self.trajectory_lib_path = "/home/calvin/Documents/racer/planner_short_hip/trajectory_lib/0413.npy"
        self.arr = np.load(self.trajectory_lib_path)
        self.traj_lib = (
            torch.from_numpy(np.reshape(self.arr, (self.arr.shape[0], self.output_shape[0], self.output_shape[1])))
            .type(torch.FloatTensor)
            .to(self.device)
        )
    
    self.opponent_1_callback(self,msg):
        # parse msg
        self.opponent_1_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_1_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_1_odom_buffer.append(msg)
            self.opponent_1_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_1_odom_buffer) > self.oppo_buffer_length):
                self.opponent_1_odom_buffer = self.opponent_1_odom_buffer[-self.oppo_buffer_length]
                self.opponent_1_odom_buffer_global_np = self.opponent_1_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_1_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_1_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_1_odom_buffer_img_grid_np = np.append(self.opponent_1_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_2_callback(self,msg):
        # parse msg
        self.opponent_2_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_2_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_2_odom_buffer.append(msg)
            self.opponent_2_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_2_odom_buffer) > self.oppo_buffer_length):
                self.opponent_2_odom_buffer = self.opponent_2_odom_buffer[-self.oppo_buffer_length]
                self.opponent_2_odom_buffer_global_np = self.opponent_2_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_2_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_2_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_2_odom_buffer_img_grid_np = np.append(self.opponent_2_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_3_callback(self,msg):
        # parse msg     
        self.opponent_3_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_3_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_3_odom_buffer.append(msg)
            self.opponent_3_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_3_odom_buffer) > self.oppo_buffer_length):
                self.opponent_3_odom_buffer = self.opponent_3_odom_buffer[-self.oppo_buffer_length]
                self.opponent_3_odom_buffer_global_np = self.opponent_3_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_3_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_3_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_3_odom_buffer_img_grid_np = np.append(self.opponent_3_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_4_callback(self,msg):
        # parse msg
        self.opponent_4_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_4_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_4_odom_buffer.append(msg)
            self.opponent_4_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_4_odom_buffer) > self.oppo_buffer_length):
                self.opponent_4_odom_buffer = self.opponent_4_odom_buffer[-self.oppo_buffer_length]
                self.opponent_4_odom_buffer_global_np = self.opponent_4_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_4_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_4_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_4_odom_buffer_img_grid_np = np.append(self.opponent_4_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_5_callback(self,msg):
        # parse msg
        self.opponent_5_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_5_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_5_odom_buffer.append(msg)
            self.opponent_5_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_5_odom_buffer) > self.oppo_buffer_length):
                self.opponent_5_odom_buffer = self.opponent_5_odom_buffer[-self.oppo_buffer_length]
                self.opponent_5_odom_buffer_global_np = self.opponent_5_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_5_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_5_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_5_odom_buffer_img_grid_np = np.append(self.opponent_5_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_6_callback(self,msg):
        # parse msg
        self.opponent_6_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_6_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_6_odom_buffer.append(msg)
            self.opponent_6_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_6_odom_buffer) > self.oppo_buffer_length):
                self.opponent_6_odom_buffer = self.opponent_6_odom_buffer[-self.oppo_buffer_length]
                self.opponent_6_odom_buffer_global_np = self.opponent_6_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_6_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_6_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_6_odom_buffer_img_grid_np = np.append(self.opponent_6_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_7_callback(self,msg):
        # parse msg
        self.opponent_7_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_7_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_7_odom_buffer.append(msg)
            self.opponent_7_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_7_odom_buffer) > self.oppo_buffer_length):
                self.opponent_7_odom_buffer = self.opponent_7_odom_buffer[-self.oppo_buffer_length]
                self.opponent_7_odom_buffer_global_np = self.opponent_7_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_7_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_7_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_7_odom_buffer_img_grid_np = np.append(self.opponent_7_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_8_callback(self,msg):
        # parse msg
        self.opponent_8_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_8_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_8_odom_buffer.append(msg)
            self.opponent_8_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_8_odom_buffer) > self.oppo_buffer_length):
                self.opponent_8_odom_buffer = self.opponent_8_odom_buffer[-self.oppo_buffer_length]
                self.opponent_8_odom_buffer_global_np = self.opponent_8_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_8_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_8_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_8_odom_buffer_img_grid_np = np.append(self.opponent_8_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_9_callback(self,msg):
        # parse msg
        self.opponent_9_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_9_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_9_odom_buffer.append(msg)
            self.opponent_9_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_9_odom_buffer) > self.oppo_buffer_length):
                self.opponent_9_odom_buffer = self.opponent_9_odom_buffer[-self.oppo_buffer_length]
                self.opponent_9_odom_buffer_global_np = self.opponent_9_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_9_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_9_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_9_odom_buffer_img_grid_np = np.append(self.opponent_9_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)

    self.opponent_10_callback(self,msg):
        # parse msg
        self.opponent_10_last_update_time = rclpy.clock.Clock.now()
        if(abs(self.opponent_10_last_update_time - rclpy.clock.Clock.now().sec) > 0.1):
            self.opponent_1_odom_buffer.append(msg)
            self.opponent_10_odom_buffer_global_np.column_stack([msg.pose.position.x,msg.pose.position.y,msg.pose.position.z])
            if(len(self.opponent_1_odom_buffer) > self.oppo_buffer_length):
                self.opponent_1_odom_buffer = self.opponent_1_odom_buffer[-self.oppo_buffer_length]
                self.opponent_10_odom_buffer_global_np = self.opponent_10_odom_buffer_global_np[1:,]
        # transform to image coordinate
        # step 1. to body coordinate space
        if self.odom_r == None:
            return 0
        for local_coor_xyz in list((odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                                - self.opponent_10_odom_buffer_global_np))):
            grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
            grid_y_idx = int(self.CENTER_Y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))

            self.opponent_10_odom_buffer_img_grid_np = np.zeros((0.2))
            if(grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE and
                and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE ):
                self.opponent_10_odom_buffer_img_grid_np = np.append(self.opponent_10_odom_buffer_img_grid_np, 
                                                                    np.array((grid_x_idx,grid_y_idx)),axis = 0)


    
    self.ego_odom_callback(self,msg):
        print("Odom callback")
        self.cur_odom = msg
        self.odom_r = R.from_quat(np.array([msg.pose.orientation.x,
                                            msg.pose.orientation.y,
                                            msg.pose.orientation.z,
                                            msg.pose.orientation.w]))
        self.last_odom_update = msg.header.stamp.sec
        time_now = rclpy.clock.Clock().now()

        if(abs(self.last_odom_update - time_now) > 0.1):
            # add odom msg to buffer
            self.odom_buffer.append(msg)

            if len(self.odom_buffer) > self.odom_buffer_length:
                self.odom_buffer = self.odom_buffer[-self.odom_buffer_length:]

        if(len(self.odom_buffer) == self.odom_buffer_length):
            '''
            Gen input 1: past trajectory
            '''
            t_odom = np.array([o.header.stamp.sec for o in self.odom_buffer])
            t_odom = t_odom - np.min(t_odom)
            sort_idx = np.argsort(t_odom)
            t_odom = t_odom[sort_idx]
            t_odom, unique_idx = np.unique(t_odom, return_index=True)
            sort_idx = sort_idx[unique_idx]

            odom_buffer = [self.odom_buffer[i] for i in sort_idx]
            odom_pos = np.array(
                [
                    [
                        o.pose.pose.position.x,
                        o.pose.pose.position.y,
                        o.pose.pose.position.z,
                    ]
                    for o in odom_buffer
                ]
            )
            odom_quat = np.array(
                [
                    [
                        o.pose.pose.orientation.x,
                        o.pose.pose.orientation.y,
                        o.pose.pose.orientation.z,
                        o.pose.pose.orientation.w,
                    ]
                    for o in odom_buffer
                ]
            )
            interpolated_t_odom = np.arange(np.min(t_odom), np.max(t_odom), self.dt)
            interpolated_odom_pos = np.zeros((interpolated_t_odom.shape[0], 3))
            interp_obj_x = interpolate.interp1d(t_odom, odom_pos[:, 0], kind="linear")
            interpolated_odom_pos[:, 0] = interp_obj_x(interpolated_t_odom)
            interp_obj_y = interpolate.interp1d(t_odom, odom_pos[:, 1], kind="linear")
            interpolated_odom_pos[:, 1] = interp_obj_y(interpolated_t_odom)
            interp_obj_z = interpolate.interp1d(t_odom, odom_pos[:, 2], kind="linear")
            interpolated_odom_pos[:, 2] = interp_obj_z(interpolated_t_odom)

            # interpolate rotations (using SLERP)
            odom_r = R.from_quat(odom_quat)
            interp_rots = Slerp(t_odom, odom_r)
            interpolated_odom_r = interp_rots(interpolated_t_odom)

            # transform to baselink
            local_odom = interpolated_odom_pos[-self.past_lenth:, :]
            local_odom_ego = (
                interpolated_odom_r[-1]
                .inv()
                .apply(local_odom - interpolated_odom_pos[-1, :])
            )
            if self.flip_y_axis:
                local_odom_ego[:, 1] *= -1.0
            if self.forward_only and np.any(local_odom_ego[:, 0] < 0):
                rospy.logerr_throttle(1, "Skipping because path is moving backwards!")
                return

            past_traj_mat = local_odom_ego

            # Overlay opponents' traj
            oppos_traj = np.concatenate((self.opponent_1_odom_buffer_img_grid_np, 
                                        self.opponent_2_odom_buffer_img_grid_np,
                                        self.opponent_3_odom_buffer_img_grid_np,
                                        self.opponent_4_odom_buffer_img_grid_np,
                                        self.opponent_5_odom_buffer_img_grid_np,
                                        self.opponent_6_odom_buffer_img_grid_np,
                                        self.opponent_7_odom_buffer_img_grid_np,
                                        self.opponent_8_odom_buffer_img_grid_np,
                                        self.opponent_9_odom_buffer_img_grid_np,
                                        self.opponent_10_odom_buffer_img_grid_np,
                                        ), axis=0)

            for i in range(len(oppos_traj)):
                grid_x_idx = oppos_traj[i][0]
                grid_y_idx = oppos_traj[i][1]
                self.input_2d[0, grid_x_idx, grid_y_idx] = 255

            '''
            Gen input 2: static information
            '''
            for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                        - self.track_bound_l_global))):
                grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
                grid_y_idx = int(self.CENTER_y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))
                
                if (grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                    and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE):
                    self.input_2d[0, grid_x_idx, grid_y_idx] = 50
                    
            for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                        - self.track_bound_r_global))):
                grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
                grid_y_idx = int(self.CENTER_y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))
                
                if (grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                    and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE):
                    self.input_2d[0, grid_x_idx, grid_y_idx] = 50

            for local_coor_xyz in list((self.odom_r.apply(np.array((self.cur_odom.pose.position.x,self.cur_odom.pose.position.y,0))
                                        - self.raceline_global))):
                grid_x_idx = int(self.CENTER_X_GRID - int(local_coor_xyz[0] / self.MODIFIED_X_GRID_DIST))
                grid_y_idx = int(self.CENTER_y_GRID - int(local_coor_xyz[1] / self.MODIFIED_Y_GRID_DIST))
                
                if (grid_x_idx > 0 and grid_x_idx < self.PRESET_X_GRID_SIZE 
                    and grid_y_idx > 0 and grid_y_idx < self.PRESET_Y_GRID_SIZE):
                    self.input_2d[0, grid_x_idx, grid_y_idx] = 125

            '''
            visualization
            '''
            self.vis_past_traj = Path()
            self.vis_past_traj.header.frame_id = "base_link"
            self.vis_past_traj.header.stamp = time_now
            self.vis_output_traj = Path()
            self.vis_output_traj.header.frame_id = "base_link"
            self.vis_output_traj.header.stamp = time_now

            for i in range(self.past_lenth):
                pt = PoseStamped()
                pt.header.frame_id = "base_link"
                pt.header.stamp = time_now
                pt.pose.position.z = 0
                pt.pose.orientation.x = 0
                pt.pose.orientation.y = 0
                pt.pose.orientation.z = 0
                pt.pose.orientation.w = 1
                pt.pose.position.x = local_odom_ego[i, 0]
                pt.pose.position.y = local_odom_ego[i, 1]
                vis_past_traj.poses.append(pt)

            # TODO: publish

            batch = {}
            batch["player_past"] = torch.from_numpy(player_past).unsqueeze(dim=0).type(torch.FloatTensor).to(device)
            batch["visual_features"] = torch.from_numpy(self.input_2d).type(torch.FloatTensor).to(device)

            z = model._params(
                visual_features=batch["visual_features"], player_past=batch["player_past"]
            ).repeat((self.arr.shape[0], 1))

            traj, prb = self.model.trajectory_library_plan(self.traj_lib, z, costmap=None, phi=1, costmap_only=False)

def main(args=None):
    rclpy.init(args=args)

    planning_node = ImitativePlanningNode()

    rclpy.spin(planning_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    planning_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
