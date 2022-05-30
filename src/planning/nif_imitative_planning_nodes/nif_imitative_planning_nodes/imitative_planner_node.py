"""Copyright (c) 2019-2020, NVIDIA CORPORATION. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE."""

from ament_index_python import get_package_share_directory
import math
import time
from nif_msgs.msg import ACTelemetryCarStatus
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R
from scipy import interpolate, spatial
from nav_msgs.msg import Path
from collections import defaultdict
import pandas as pd
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
from visualization_msgs.msg import Marker, MarkerArray
import matplotlib.pyplot as plt
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import UInt8, Float32MultiArray
import dill

from scipy.interpolate import interp1d

home_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "."))
sys.path.append(home_dir)
home_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "ac_track_db"))
sys.path.append(home_dir)

from oatomobile.baselines.torch.dim.model_ac_traj import (
    ImitativeModel,
    ImitativeModel_slim,
)
from threading import Thread
import threading

# from oatomobile.baselines.torch.dim.model_ac import ImitativeModel


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


track_db_path = get_share_file(
    "nif_imitative_planning_nodes", "nif_imitative_planning_nodes/ac_track_db"
)
model_weight_db_path = get_share_file(
    "nif_imitative_planning_nodes", "nif_imitative_planning_nodes/ac_weight_files"
)
traj_lib_db_path = get_share_file(
    "nif_imitative_planning_nodes", "nif_imitative_planning_nodes/ac_trajectory_lib"
)


class ImitativePlanningNode(Node):
    def __init__(self):
        super().__init__("imitative_planning_node")

        self.verbose = True
        self.dt = 0.01  # [sec]
        self.use_traj_lib = True
        self.num_samples = 1
        self.vis_density_function = False
        self.cnt = 0
        """
        Testing purpose
        """
        self.cnt = 0
        self.highest_imitation_prior_path_idx_pub = self.create_publisher(
            UInt8, "imitative/highest_prior_idx", rclpy.qos.qos_profile_sensor_data
        )
        self.highest_imitation_prior_pub = self.create_publisher(
            Float32MultiArray, "imitative/highest_prior_array", rclpy.qos.qos_profile_sensor_data
        )
        self.path_pub = self.create_publisher(
            Path, "imitative/out_path", rclpy.qos.qos_profile_sensor_data
        )
        self.samples_pub = self.create_publisher(
            MarkerArray, "imitative/samples", rclpy.qos.qos_profile_sensor_data
        )
        self.track_bound_l_pub = self.create_publisher(
            Path,
            "imitative/input/track_bound_l_path",
            rclpy.qos.qos_profile_sensor_data,
        )
        self.track_bound_r_pub = self.create_publisher(
            Path,
            "imitative/input/track_bound_r_path",
            rclpy.qos.qos_profile_sensor_data,
        )
        self.raceline_pub = self.create_publisher(
            Path, "imitative/input/raceline_path", rclpy.qos.qos_profile_sensor_data
        )
        self.player_past_pub = self.create_publisher(
            Path, "imitative/input/past_path", rclpy.qos.qos_profile_sensor_data
        )

        self.pub_ego_marker = self.create_publisher(
            Marker, "/ac/car_marker_" + str(0), rclpy.qos.qos_profile_sensor_data
        )
        self.pub_oppo_1_marker = self.create_publisher(
            Marker, "/ac/car_marker_" + str(1), rclpy.qos.qos_profile_sensor_data
        )
        self.pub_oppo_2_marker = self.create_publisher(
            Marker, "/ac/car_marker_" + str(2), rclpy.qos.qos_profile_sensor_data
        )
        self.pub_oppo_3_marker = self.create_publisher(
            Marker, "/ac/car_marker_" + str(3), rclpy.qos.qos_profile_sensor_data
        )

        """
        Create a subscribers
        """
        self.ego_odom_subscription = self.create_subscription(
            ACTelemetryCarStatus,
            "/ac/car_status_0",
            self.ego_veh_status_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.opponent_1_subscription = self.create_subscription(
            ACTelemetryCarStatus,
            "/ac/car_status_1",
            self.opponent_1_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.opponent_2_subscription = self.create_subscription(
            ACTelemetryCarStatus,
            "/ac/car_status_2",
            self.opponent_2_callback,
            rclpy.qos.qos_profile_sensor_data,
        )
        self.opponent_3_subscription = self.create_subscription(
            ACTelemetryCarStatus,
            "/ac/car_status_3",
            self.opponent_3_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.frenet_path_candidates_subscription = self.create_subscription(
            MarkerArray,
            "/connecting_path_list",
            self.frenet_path_candidates_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self.frenet_path_candidates_npy = None
        self.frenet_path_candidates_tensor = None
        self.dynamic_traj_lib_valid_flg = False

        self.opponent_1_odom_buffer = []
        self.opponent_2_odom_buffer = []
        self.opponent_3_odom_buffer = []

        self.opponent_1_past_traj_body_buffer = []
        self.opponent_2_past_traj_body_buffer = []
        self.opponent_3_past_traj_body_buffer = []
        self.player_past = None

        """
        Marker init
        """
        self.ego_marker = Marker()
        self.ego_marker.header.frame_id = "odom"
        self.ego_marker.id = 0
        self.ego_marker.type = 10
        self.ego_marker.mesh_resource = "package://il15_description/visual/il15.dae"
        self.ego_marker.action = self.ego_marker.ADD
        self.ego_marker.pose.position.x = 0.0
        self.ego_marker.pose.position.y = 0.0
        self.ego_marker.pose.position.z = 0.0
        self.ego_marker.pose.orientation.x = 0.0
        self.ego_marker.pose.orientation.y = 0.0
        self.ego_marker.pose.orientation.z = 0.0
        self.ego_marker.pose.orientation.w = 1.0
        self.ego_marker.lifetime = Duration(seconds=1, nanoseconds=20000000).to_msg()
        self.ego_marker.scale.x = 1.0
        self.ego_marker.scale.y = 1.0
        self.ego_marker.scale.z = 1.0
        self.ego_marker.color.a = 1.0

        self.oppo_1_marker = self.ego_marker
        self.oppo_2_marker = self.ego_marker
        self.oppo_3_marker = self.ego_marker

        self.ego_marker.color.r = 0.4
        self.ego_marker.color.g = 0.65
        self.ego_marker.color.b = 0.729

        self.oppo_1_marker.color.r = 1.0
        self.oppo_1_marker.color.g = 0.0
        self.oppo_1_marker.color.b = 0.729
        self.oppo_2_marker.color.r = 1.0
        self.oppo_2_marker.color.g = 0.0
        self.oppo_2_marker.color.b = 0.729
        self.oppo_3_marker.color.r = 1.0
        self.oppo_3_marker.color.g = 0.0
        self.oppo_3_marker.color.b = 0.729

        self.opponent_1_odom_buffer_global_np = np.empty((0, 3))
        self.opponent_2_odom_buffer_global_np = np.empty((0, 3))
        self.opponent_3_odom_buffer_global_np = np.empty((0, 3))

        self.opponent_1_odom_buffer_global_list = []
        self.opponent_2_odom_buffer_global_list = []
        self.opponent_3_odom_buffer_global_list = []

        self.opponent_1_odom_buffer_img_grid_np = np.empty((0, 2))
        self.opponent_2_odom_buffer_img_grid_np = np.empty((0, 2))
        self.opponent_3_odom_buffer_img_grid_np = np.empty((0, 2))

        self.cur_odom = Odometry()
        self.ego_yaw = None
        self.odom_buffer = []
        self.odom_buffer_np = np.empty((0, 3), dtype=float)
        self.last_odom_update = self.get_clock().now()
        self.past_traj_path_body = Path()
        self.past_traj_path_body.header.frame_id = "base_link"

        """
        Load static information
        """
        inner_bound_file_path = track_db_path + "/LVMS/lvms_inner_line.csv"
        outer_bound_file_path = track_db_path + "/LVMS/lvms_outer_line.csv"
        raceline_file_path = track_db_path + "/LVMS/race_line_w_field.csv"
        left_center_file_path = track_db_path + "/LVMS/left_center_line_w_field.csv"
        right_center_file_path = track_db_path + "/LVMS/right_center_line_w_field.csv"

        self.inner_bound_data = defaultdict(dict)
        self.outer_bound_data = defaultdict(dict)
        self.race_line_data = defaultdict(dict)
        self.left_center_data = defaultdict(dict)
        self.right_center_data = defaultdict(dict)

        self.inner_bound_data = pd.read_csv(inner_bound_file_path)
        self.outer_bound_data = pd.read_csv(outer_bound_file_path)
        self.race_line_data = pd.read_csv(raceline_file_path)
        self.left_center_data = pd.read_csv(left_center_file_path)
        self.right_center_data = pd.read_csv(right_center_file_path)

        """
        Load racing line
        """
        self.raceline_field_name_x = " x_m"
        self.raceline_field_name_y = " y_m"
        self.race_line_data[self.raceline_field_name_x] = pd.DataFrame(
            self.race_line_data, columns=[self.raceline_field_name_x]
        ).values
        self.race_line_data[self.raceline_field_name_y] = pd.DataFrame(
            self.race_line_data, columns=[self.raceline_field_name_y]
        ).values
        self.raceline_global = np.array(
            np.column_stack(
                (
                    self.race_line_data[self.raceline_field_name_x],
                    self.race_line_data[self.raceline_field_name_y],
                    [0] * len(self.race_line_data[self.raceline_field_name_y]),
                )
            )
        )

        """
        Load biased centerlines
        """
        self.biased_line_field_name_x = "interpolated_x"
        self.biased_line_field_name_y = "interpolated_y"
        self.left_center_data[self.biased_line_field_name_x] = pd.DataFrame(
            self.left_center_data, columns=[self.biased_line_field_name_x]
        ).values
        self.left_center_data[self.biased_line_field_name_y] = pd.DataFrame(
            self.left_center_data, columns=[self.biased_line_field_name_y]
        ).values
        self.left_center_global = np.array(
            np.column_stack(
                (
                    self.left_center_data[self.biased_line_field_name_x],
                    self.left_center_data[self.biased_line_field_name_y],
                    [0] * len(self.left_center_data[self.biased_line_field_name_y]),
                )
            )
        )

        self.right_center_data[self.biased_line_field_name_x] = pd.DataFrame(
            self.right_center_data, columns=[self.biased_line_field_name_x]
        ).values
        self.right_center_data[self.biased_line_field_name_y] = pd.DataFrame(
            self.right_center_data, columns=[self.biased_line_field_name_y]
        ).values
        self.right_center_global = np.array(
            np.column_stack(
                (
                    self.right_center_data[self.biased_line_field_name_x],
                    self.right_center_data[self.biased_line_field_name_y],
                    [0] * len(self.right_center_data[self.biased_line_field_name_y]),
                )
            )
        )

        """
        Load track boundary lines
        """
        self.bound_field_name_x = "interpolated_x"
        self.bound_field_name_y = "interpolated_y"
        self.bound_field_name_z = "interpolated_z"

        self.inner_bound_data[self.bound_field_name_x] = pd.DataFrame(
            self.inner_bound_data, columns=[self.bound_field_name_x]
        ).values
        self.inner_bound_data[self.bound_field_name_y] = pd.DataFrame(
            self.inner_bound_data, columns=[self.bound_field_name_y]
        ).values
        self.inner_bound_data[self.bound_field_name_z] = pd.DataFrame(
            self.inner_bound_data, columns=[self.bound_field_name_z]
        ).values
        self.outer_bound_data[self.bound_field_name_x] = pd.DataFrame(
            self.outer_bound_data, columns=[self.bound_field_name_x]
        ).values
        self.outer_bound_data[self.bound_field_name_y] = pd.DataFrame(
            self.outer_bound_data, columns=[self.bound_field_name_y]
        ).values
        self.outer_bound_data[self.bound_field_name_z] = pd.DataFrame(
            self.outer_bound_data, columns=[self.bound_field_name_z]
        ).values

        self.track_bound_l_global = np.column_stack(
            [
                self.inner_bound_data[self.bound_field_name_x],
                self.inner_bound_data[self.bound_field_name_y],
                [0] * len(self.inner_bound_data[self.bound_field_name_y]),
            ]
        )
        self.track_bound_r_global = np.column_stack(
            [
                self.outer_bound_data[self.bound_field_name_x],
                self.outer_bound_data[self.bound_field_name_y],
                [0] * len(self.outer_bound_data[self.bound_field_name_y]),
            ]
        )

        """
        Load race line and track boundaries as trees
        """
        self.raceline_tree = spatial.KDTree(self.raceline_global)
        self.track_bound_l_tree = spatial.KDTree(self.track_bound_l_global)
        self.track_bound_r_tree = spatial.KDTree(self.track_bound_r_global)
        self.left_centerline_tree = spatial.KDTree(self.left_center_global)
        self.right_centerline_tree = spatial.KDTree(self.right_center_global)

        self.racenline_idx = None
        self.track_bound_l_idx = None
        self.track_bound_r_idx = None
        self.left_centerline_idx = None
        self.right_centerline_idx = None

        """
        Preparing as paths for visualization
        """
        self.track_bound_l_path_global = Path()
        self.track_bound_r_path_global = Path()
        self.race_line_path_global = Path()
        self.left_centerline_path_global = Path()
        self.right_centerline_path_global = Path()

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

        self.left_centerline_path_global.header.frame_id = "odom"
        for i in range(self.left_center_global.shape[0]):
            pt = PoseStamped()
            pt.header.frame_id = "odom"
            pt.pose.position.x = self.left_center_global[i][0]
            pt.pose.position.y = self.left_center_global[i][1]
            pt.pose.position.z = 0.0
            self.left_centerline_path_global.poses.append(pt)
        self.left_centerline_path_global_len = len(
            self.left_centerline_path_global.poses
        )

        self.right_centerline_path_global.header.frame_id = "odom"
        for i in range(self.right_center_global.shape[0]):
            pt = PoseStamped()
            pt.header.frame_id = "odom"
            pt.pose.position.x = self.right_center_global[i][0]
            pt.pose.position.y = self.right_center_global[i][1]
            pt.pose.position.z = 0.0
            self.right_centerline_path_global.poses.append(pt)
        self.right_centerline_path_global_len = len(
            self.right_centerline_path_global.poses
        )

        """
        inference configuration
        """
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if self.verbose:
            print("Device : ", self.device)

        self.ego_keep_past_traj_time = 1.0
        self.ego_planning_traj_time = 1.0
        self.oppo_keep_past_traj_time = 1.0

        self.ego_past_buffer_length = int(self.ego_keep_past_traj_time / self.dt)
        self.ego_planning_length = int(self.ego_planning_traj_time / self.dt)
        self.oppo_past_buffer_length = int(self.oppo_keep_past_traj_time / self.dt)

        self.UNIT_GRID_DIST = 0.5

        self.use_depth = True
        self.use_lidar = False
        self.use_rgb = False

        self.grid_size = 200
        self.PRESET_X_GRID_SIZE = self.grid_size
        self.PRESET_Y_GRID_SIZE = self.grid_size
        self.CENTER_X_GRID = self.PRESET_X_GRID_SIZE / 2
        self.CENTER_Y_GRID = self.PRESET_Y_GRID_SIZE / 2
        self.CHANNLE = 1

        # TODO: update
        self.OVERLAY_BOUNDARY_VALUE = 50
        self.OVERLAY_RACELINE_VALUE = 100
        self.OVERLAY_OPPONENT_VALUE = 255

        self.input_visual_feature = np.full(
            (1, self.CHANNLE, self.PRESET_X_GRID_SIZE, self.PRESET_Y_GRID_SIZE), 0
        )

        self.input_visual_feature_test = np.full(
            (1, self.CHANNLE, self.PRESET_X_GRID_SIZE, self.PRESET_Y_GRID_SIZE), 0
        )

        """
        Minimize online for loop 
        """
        self.slice_length_full = 100

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

        self.sliced_left_centerline = Path()
        self.sliced_left_centerline.header.frame_id = "odom"
        self.sliced_left_centerline.header.stamp = self.get_clock().now().to_msg()
        self.sliced_left_centerline.poses = [ps_global] * self.slice_length_full

        self.sliced_right_centerline = Path()
        self.sliced_right_centerline.header.frame_id = "odom"
        self.sliced_right_centerline.header.stamp = self.get_clock().now().to_msg()
        self.sliced_right_centerline.poses = [ps_global] * self.slice_length_full

        # ----------------------------------------

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

        self.sliced_left_center_body = Path()
        self.sliced_left_center_body.header.frame_id = "base_link"
        self.sliced_left_center_body.header.stamp = self.get_clock().now().to_msg()
        self.sliced_left_center_body.poses = [ps_body] * self.slice_length_full

        self.sliced_right_center_body = Path()
        self.sliced_right_center_body.header.frame_id = "base_link"
        self.sliced_right_center_body.header.stamp = self.get_clock().now().to_msg()
        self.sliced_right_center_body.poses = [ps_body] * self.slice_length_full

        self.last = self.get_clock().now()

        """
        Trajectory based model setup
        """
        self.MAX_BOUNDARY_DIST = 100
        self.NUM_BOUNDARY_PT = 100
        self.MAX_RACELINE_DIST = 100
        self.NUM_RACELINE_PT = 50
        self.BOUNDARY_TRAJ_DOWNSAMPLE = 2
        self.RACELINE_TRAJ_DOWNSAMPLE = 2
        self.NUM_OPPO_PAST_TRAJ_PT = 50
        self.OPPO_TRAJ_DOWNSAMPLE = 2
        self.NUM_EGO_PAST_TRAJ_PT = 50
        self.NUM_EGO_FUTURE_TRAJ_PT = 100
        self.EGO_TRAJ_DOWNSAMPLE = 2
        self.NUM_POS_DIM = 3

        self.boundary_left_body = []
        self.boundary_right_body = []
        self.raceline_body = []
        self.left_centerline_body = []
        self.right_centerline_body = []

        self.output_shape = [
            int(self.NUM_EGO_FUTURE_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE),
            self.NUM_POS_DIM,
        ]
        self.input_shape = [
            int(self.NUM_EGO_PAST_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE),
            self.NUM_POS_DIM,
        ]

        """
        load model
        """
        # self.model_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_weight_files/model-452.pt"

        # self.model_path = (
        #     model_weight_db_path
        #     + "/traj_based/slim_w_mobilenetV3/model-490.pt"  # hidden 64
        # )

        self.model_path = model_weight_db_path + "/527/model-300.pt"  # hidden 16

        # self.model = ImitativeModel(
        #     future_traj_shape=self.output_shape,
        #     NUM_POS_DIM=self.NUM_POS_DIM,
        #     past_traj_shape=self.input_shape,
        # ).to(self.device)

        self.model = ImitativeModel_slim(
            future_traj_shape=self.output_shape,
            num_pos_dim=self.NUM_POS_DIM,
            past_traj_shape=self.input_shape,
        ).to(self.device)

        self.model.load_state_dict(
            torch.load(self.model_path, map_location=self.device)
        )

        self.model.eval()

        """
        load trajectory lib
        """
        if self.use_traj_lib == True:
            self.trajectory_lib_path = traj_lib_db_path + "/527.npy"
            self.arr = np.load(self.trajectory_lib_path)
            self.traj_lib = (
                torch.from_numpy(
                    np.reshape(
                        self.arr,
                        (self.arr.shape[0], self.output_shape[0], self.output_shape[1]),
                    )
                )
                .type(torch.FloatTensor)
                .to(self.device)
            )

            self.trajs_candidates = np.reshape(
                self.arr,
                (self.arr.shape[0], self.output_shape[0], self.output_shape[1]),
            )

        """
        Safety features
        """
        self.left_boundary_close_flg = False
        self.right_boundary_close_flg = False
        self.left_boundary_dist = 0.0
        self.right_boundary_dist = 0.0
        self.close_dist_thres = 1.0

        # Additional threading for inference
        self.imitative_planning_thread()

    def goal_pt_to_body(
        self,
        cur_odom_x,
        cur_odom_y,
        cur_odom_yaw_rad,
        global_pt_x,
        global_pt_y,
        global_pt_yaw_rad,
    ):
        body_x = math.cos(-1 * cur_odom_yaw_rad) * (
            global_pt_x - cur_odom_x
        ) - math.sin(-1 * cur_odom_yaw_rad) * (global_pt_y - cur_odom_y)
        body_y = math.sin(-1 * cur_odom_yaw_rad) * (
            global_pt_x - cur_odom_x
        ) + math.cos(-1 * cur_odom_yaw_rad) * (global_pt_y - cur_odom_y)
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

        return roll_x, pitch_y, yaw_z  # in radians

    def convert_geometry_info_to_body(self, ego_odom):
        for global_pose in self.sliced_track_bound_l.poses:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.position.x,
                global_pose.position.y,
                0.0,
            )

        for global_pose in self.sliced_track_bound_r.poses:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.position.x,
                global_pose.position.y,
                0.0,
            )

        for global_pose in self.sliced_raceline.poses:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.position.x,
                global_pose.position.y,
                0.0,
            )

    def get_geometry_info(self, ego_odom):
        # get boundaries and race line
        ego_pt = [
            ego_odom.pose.pose.position.x,
            ego_odom.pose.pose.position.y,
            ego_odom.pose.pose.position.z,
        ]

        """
        Find closest index
        """
        _, self.track_bound_l_idx = self.track_bound_l_tree.query(ego_pt)
        _, self.track_bound_r_idx = self.track_bound_r_tree.query(ego_pt)
        _, self.racenline_idx = self.raceline_tree.query(ego_pt)

        """
        Slicing track geometry information
        """
        # Left boundary
        if self.track_bound_l_idx + self.NUM_BOUNDARY_PT > len(
            self.track_bound_l_path_global.poses
        ):
            self.sliced_track_bound_l.poses[
                : len(self.track_bound_l_path_global.poses)
                - (self.track_bound_l_idx + self.NUM_BOUNDARY_PT)
                - 1
            ] = self.track_bound_l_path_global.poses[self.track_bound_l_idx :]

            self.sliced_track_bound_l.poses[
                len(self.track_bound_l_path_global.poses)
                - (self.track_bound_l_idx + self.NUM_BOUNDARY_PT) :
            ] = self.track_bound_l_path_global.poses[
                : (self.track_bound_l_idx + self.NUM_BOUNDARY_PT)
                - len(self.track_bound_l_path_global.poses)
            ]
        else:
            self.sliced_track_bound_l.poses = self.track_bound_l_path_global.poses[
                self.track_bound_l_idx : self.track_bound_l_idx + self.NUM_BOUNDARY_PT
            ]

        self.boundary_left_body.clear()

        # if len(self.sliced_track_bound_l.poses) != self.NUM_BOUNDARY_PT:
        #     print("Left")
        #     print("cur idx = ", self.track_bound_l_idx)
        #     print("len self.track_bound_l_path_global.poses = ", len(self.track_bound_l_path_global.poses))
        #     print("len  = ", len(self.sliced_track_bound_l.poses))

        # for i in range(
        #     min([len(self.track_bound_l_path_global.poses), self.NUM_BOUNDARY_PT])
        # ):
        for global_pose in self.sliced_track_bound_l.poses:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                0.0,
            )
            if len(self.boundary_left_body) == 0:
                self.left_boundary_dist = body_y
                if self.left_boundary_dist < (self.close_dist_thres):
                    # vehicle may out of the track or close to the boundary
                    self.left_boundary_close_flg = True
                else:
                    self.left_boundary_close_flg = False

            self.boundary_left_body.append([body_x, body_y, _])
        self.boundary_left_body = self.boundary_left_body[: self.NUM_BOUNDARY_PT]

        # Right boundary
        if self.track_bound_r_idx + self.NUM_BOUNDARY_PT > len(
            self.track_bound_r_path_global.poses
        ):

            self.sliced_track_bound_r.poses[
                : len(self.track_bound_r_path_global.poses)
                - (self.track_bound_r_idx + self.NUM_BOUNDARY_PT)
                - 1
            ] = self.track_bound_r_path_global.poses[self.track_bound_r_idx :]

            self.sliced_track_bound_r.poses[
                len(self.track_bound_r_path_global.poses)
                - (self.track_bound_r_idx + self.NUM_BOUNDARY_PT) :
            ] = self.track_bound_r_path_global.poses[
                : (self.track_bound_r_idx + self.NUM_BOUNDARY_PT)
                - len(self.track_bound_r_path_global.poses)
            ]
        else:
            self.sliced_track_bound_r.poses = self.track_bound_r_path_global.poses[
                self.track_bound_r_idx : self.track_bound_r_idx + self.NUM_BOUNDARY_PT
            ]

        # if len(self.sliced_track_bound_r.poses) != self.NUM_BOUNDARY_PT:
        #     print("Right")
        #     print("cur idx = ", self.track_bound_r_idx)
        #     print("len self.track_bound_l_path_global.poses = ", len(self.track_bound_r_path_global.poses))
        #     print("len  = ", len(self.sliced_track_bound_r.poses))

        self.boundary_right_body.clear()
        for global_pose in self.sliced_track_bound_r.poses:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                0.0,
            )
            if len(self.boundary_right_body) == 0:
                self.right_boundary_dist = body_y
                if self.right_boundary_dist > (-1.0 * self.close_dist_thres):
                    # vehicle may out of the track or close to the boundary
                    self.right_boundary_close_flg = True
                else:
                    self.right_boundary_close_flg = False
            self.boundary_right_body.append([body_x, body_y, _])
        self.boundary_right_body = self.boundary_right_body[: self.NUM_BOUNDARY_PT]

        # Raceline
        if self.racenline_idx + self.NUM_RACELINE_PT > len(
            self.race_line_path_global.poses
        ):

            self.sliced_raceline.poses[
                : len(self.race_line_path_global.poses)
                - (self.racenline_idx + self.NUM_RACELINE_PT)
                - 1
            ] = self.race_line_path_global.poses[self.racenline_idx :]

            self.sliced_raceline.poses[
                len(self.race_line_path_global.poses)
                - (self.racenline_idx + self.NUM_RACELINE_PT) :
            ] = self.race_line_path_global.poses[
                : (self.racenline_idx + self.NUM_RACELINE_PT)
                - len(self.race_line_path_global.poses)
            ]
        else:
            self.sliced_raceline.poses = self.race_line_path_global.poses[
                self.racenline_idx : self.racenline_idx + self.NUM_RACELINE_PT
            ]

        # if len(self.sliced_raceline.poses) != self.NUM_RACELINE_PT:
        #     print("Raceline")
        #     print("cur idx = ", self.racenline_idx)
        #     print("len self.track_bound_l_path_global.poses = ", len(self.race_line_path_global.poses))
        #     print("len  = ", len(self.sliced_raceline.poses))

        self.raceline_body.clear()
        for global_pose in self.sliced_raceline.poses:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                0.0,
            )
            self.raceline_body.append([body_x, body_y, _])
        self.raceline_body = self.raceline_body[: self.NUM_RACELINE_PT]

    def frenet_path_candidates_callback(self, msg):
        # message shaping for network inputing

        self.frenet_path_candidates_npy = np.full(
            (
                len(msg.markers),
                int(self.NUM_EGO_FUTURE_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE),
                self.NUM_POS_DIM,
            ),
            0.0,
        )  # init with zero

        for candidate_idx in range(len(msg.markers)):
            for pt_idx in range(len(msg.markers[candidate_idx].points)):

                self.frenet_path_candidates_npy[candidate_idx][pt_idx][0] = (
                    msg.markers[candidate_idx].points[pt_idx].x
                )
                self.frenet_path_candidates_npy[candidate_idx][pt_idx][1] = (
                    msg.markers[candidate_idx].points[pt_idx].y
                )
                self.frenet_path_candidates_npy[candidate_idx][pt_idx][2] = (
                    msg.markers[candidate_idx].points[pt_idx].z
                )

        self.frenet_path_candidates_tensor = (
            torch.from_numpy(self.frenet_path_candidates_npy)
            .type(torch.FloatTensor)
            .to(self.device)
        )

        self.dynamic_traj_lib_valid_flg = True

    def ego_veh_status_callback(self, msg):

        self.ego_marker.pose = msg.odometry.pose
        self.ego_marker.color.r = 0.4
        self.ego_marker.color.g = 0.65
        self.ego_marker.color.b = 0.729
        self.pub_ego_marker.publish(self.ego_marker)

        self.cur_odom.header.frame_id = "odom"
        self.cur_odom.header.stamp = msg.header.stamp
        self.cur_odom.pose.pose = msg.odometry.pose

        _, _, self.ego_yaw = self.euler_from_quaternion(
            self.cur_odom.pose.pose.orientation.x,
            self.cur_odom.pose.pose.orientation.y,
            self.cur_odom.pose.pose.orientation.z,
            self.cur_odom.pose.pose.orientation.w,
        )

        """
        Append msg, assume that the data is updating at 100Hz strictly.
        """
        self.odom_buffer.append(self.cur_odom.pose.pose)

        if len(self.odom_buffer) > (self.NUM_EGO_PAST_TRAJ_PT):
            self.odom_buffer = self.odom_buffer[-self.NUM_EGO_PAST_TRAJ_PT :]

            cnt = 0
            """
            Gen input 1: Past trajectory (to body coordinate)
            """
            player_past_list = []
            self.past_traj_path_body.poses = []
            self.past_traj_path_body.header.frame_id = "base_link"

            for global_pose in self.odom_buffer:
                if cnt % self.EGO_TRAJ_DOWNSAMPLE == 0:
                    body_x, body_y, _ = self.goal_pt_to_body(
                        self.cur_odom.pose.pose.position.x,
                        self.cur_odom.pose.pose.position.y,
                        self.ego_yaw,
                        global_pose.position.x,
                        global_pose.position.y,
                        0.0,
                    )
                    pt_local = PoseStamped()
                    pt_local.header.frame_id = "base_link"
                    pt_local.header.stamp = self.get_clock().now().to_msg()
                    pt_local.pose.position.x = body_x
                    pt_local.pose.position.y = body_y
                    pt_local.pose.position.z = 0.0

                    player_past_list.append([body_x, body_y, 0.0])
                    self.past_traj_path_body.poses.append(pt_local)
                cnt += 1

            self.player_past = np.array(player_past_list)
            self.player_past_pub.publish(self.past_traj_path_body)

    def opponent_1_callback(self, msg):

        self.oppo_1_marker.pose = msg.odometry.pose
        self.oppo_1_marker.color.r = 1.0
        self.oppo_1_marker.color.g = 0.0
        self.oppo_1_marker.color.b = 0.729
        self.pub_oppo_1_marker.publish(self.oppo_1_marker)
        """
        Append msg, assume that the data is updating at 100Hz strictly.
        """
        self.opponent_1_odom_buffer.append(msg.odometry.pose)

        if len(self.opponent_1_odom_buffer) > self.NUM_OPPO_PAST_TRAJ_PT:
            self.opponent_1_odom_buffer = self.opponent_1_odom_buffer[
                -self.NUM_OPPO_PAST_TRAJ_PT :
            ]
            self.opponent_1_past_traj_body_buffer.clear()
            cnt = 0
            for oppo_1_global_pose in self.opponent_1_odom_buffer:
                if cnt % self.OPPO_TRAJ_DOWNSAMPLE == 0:
                    body_x, body_y, _ = self.goal_pt_to_body(
                        self.cur_odom.pose.pose.position.x,
                        self.cur_odom.pose.pose.position.y,
                        self.ego_yaw,
                        oppo_1_global_pose.position.x,
                        oppo_1_global_pose.position.y,
                        0.0,
                    )
                    if np.sqrt(body_x * body_x + body_y * body_y) > 200:
                        body_x = 0
                        body_y = 0
                    self.opponent_1_past_traj_body_buffer.append([body_x, body_y, 0])
                cnt += 1

    def opponent_2_callback(self, msg):
        self.oppo_2_marker.pose = msg.odometry.pose
        self.oppo_2_marker.color.r = 1.0
        self.oppo_2_marker.color.g = 0.0
        self.oppo_2_marker.color.b = 0.729
        self.pub_oppo_2_marker.publish(self.oppo_2_marker)
        """
        Append msg, assume that the data is updating at 100Hz strictly.
        """
        self.opponent_2_odom_buffer.append(msg.odometry.pose)

        if len(self.opponent_2_odom_buffer) > self.NUM_OPPO_PAST_TRAJ_PT:
            self.opponent_2_odom_buffer = self.opponent_2_odom_buffer[
                -self.NUM_OPPO_PAST_TRAJ_PT :
            ]
            self.opponent_2_past_traj_body_buffer.clear()
            cnt = 0
            for oppo_global_pose in self.opponent_2_odom_buffer:
                if cnt % self.OPPO_TRAJ_DOWNSAMPLE == 0:
                    body_x, body_y, _ = self.goal_pt_to_body(
                        self.cur_odom.pose.pose.position.x,
                        self.cur_odom.pose.pose.position.y,
                        self.ego_yaw,
                        oppo_global_pose.position.x,
                        oppo_global_pose.position.y,
                        0.0,
                    )
                    if np.sqrt(body_x * body_x + body_y * body_y) > 200:
                        body_x = 0
                        body_y = 0
                    self.opponent_2_past_traj_body_buffer.append([body_x, body_y, 0])
                cnt += 1

    def opponent_3_callback(self, msg):
        self.oppo_3_marker.pose = msg.odometry.pose
        self.oppo_3_marker.color.r = 1.0
        self.oppo_3_marker.color.g = 0.0
        self.oppo_3_marker.color.b = 0.729
        self.pub_oppo_3_marker.publish(self.oppo_3_marker)
        """
        Append msg, assume that the data is updating at 100Hz strictly.
        """
        self.opponent_3_odom_buffer.append(msg.odometry.pose)

        if len(self.opponent_3_odom_buffer) > self.NUM_OPPO_PAST_TRAJ_PT:
            self.opponent_3_odom_buffer = self.opponent_3_odom_buffer[
                -self.NUM_OPPO_PAST_TRAJ_PT :
            ]
            self.opponent_3_past_traj_body_buffer.clear()
            cnt = 0
            for oppo_global_pose in self.opponent_3_odom_buffer:
                if cnt % self.OPPO_TRAJ_DOWNSAMPLE == 0:
                    body_x, body_y, _ = self.goal_pt_to_body(
                        self.cur_odom.pose.pose.position.x,
                        self.cur_odom.pose.pose.position.y,
                        self.ego_yaw,
                        oppo_global_pose.position.x,
                        oppo_global_pose.position.y,
                        0.0,
                    )
                    if np.sqrt(body_x * body_x + body_y * body_y) > 200:
                        body_x = 0
                        body_y = 0
                    self.opponent_3_past_traj_body_buffer.append([body_x, body_y, 0])
                cnt += 1

    def output_visualization(self, traj_cpu, prb_cpu_np):
        normed_prb = np.linalg.norm(prb_cpu_np)

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.

        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)
        qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.cos(pitch / 2) * np.sin(yaw / 2)
        qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.cos(yaw / 2)
        qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
            roll / 2
        ) * np.sin(pitch / 2) * np.sin(yaw / 2)

        return [qx, qy, qz, qw]

    def imitative_planning_thread(self):

        batch = {}

        if (
            len(self.odom_buffer) >= (self.NUM_EGO_PAST_TRAJ_PT)
            and self.player_past is not None
        ):

            tic = self.get_clock().now()

            """
            Gen input 2: Visual feature (Gen self.input_visual_feature)
            """
            # Track geometry (First)
            self.get_geometry_info(self.cur_odom)

            toc = self.get_clock().now()
            
            # print("get_geometry_info : ", toc - tic)

            """
            INPUT modalities
            """
            tic = self.get_clock().now()

            batch["player_past"] = (
                torch.from_numpy(self.player_past)
                .unsqueeze(dim=0)
                .type(torch.FloatTensor)
                .to(self.device)
            )
            batch["left_bound"] = (
                torch.from_numpy(np.array(self.boundary_left_body))
                .unsqueeze(dim=0)
                .type(torch.FloatTensor)
                .to(self.device)
            )
            batch["right_bound"] = (
                torch.from_numpy(np.array(self.boundary_right_body))
                .unsqueeze(dim=0)
                .type(torch.FloatTensor)
                .to(self.device)
            )
            batch["race_line"] = (
                torch.from_numpy(np.array(self.raceline_body))
                .unsqueeze(dim=0)
                .type(torch.FloatTensor)
                .to(self.device)
            )

            if len(self.opponent_1_past_traj_body_buffer) < int(
                self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE
            ):
                self.opponent_1_past_traj_body_buffer = [[0] * 3] * int(
                    self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE
                )

            if len(self.opponent_2_past_traj_body_buffer) < int(
                self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE
            ):
                self.opponent_2_past_traj_body_buffer = [[0] * 3] * int(
                    self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE
                )

            if len(self.opponent_3_past_traj_body_buffer) < int(
                self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE
            ):
                self.opponent_3_past_traj_body_buffer = [[0] * 3] * int(
                    self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE
                )

            batch["oppo1_body"] = (
                torch.from_numpy(np.array(self.opponent_1_past_traj_body_buffer))
                .unsqueeze(dim=0)
                .type(torch.FloatTensor)
                .to(self.device)
            )
            batch["oppo2_body"] = (
                torch.from_numpy(np.array(self.opponent_2_past_traj_body_buffer))
                .unsqueeze(dim=0)
                .type(torch.FloatTensor)
                .to(self.device)
            )
            batch["oppo3_body"] = (
                torch.from_numpy(np.array(self.opponent_3_past_traj_body_buffer))
                .unsqueeze(dim=0)
                .type(torch.FloatTensor)
                .to(self.device)
            )

            toc = self.get_clock().now()
            
            # print("input model : ",toc - tic)

            self.use_traj_lib = True
            self.use_static_traj_lib = False
            if self.use_traj_lib:
                if self.use_static_traj_lib:
                    z = self.model._params(
                        ego_past=batch["player_past"],
                        raceline=batch["race_line"],
                        environmental=torch.stack(
                            [batch["left_bound"], batch["right_bound"]], dim=1
                        ),
                        oppo=torch.stack(
                            [
                                batch["oppo1_body"],
                                batch["oppo2_body"],
                                batch["oppo3_body"],
                            ],
                            dim=1,
                        ),
                    ).repeat((self.arr.shape[0], 1))

                    traj, prb = self.model.trajectory_library_plan(
                        self.traj_lib, z, costmap=None, phi=1, costmap_only=False
                    )

                    # GPU to CPU
                    traj_cpu_np = traj.detach().cpu().numpy().astype(np.float64)
                    prb_cpu_np = prb.detach().cpu().numpy().astype(np.float64)

                    # print(traj_cpu_np.shape) --> (50,3)

                    # Publish result
                    vis_path = Path()
                    vis_path.header.frame_id = "base_link"

                    for i in range(traj_cpu_np.shape[0]):
                        pt = PoseStamped()
                        pt.header.frame_id = "base_link"
                        pt.pose.position.x = traj_cpu_np[i][0]
                        pt.pose.position.y = traj_cpu_np[i][1]
                        vis_path.poses.append(pt)

                    self.path_pub.publish(vis_path)

                else:
                    if self.dynamic_traj_lib_valid_flg:
                        tic = self.get_clock().now()
            
                        # dynamic traj lib
                        z = self.model._params(
                            ego_past=batch["player_past"],
                            raceline=batch["race_line"],
                            environmental=torch.stack(
                                [batch["left_bound"], batch["right_bound"]], dim=1
                            ),
                            oppo=torch.stack(
                                [
                                    batch["oppo1_body"],
                                    batch["oppo2_body"],
                                    batch["oppo3_body"],
                                ],
                                dim=1,
                            ),
                        # ).repeat((self.frenet_path_candidates_npy.shape[0], 1))
                        ).repeat((self.frenet_path_candidates_npy.shape[0], 1))

                        traj, prb, traj_idx = self.model.trajectory_library_plan_with_idx(
                            self.frenet_path_candidates_tensor,
                            z,
                            costmap=None,
                            phi=1,
                            costmap_only=False,
                        )

                        traj_cpu_np = traj.detach().cpu().numpy().astype(np.float64)
                        traj_idx_cpu = traj_idx.detach().cpu().numpy().astype(int)
                        loss_cpu_np = prb.detach().cpu().numpy().astype(float)

                        highest_imitation_path_idx_msg = UInt8()
                        highest_imitation_path_idx_msg.data = traj_idx_cpu.item()

                        highest_imitation_path_msg = Float32MultiArray()
                        # highest_imitation_path_msg.data = loss_cpu_np.item()

                        # print((loss_cpu_np))
                        # print((loss_cpu_np[0]))
                        for loss_ in loss_cpu_np:
                            highest_imitation_path_msg.data.append(loss_)

                        self.highest_imitation_prior_path_idx_pub.publish(
                            highest_imitation_path_idx_msg
                        )
                        self.highest_imitation_prior_pub.publish(highest_imitation_path_msg)

                        # Publish result
                        vis_path = Path()
                        vis_path.header.frame_id = "base_link"

                        for i in range(traj_cpu_np.shape[0]):
                            pt = PoseStamped()
                            pt.header.frame_id = "base_link"
                            pt.pose.position.x = traj_cpu_np[i][0]
                            pt.pose.position.y = traj_cpu_np[i][1]
                            vis_path.poses.append(pt)

                        self.path_pub.publish(vis_path)
                        toc = self.get_clock().now()

            else:
                tic = self.get_clock().now()

                vis_path = Path()
                vis_path.header.frame_id = "base_link"

                z = self.model._params(
                    ego_past=batch["player_past"],
                    raceline=batch["race_line"],
                    environmental=torch.stack(
                        [batch["left_bound"], batch["right_bound"]], dim=1
                    ),
                    oppo=torch.stack(
                        [batch["oppo1_body"], batch["oppo2_body"], batch["oppo3_body"]],
                        dim=1,
                    ),
                ).repeat((self.num_samples, 1))

                # Queries model.
                plan = (
                    self.model(num_steps=3, epsilon=1.0, lr=1e-3, observation=z)
                    .detach()
                    .cpu()
                    .numpy()[0]
                )  # [T, 2]

                player_future_length = 50
                increments = player_future_length // plan.shape[0]
                time_index = list(range(0, player_future_length, increments))  # [T]
                plan_interp = interp1d(x=time_index, y=plan, axis=0)
                sample_cpu = plan_interp(np.arange(0, time_index[-1]))

                # Publish result
                vis_flg = True
                if vis_flg:

                    for i in range(sample_cpu.shape[0]):
                        pt = PoseStamped()
                        pt.header.frame_id = "base_link"
                        pt.pose.position.x = sample_cpu[i][0]
                        pt.pose.position.y = sample_cpu[i][1]

                        vis_path.poses.append(pt)

                    self.path_pub.publish(vis_path)
        threading.Timer(0.03, self.imitative_planning_thread).start()


def main(args=None):
    rclpy.init(args=args)

    planning_node = ImitativePlanningNode()

    # executor = MultiThreadedExecutor(num_threads=6)
    # executor.add_node(planning_node)

    try:
        rclpy.spin(planning_node)
        # executor.spin()
    finally:
        # executor.shutdown()
        planning_node.destroy_node()


if __name__ == "__main__":
    main()
