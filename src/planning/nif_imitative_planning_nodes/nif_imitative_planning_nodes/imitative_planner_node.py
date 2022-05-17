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
import visdom
import dill

home_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "."))
sys.path.append(home_dir)
home_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "ac_track_db"))
sys.path.append(home_dir)

# from oatomobile.baselines.torch.dim.model_ac import ImitativeModel
from oatomobile.baselines.torch.dim.model_ac_traj import ImitativeModel


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


track_db_path = get_share_file("nif_imitative_planning_nodes", "ac_track_db")


class ImitativePlanningNode(Node):
    def __init__(self):
        super().__init__("imitative_planning_node")

        self.verbose = True
        self.dt = 0.01  # [sec]
        self.use_traj_lib = False
        self.num_samples = 5
        self.vis_density_function = False
        self.vis = visdom.Visdom()
        # self.vis.text("Hello wolrd", env="main")
        self.cnt = 0
        """
        Testing purpose
        """
        self.cnt = 0
        self.path_pub = self.create_publisher(
            Path, "imitative/out_path", rclpy.qos.qos_profile_sensor_data
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

        self.opponent_1_odom_buffer = []
        self.opponent_2_odom_buffer = []
        self.opponent_3_odom_buffer = []

        self.opponent_1_past_traj_body_buffer = []
        self.opponent_2_past_traj_body_buffer = []
        self.opponent_3_past_traj_body_buffer = []

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
        inner_bound_file_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/lvms_inner_line.csv"
        outer_bound_file_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/lvms_outer_line.csv"
        raceline_file_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_track_db/LVMS/race_line_w_field.csv"
        self.inner_bound_data = defaultdict(dict)
        self.outer_bound_data = defaultdict(dict)
        self.raceline_data = defaultdict(dict)
        self.inner_bound_data = pd.read_csv(inner_bound_file_path)
        self.outer_bound_data = pd.read_csv(outer_bound_file_path)
        self.race_line_data = pd.read_csv(raceline_file_path)

        """
        Load racing line
        """
        self.raceline_field_name_x = " x_m"
        self.raceline_field_name_y = " y_m"
        self.raceline_data[self.raceline_field_name_x] = pd.DataFrame(
            self.race_line_data, columns=[self.raceline_field_name_x]
        ).values
        self.raceline_data[self.raceline_field_name_y] = pd.DataFrame(
            self.race_line_data, columns=[self.raceline_field_name_y]
        ).values
        self.raceline_global = np.array(
            np.column_stack(
                (
                    self.raceline_data[self.raceline_field_name_x],
                    self.raceline_data[self.raceline_field_name_y],
                    [0] * len(self.raceline_data[self.raceline_field_name_y]),
                )
            )
        )

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

        """
        Load track boundary lines
        """
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
        self.racenline_idx = None
        self.track_bound_l_idx = None
        self.track_bound_r_idx = None

        """
        Preparing as paths for visualization
        """
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
        self.num_pos_dim = 3

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

        self.boundary_left_body = []
        self.boundary_right_body = []
        self.raceline_body = []

        self.output_shape = [
            int(self.NUM_EGO_FUTURE_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE),
            self.num_pos_dim,
        ]
        self.input_shape = [
            int(self.NUM_EGO_PAST_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE),
            self.num_pos_dim,
        ]

        """
        load model
        """
        # self.model_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_weight_files/model-452.pt"
        self.model_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_weight_files/traj_based/weighted_mle_w_slim_model/model-490.pt"
        self.model = ImitativeModel(
            future_traj_shape=self.output_shape,
            num_pos_dim=self.num_pos_dim,
            past_traj_shape=self.input_shape,
        ).to(self.device)
        self.model.load_state_dict(
            torch.load(
                self.model_path,
                map_location=self.device,
            )
        )

        self.model.eval()

        """
        load trajectory lib
        """
        if self.use_traj_lib == True:
            self.trajectory_lib_path = "/home/usrg-racing/nif/build/nif_imitative_planning_nodes/nif_imitative_planning_nodes/ac_trajectory_lib/0413.npy"
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

    def overlay_opponents(self, ego_odom):
        """
        Overlaying
        """
        downsample = 10
        for oppo_1_global_pose in self.opponent_1_odom_buffer[0::downsample]:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                oppo_1_global_pose.position.x,
                oppo_1_global_pose.position.y,
                0.0,
            )
            grid_x_idx = int(
                self.CENTER_X_GRID
                - int(body_x / (self.UNIT_GRID_DIST / self.PRESET_X_GRID_SIZE))
            )
            grid_y_idx = int(
                self.CENTER_Y_GRID
                - int(body_y / (self.UNIT_GRID_DIST / self.PRESET_Y_GRID_SIZE))
            )
            if (
                grid_x_idx > 0
                and grid_x_idx < self.PRESET_X_GRID_SIZE
                and grid_y_idx > 0
                and grid_y_idx < self.PRESET_Y_GRID_SIZE
            ):
                self.input_visual_feature[
                    0, 0, grid_x_idx, grid_y_idx
                ] = self.OVERLAY_OPPONENT_VALUE

        for oppo_2_global_pose in self.opponent_2_odom_buffer[0::downsample]:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                oppo_2_global_pose.position.x,
                oppo_2_global_pose.position.y,
                0.0,
            )
            grid_x_idx = int(
                self.CENTER_X_GRID
                - int(body_x / (self.UNIT_GRID_DIST / self.PRESET_X_GRID_SIZE))
            )
            grid_y_idx = int(
                self.CENTER_Y_GRID
                - int(body_y / (self.UNIT_GRID_DIST / self.PRESET_Y_GRID_SIZE))
            )
            if (
                grid_x_idx > 0
                and grid_x_idx < self.PRESET_X_GRID_SIZE
                and grid_y_idx > 0
                and grid_y_idx < self.PRESET_Y_GRID_SIZE
            ):
                self.input_visual_feature[
                    0, 0, grid_x_idx, grid_y_idx
                ] = self.OVERLAY_OPPONENT_VALUE

        for oppo_3_global_pose in self.opponent_3_odom_buffer[0::downsample]:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                oppo_3_global_pose.position.x,
                oppo_3_global_pose.position.y,
                0.0,
            )
            grid_x_idx = int(
                self.CENTER_X_GRID
                - int(body_x / (self.UNIT_GRID_DIST / self.PRESET_X_GRID_SIZE))
            )
            grid_y_idx = int(
                self.CENTER_Y_GRID
                - int(body_y / (self.UNIT_GRID_DIST / self.PRESET_Y_GRID_SIZE))
            )
            if (
                grid_x_idx > 0
                and grid_x_idx < self.PRESET_X_GRID_SIZE
                and grid_y_idx > 0
                and grid_y_idx < self.PRESET_Y_GRID_SIZE
            ):
                self.input_visual_feature[
                    0, 0, grid_x_idx, grid_y_idx
                ] = self.OVERLAY_OPPONENT_VALUE

    def overlay_lines(self, ego_odom):
        self.input_visual_feature.fill(0)

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
        if self.track_bound_l_idx < (self.slice_length_full / 2):
            self.sliced_track_bound_l.poses[
                0 : int(self.slice_length_full / 2 - self.track_bound_l_idx)
            ] = self.track_bound_l_path_global.poses[
                int(-(self.slice_length_full / 2 - self.track_bound_l_idx)) :
            ]
            self.sliced_track_bound_l.poses[
                int(self.slice_length_full / 2 - self.track_bound_l_idx) :
            ] = self.track_bound_l_path_global.poses[
                0 : int(self.track_bound_l_idx + (self.slice_length_full / 2))
            ]
        elif self.track_bound_l_idx > self.track_bound_l_path_global_len - (
            self.slice_length_full / 2
        ):
            self.sliced_track_bound_l.poses[
                0 : int(
                    self.track_bound_l_path_global_len
                    - self.track_bound_l_idx
                    + self.slice_length_full / 2
                )
            ] = self.track_bound_l_path_global.poses[
                int(self.track_bound_l_idx - (self.slice_length_full / 2)) :
            ]
            self.sliced_track_bound_l.poses[
                int(
                    self.track_bound_l_path_global_len
                    - self.track_bound_l_idx
                    + self.slice_length_full / 2
                ) :
            ] = self.track_bound_l_path_global.poses[
                0 : int(
                    self.track_bound_l_idx
                    + (self.slice_length_full / 2)
                    - self.track_bound_l_path_global_len
                    - 1
                )
            ]
        else:
            self.sliced_track_bound_l.poses = self.track_bound_l_path_global.poses[
                int(self.track_bound_l_idx - (self.slice_length_full / 2)) : int(
                    self.track_bound_l_idx + (self.slice_length_full / 2)
                )
            ]

        if self.track_bound_r_idx < (self.slice_length_full / 2):
            self.sliced_track_bound_r.poses[
                0 : int(self.slice_length_full / 2 - self.track_bound_r_idx)
            ] = self.track_bound_r_path_global.poses[
                int(-(self.slice_length_full / 2 - self.track_bound_r_idx)) :
            ]
            self.sliced_track_bound_r.poses[
                int(self.slice_length_full / 2 - self.track_bound_r_idx) :
            ] = self.track_bound_r_path_global.poses[
                0 : int(self.track_bound_r_idx + (self.slice_length_full / 2))
            ]
        elif self.track_bound_r_idx > self.track_bound_r_path_global_len - (
            self.slice_length_full / 2
        ):
            self.sliced_track_bound_r.poses[
                0 : int(
                    self.track_bound_r_path_global_len
                    - self.track_bound_r_idx
                    + self.slice_length_full / 2
                )
            ] = self.track_bound_r_path_global.poses[
                int(self.track_bound_r_idx - (self.slice_length_full / 2)) :
            ]
            self.sliced_track_bound_r.poses[
                int(
                    self.track_bound_r_path_global_len
                    - self.track_bound_r_idx
                    + self.slice_length_full / 2
                ) :
            ] = self.track_bound_r_path_global.poses[
                0 : int(
                    self.track_bound_r_idx
                    + (self.slice_length_full / 2)
                    - self.track_bound_r_path_global_len
                    - 1
                )
            ]
        else:
            self.sliced_track_bound_r.poses = self.track_bound_r_path_global.poses[
                int(self.track_bound_r_idx - (self.slice_length_full / 2)) : int(
                    self.track_bound_r_idx + (self.slice_length_full / 2)
                )
            ]

        if self.racenline_idx < (self.slice_length_full / 2):
            self.sliced_raceline.poses[
                0 : int(self.slice_length_full / 2 - self.racenline_idx)
            ] = self.race_line_path_global.poses[
                int(-(self.slice_length_full / 2 - self.racenline_idx)) :
            ]
            self.sliced_raceline.poses[
                int(self.slice_length_full / 2 - self.racenline_idx) :
            ] = self.race_line_path_global.poses[
                0 : int(self.racenline_idx + (self.slice_length_full / 2))
            ]
        elif self.racenline_idx > self.race_line_path_global_len - (
            self.slice_length_full / 2
        ):
            self.sliced_raceline.poses[
                0 : int(
                    self.race_line_path_global_len
                    - self.racenline_idx
                    + self.slice_length_full / 2
                )
            ] = self.race_line_path_global.poses[
                int(self.racenline_idx - (self.slice_length_full / 2)) :
            ]
            self.sliced_raceline.poses[
                int(
                    self.race_line_path_global_len
                    - self.racenline_idx
                    + self.slice_length_full / 2
                ) :
            ] = self.race_line_path_global.poses[
                0 : int(
                    self.racenline_idx
                    + (self.slice_length_full / 2)
                    - self.race_line_path_global_len
                    - 1
                )
            ]
        else:
            self.sliced_raceline.poses = self.race_line_path_global.poses[
                int(self.racenline_idx - (self.slice_length_full / 2)) : int(
                    self.racenline_idx + (self.slice_length_full / 2)
                )
            ]

        """
        Overlaying
        """
        for global_pose_idx_l in range(
            min(
                len(self.sliced_track_bound_l.poses),
                len(self.sliced_track_bound_l_body.poses),
            )
        ):
            global_pose = self.sliced_track_bound_l.poses[global_pose_idx_l]
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                0.0,
            )

            grid_x_idx = int(
                self.CENTER_X_GRID
                - int(body_x / (self.UNIT_GRID_DIST / self.PRESET_X_GRID_SIZE))
            )
            grid_y_idx = int(
                self.CENTER_Y_GRID
                - int(body_y / (self.UNIT_GRID_DIST / self.PRESET_Y_GRID_SIZE))
            )
            if (
                grid_x_idx > 0
                and grid_x_idx < self.PRESET_X_GRID_SIZE
                and grid_y_idx > 0
                and grid_y_idx < self.PRESET_Y_GRID_SIZE
            ):
                self.input_visual_feature[
                    0, 0, grid_x_idx, grid_y_idx
                ] = self.OVERLAY_BOUNDARY_VALUE

        for global_pose_idx_r in range(
            min(
                len(self.sliced_track_bound_r.poses),
                len(self.sliced_track_bound_r_body.poses),
            )
        ):
            global_pose = self.sliced_track_bound_r.poses[global_pose_idx_r]
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                0.0,
            )

            grid_x_idx = int(
                self.CENTER_X_GRID
                - int(body_x / (self.UNIT_GRID_DIST / self.PRESET_X_GRID_SIZE))
            )
            grid_y_idx = int(
                self.CENTER_Y_GRID
                - int(body_y / (self.UNIT_GRID_DIST / self.PRESET_Y_GRID_SIZE))
            )
            if (
                grid_x_idx > 0
                and grid_x_idx < self.PRESET_X_GRID_SIZE
                and grid_y_idx > 0
                and grid_y_idx < self.PRESET_Y_GRID_SIZE
            ):
                self.input_visual_feature[
                    0, 0, grid_x_idx, grid_y_idx
                ] = self.OVERLAY_BOUNDARY_VALUE

        for global_pose_idx_race in range(
            min(len(self.sliced_raceline.poses), len(self.sliced_race_line_body.poses))
        ):
            global_pose = self.sliced_raceline.poses[global_pose_idx_race]
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                0.0,
            )

            grid_x_idx = int(
                self.CENTER_X_GRID
                - int(body_x / (self.UNIT_GRID_DIST / self.PRESET_X_GRID_SIZE))
            )
            grid_y_idx = int(
                self.CENTER_Y_GRID
                - int(body_y / (self.UNIT_GRID_DIST / self.PRESET_Y_GRID_SIZE))
            )
            if (
                grid_x_idx > 0
                and grid_x_idx < self.PRESET_X_GRID_SIZE
                and grid_y_idx > 0
                and grid_y_idx < self.PRESET_Y_GRID_SIZE
            ):
                self.input_visual_feature[
                    0, 0, grid_x_idx, grid_y_idx
                ] = self.OVERLAY_RACELINE_VALUE

        """
        Publish
        """

        self.sliced_track_bound_l.header.stamp = self.get_clock().now().to_msg()
        self.sliced_track_bound_r.header.stamp = self.get_clock().now().to_msg()
        self.sliced_raceline.header.stamp = self.get_clock().now().to_msg()
        self.raceline_pub.publish(self.sliced_raceline)
        self.track_bound_l_pub.publish(self.sliced_track_bound_l)
        self.track_bound_r_pub.publish(self.sliced_track_bound_r)

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
        if self.track_bound_l_idx + self.NUM_BOUNDARY_PT > len(
            self.track_bound_l_path_global.poses
        ):
            self.sliced_track_bound_l.poses[
                self.track_bound_l_idx :
            ] = self.track_bound_l_path_global.poses[self.track_bound_l_idx :]
            self.sliced_track_bound_l.poses[
                0 : self.track_bound_l_idx
            ] = self.track_bound_l_path_global.poses[
                : (self.track_bound_l_idx + self.NUM_BOUNDARY_PT)
                - len(self.track_bound_l_path_global.poses)
            ]
        else:
            self.sliced_track_bound_l.poses = self.track_bound_l_path_global.poses[
                self.track_bound_l_idx : self.track_bound_l_idx + self.NUM_BOUNDARY_PT
            ]

        self.boundary_left_body.clear()
        for global_pose in self.sliced_track_bound_l.poses:
            body_x, body_y, _ = self.goal_pt_to_body(
                ego_odom.pose.pose.position.x,
                ego_odom.pose.pose.position.y,
                self.ego_yaw,
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                0.0,
            )
            self.boundary_left_body.append([body_x, body_y, _])

        if self.track_bound_r_idx + self.NUM_BOUNDARY_PT > len(
            self.track_bound_r_path_global.poses
        ):
            self.sliced_track_bound_r.poses[
                self.track_bound_r_idx :
            ] = self.track_bound_r_path_global.poses[self.track_bound_r_idx :]
            self.sliced_track_bound_r.poses[
                0 : self.track_bound_r_idx
            ] = self.track_bound_r_path_global.poses[
                : (self.track_bound_r_idx + self.NUM_BOUNDARY_PT)
                - len(self.track_bound_r_path_global.poses)
            ]
        else:
            self.sliced_track_bound_r.poses = self.track_bound_r_path_global.poses[
                self.track_bound_r_idx : self.track_bound_r_idx + self.NUM_BOUNDARY_PT
            ]

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
            self.boundary_right_body.append([body_x, body_y, _])

        if self.racenline_idx + self.NUM_RACELINE_PT > len(
            self.race_line_path_global.poses
        ):
            self.sliced_raceline.poses[
                self.racenline_idx :
            ] = self.race_line_path_global.poses[self.racenline_idx :]
            self.sliced_raceline.poses[
                0 : self.racenline_idx
            ] = self.race_line_path_global.poses[
                : (self.racenline_idx + self.NUM_RACELINE_PT)
                - len(self.race_line_path_global.poses)
            ]
        else:
            self.sliced_raceline.poses = self.race_line_path_global.poses[
                self.racenline_idx : self.racenline_idx + self.NUM_RACELINE_PT
            ]

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

    def opponent_1_callback(self, msg):

        self.last_odom_update = self.get_clock().now()

        self.oppo_1_marker.pose = msg.odometry.pose
        self.pub_oppo_1_marker.publish(self.oppo_1_marker)
        """
        Append msg, assume that the data is updating at 100Hz strictly.
        DOWNSAMPLED 10 TIMES (MATCHED WITH DATASET)
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

    def ego_veh_status_callback(self, msg):

        self.cnt += 1

        self.cnt = self.cnt % 10

        self.ego_marker.pose = msg.odometry.pose
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

        if len(self.odom_buffer) > (
            self.NUM_EGO_PAST_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE
        ):
            self.odom_buffer = self.odom_buffer[
                -1 * int(self.NUM_EGO_PAST_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE) :
            ]
        #     self.odom_buffer_np = np.delete(self.odom_buffer_np, 0, axis=0)

        batch = {}

        if (
            len(self.odom_buffer)
            >= (self.NUM_EGO_PAST_TRAJ_PT / self.EGO_TRAJ_DOWNSAMPLE)
            and len(self.opponent_1_past_traj_body_buffer)
            >= (self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE)
            and len(self.opponent_2_past_traj_body_buffer)
            >= (self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE)
            and len(self.opponent_3_past_traj_body_buffer)
            >= (self.NUM_OPPO_PAST_TRAJ_PT / self.OPPO_TRAJ_DOWNSAMPLE)
        ):
            cnt = 0
            """
            Gen input 1: Past trajectory (to body coordinate)
            """
            # player_past = np.empty((0, 3), dtype=float)
            player_past_list = []
            self.past_traj_path_body.poses = []
            self.past_traj_path_body.header.frame_id = "base_link"

            cur_x = self.cur_odom.pose.pose.position.x
            cur_y = self.cur_odom.pose.pose.position.y
            cur_yaw = self.ego_yaw
            for global_pose in self.odom_buffer:
                body_x, body_y, _ = self.goal_pt_to_body(
                    cur_x,
                    cur_y,
                    cur_yaw,
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

                if cnt % self.EGO_TRAJ_DOWNSAMPLE == 0:
                    player_past_list.append([body_x, body_y, 0.0])
                    self.past_traj_path_body.poses.append(pt_local)
                    cnt = 0
                else:
                    cnt += 1

            player_past = np.array(player_past_list)
            self.player_past_pub.publish(self.past_traj_path_body)

            """
            Gen input 2: Visual feature (Gen self.input_visual_feature)
            """
            # Track geometry (First)
            # self.overlay_lines(self.cur_odom)
            self.get_geometry_info(self.cur_odom)

            # Opponents (Second)
            # self.overlay_opponents(self.cur_odom)

            """
            INPUT modalities
            """

            tic = self.get_clock().now()

            batch["player_past"] = (
                torch.from_numpy(player_past)
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

            print("preparation time : ", toc - tic)

            if self.use_traj_lib:
                # z = self.model._params(
                #     visual_features=batch["visual_features"],
                #     player_past=batch["player_past"],
                # ).repeat((self.arr.shape[0], 1))

                z = self.model._params(
                    player_past=batch["player_past"],
                    left_bound=batch["left_bound"],
                    right_bound=batch["right_bound"],
                    race_line=batch["race_line"],
                    oppo1_body=batch["oppo1_body"],
                    oppo2_body=batch["oppo2_body"],
                    oppo3_body=batch["oppo3_body"],
                ).repeat((self.arr.shape[0], 1))

                traj, prb = self.model.trajectory_library_plan(
                    self.traj_lib, z, costmap=None, phi=1, costmap_only=False
                )

                # GPU to CPU
                traj_cpu_np = traj.detach().cpu().numpy().astype(np.float64)
                prb_cpu_np = prb.detach().cpu().numpy().astype(np.float64)

                # Publish result
                vis_path = Path()
                vis_path.header.frame_id = "base_link"

                for i in range(self.trajs_candidates.shape[0]):
                    for j in range(self.trajs_candidates.shape[1]):
                        pt = PoseStamped()
                        pt.header.frame_id = "base_link"
                        pt.pose.position.x = self.trajs_candidates[i][j][0]
                        pt.pose.position.y = self.trajs_candidates[i][j][1]
                        vis_path.poses.append(pt)
                self.path_pub.publish(vis_path)

            else:
                tic = self.get_clock().now()

                z = self.model._params(
                    player_past=batch["player_past"],
                    left_bound=batch["left_bound"],
                    right_bound=batch["right_bound"],
                    race_line=batch["race_line"],
                    oppo1_body=batch["oppo1_body"],
                    oppo2_body=batch["oppo2_body"],
                    oppo3_body=batch["oppo3_body"],
                ).repeat((self.num_samples, 1))

                toc = self.get_clock().now()

                print("infer time : ", toc - tic)

                samples = self.model._decoder(z).reshape(
                    self.num_samples, self.output_shape[0], self.output_shape[1]
                )

                if self.vis_density_function:
                    _, log_prob, logabsdet, mu_t, sig_t = self.model._decoder._inverse(
                        y=samples, z=z, return_rollouts=True
                    )

                    r = torch.zeros(100, 30).numpy()
                    # self.vis.image(r)

                    def overlay_traj_to_map(traj, imitation_prior, bg):
                        overlay_map = bg.copy()

                        mean_imitation_prior = sum(imitation_prior) / len(
                            imitation_prior
                        )

                        norm_imitation_prior = [
                            float(i) / sum(imitation_prior) for i in imitation_prior
                        ]

                        # (Batch, lenth, pos_dim(x,y,z)):Traj
                        for batch_idx in range(traj.shape[0]):
                            for pose_idx in range(traj.shape[1]):
                                overlay_map[
                                    int(traj[batch_idx][pose_idx][0]),
                                    15 - int(traj[batch_idx][pose_idx][1]),
                                ] = (
                                    imitation_prior[batch_idx] - mean_imitation_prior
                                ) * 100
                        return overlay_map

                    sample_cpu = samples.detach().cpu().numpy().astype(np.float64)
                    log_prob_cpu = log_prob.detach().cpu().numpy().astype(np.float64)
                    mu_t_cpu = np.array(mu_t)
                    sig_t_cpu = np.array(sig_t)

                    # output_datum = dict(
                    #     output_trajs=sample_cpu,
                    #     log_prob=log_prob_cpu,
                    #     mu_t=mu_t_cpu,
                    #     sig_t=sig_t_cpu,
                    # )
                    # filename = (
                    #     "AC_output/" + str(self.get_clock().now().nanoseconds) + ".dill"
                    # )
                    # with open(filename, "wb") as f:
                    #     dill.dump(output_datum, f)

                    """
                    Visdom visualization
                    """

                sample_cpu = samples.detach().cpu().numpy().astype(np.float64)

                # Publish result
                vis_flg = True
                if vis_flg:
                    vis_path = Path()
                    vis_path.header.frame_id = "base_link"

                    for i in range(sample_cpu.shape[0]):
                        for j in range(sample_cpu.shape[1]):
                            pt = PoseStamped()
                            pt.header.frame_id = "base_link"
                            pt.pose.position.x = sample_cpu[i][j][0]
                            pt.pose.position.y = sample_cpu[i][j][1]
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


if __name__ == "__main__":
    main()
