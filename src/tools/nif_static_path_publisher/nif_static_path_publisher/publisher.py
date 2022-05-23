'''
@file   client.py
@author USRG @ KAIST, Andrea Finazzi
@date   2022-02-10
@brief  Client of the UDP to ROS2 interface for Assetto Corsa
'''

# Simulation state dict:
#
#
# 3D vectors
# csWorldPosition = [ac.getCarState(cid, acsys.CS.WorldPosition) for cid in range(carsCount)],
# csLinearVelocityVector = [ac.getCarState(cid, acsys.CS.Velocity) for cid in range(carsCount)],
# csLinearVelocityVectorLocal = [ac.getCarState(cid, acsys.CS.LocalVelocity) for cid in range(carsCount)],
# csAngularVelocityVectorLocal = [ac.getCarState(cid, acsys.CS.LocalAngularVelocity) for cid in range(carsCount)],
# csWheelAngularVelocityVector = [ac.getCarState(cid, acsys.CS.WheelAngularSpeed) for cid in range(carsCount)],

# # 4D vectors
# csSlipAngle = [ac.getCarState(cid, acsys.CS.SlipAngle) for cid in range(carsCount)],
# csSlipRatio = [ac.getCarState(cid, acsys.CS.SlipRatio) for cid in range(carsCount)],
# csTyreSlip = [ac.getCarState(cid, acsys.CS.TyreSlip) for cid in range(carsCount)],
# # Depending on TyreSlip format...
# # csTyreSlip = {
#     # 'FL' : [ac.getCarState(cid, acsys.CS.TyreSlip, 'FL') for cid in range(carsCount)],
#     # 'FR' : [ac.getCarState(cid, acsys.CS.TyreSlip, 'FR') for cid in range(carsCount)],
#     # 'RL' : [ac.getCarState(cid, acsys.CS.TyreSlip, 'RL') for cid in range(carsCount)],
#     # 'RR' : [ac.getCarState(cid, acsys.CS.TyreSlip, 'RR') for cid in range(carsCount)],
# # }

# # Scalar
# csNormSplinePosition = [ac.getCarState(cid, acsys.CS.NormalizedSplinePosition) for cid in range(carsCount)],
# csLinearSpeedMPS = [ac.getCarState(cid, acsys.CS.SpeedMS) for cid in range(carsCount)],
# csControlSteerAngle =  [ac.getCarState(cid, acsys.CS.Steer) for cid in range(carsCount)],
# csControlGasNorm =  [ac.getCarState(cid, acsys.CS.Gas) for cid in range(carsCount)],
# csControlBrakeNorm =  [ac.getCarState(cid, acsys.CS.Brake) for cid in range(carsCount)],
# csControlClutchNorm =  [ac.getCarState(cid, acsys.CS.Clutch) for cid in range(carsCount)],
# csControlGear =  [ac.getCarState(cid, acsys.CS.Gear) for cid in range(carsCount)],

# csLapTime = [ac.getCarState(cid, acsys.CS.LapTime) for cid in range(carsCount)],
# csLapCount = [ac.getCarState(cid, acsys.CS.LapCount) for cid in range(carsCount)],
# csLastLap = [ac.getCarState(cid, acsys.CS.LastLap) for cid in range(carsCount)],
# csBestLap = [ac.getCarState(cid, acsys.CS.BestLap) for cid in range(carsCount)],
# csLeaderboardPosition = [ac.getCarRealTimeLeaderboardPosition(cid) for cid in range(carsCount)],

# # Bool {0, 1}
# isLapInvalidated = [ac.getCarState(cid, acsys.CS.LapInvalidated) for cid in range(carsCount)],
# isCarInPitlane = [ac.isCarInPitline(cid) for cid in range(carsCount)],
# isCarInPit = [ac.isCarInPit(cid) for cid in range(carsCount)],
# isAIcontrolled = [ac.isAIControlled(cid) for cid in range(carsCount)],

# trackName = ac.getTrackName(0),
# trackConfig = ac.getTrackConfiguration(0),
# carsCount = carsCount,
# carNames = [ac.getCarName(cid) for cid in range(carsCount)],
# csRaceFinished = [ac.getCarState(cid, acsys.CS.RaceFinished) for cid in range(carsCount)],

from ament_index_python import get_package_share_directory
from http import server
import marshal
from socket import *

import pickle
from matplotlib.transforms import Transform

from sympy import Quaternion, true
from nifpy_common_nodes.base_node import BaseNode

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import UInt8
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
import pandas as pd
import os

def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)

class PubNode(rclpy.node.Node):
    def __init__(self, node_name: str, ) -> None:
        super().__init__(node_name)

        self.target_frame = "odom"

        # Timers, 1Hz
        self.timer_recv = self.create_timer(0.01, self.timer_callback)

        # Publishers
        self.pub_path_candidates_1 = self.create_publisher(Path, '/path_candidates_1', rclpy.qos.qos_profile_sensor_data)
        self.pub_path_candidates_2 = self.create_publisher(Path, '/path_candidates_2', rclpy.qos.qos_profile_sensor_data)
        self.pub_path_candidates_3 = self.create_publisher(Path, '/path_candidates_3', rclpy.qos.qos_profile_sensor_data)

        self.path_1 = Path()
        self.path_1.header.frame_id = self.target_frame
        self.path_2 = Path()
        self.path_2.header.frame_id = self.target_frame
        self.path_3 = Path()
        self.path_3.header.frame_id = self.target_frame

        self.track_db_path = get_share_file(
            "nif_imitative_planning_nodes", "nif_imitative_planning_nodes/ac_track_db")

        # Read files
        self.path_1_file_path = self.track_db_path + "/LVMS/lvms_inner_line.csv"
        self.path_2_file_path = self.track_db_path + "/LVMS/lvms_outer_line.csv"
        self.path_3_file_path = self.track_db_path + "/LVMS/race_line_w_field.csv"

        self.file_list = [self.path_1_file_path, self.path_2_file_path, self.path_3_file_path]

        self.init_paths()

    def timer_callback(self):
        self.pub_path_candidates_1.publish(self.path_1)
        self.pub_path_candidates_2.publish(self.path_2)
        self.pub_path_candidates_3.publish(self.path_3)

    def init_paths(self):

        # read files
        path_1 = pd.read_csv(self.path_1_file_path)
        path_2 = pd.read_csv(self.path_2_file_path)
        path_3 = pd.read_csv(self.path_3_file_path)

        for pt in zip(path_1.iloc[:, 0], path_1.iloc[:, 1]):
            p = PoseStamped()
            p.header.frame_id = self.target_frame
            p.pose.position.x = pt[0]
            p.pose.position.y = pt[1]
            self.path_1._poses.append(p)
        for pt in zip(path_2.iloc[:, 0], path_2.iloc[:, 1]):
            p = PoseStamped()
            p.header.frame_id = self.target_frame
            p.pose.position.x = pt[0]
            p.pose.position.y = pt[1]
            self.path_2._poses.append(p)
        for pt in zip(path_3.iloc[:, 0], path_3.iloc[:, 1]):
            p = PoseStamped()
            p.header.frame_id = self.target_frame
            p.pose.position.x = pt[0]
            p.pose.position.y = pt[1]
            self.path_3._poses.append(p)
        self.path_1.header.frame_id = self.target_frame
        self.path_2.header.frame_id = self.target_frame
        self.path_3.header.frame_id = self.target_frame

        print(len(self.path_1.poses))
        print(len(self.path_2.poses))
        print(len(self.path_3.poses))

    def timer_recv_callback(self):
        try:
            data, addr = self.udp_client.recvfrom(BUFFER_SIZE)
            data_loaded = pickle.loads(data)  # data loaded.

            self.now = self.get_clock().now()
            self.parseState(data_loaded)
            # print(str(data_loaded))
        except timeout:
            print("Timeout")

    def parseState(self, state: dict):
        if self.is_first_call:
            self.is_first_call = False

            for cid in range(state['carsCount']):
                self.pubs_car_status.append(self.create_publisher(
                    ACTelemetryCarStatus, '/ac/car_status_' + str(cid), rclpy.qos.qos_profile_sensor_data))

        car_count = UInt8()
        car_count.data = state['carsCount']
        self.pub_car_count.publish(car_count)

        self.update_odom_ground_truth(state)
        self.update_oppo_markers(state)

    def update_odom_ground_truth(self, state: dict):
        self.ego_odom.pose.pose.position.x = state['csWorldPosition'][0][0]
        self.ego_odom.pose.pose.position.y = state['csWorldPosition'][0][1]
        self.ego_odom.pose.pose.position.z = state['csWorldPosition'][0][2]

        self.ego_odom.header.stamp = self.now.to_msg()

        self.pub_odom_ground_truth.publish(self.ego_odom)
        # self.tf_broadcast(self.ego_odom)

    def update_oppo_markers(self, state):
        marker_array = MarkerArray()

        for (cid, csWorldPosition) in enumerate(state['csWorldPosition']):
            # marker = Marker()
            # marker.header.stamp = self.now.to_msg()
            # marker.header.frame_id = R_FRAME_ODOM

            # marker.pose.position.x = csWorldPosition[0]
            # marker.pose.position.y = csWorldPosition[1]
            # marker.pose.position.z = csWorldPosition[2]

            # marker.scale.x = 10.0
            # marker.scale.y = 1.0
            # marker.scale.z = 1.0

            # marker.color.r = 0.0
            # marker.color.b = 1.0
            # marker.color.g = 1.0
            # marker.color.a = 1.0
            # # marker.scale.x = 1.0 + tracked_objects.perception_list[tracked_obj_idx].obj_velocity_in_local.linear.x;
            # # marker.scale.y = 1.0;
            # # marker.scale.z = 1.0;

            # marker.id = cid
            # marker.lifetime = Duration(seconds=0, nanoseconds=20000000).to_msg()

            # marker_array.markers.append(marker)

            # Create Car Status Message
            car_status = ACTelemetryCarStatus()
            car_status.header.frame_id = "odom"
            car_status.header.stamp = self.now.to_msg()

            car_status.cid = cid

            car_status.world_position.x = csWorldPosition[0]
            car_status.world_position.y = csWorldPosition[1]
            car_status.world_position.z = csWorldPosition[2]

            car_status.linear_velocity_vector.x = state['csLinearVelocityVector'][cid][0]
            car_status.linear_velocity_vector.y = state['csLinearVelocityVector'][cid][1]
            car_status.linear_velocity_vector.z = state['csLinearVelocityVector'][cid][2]

            car_status.twist_local.linear.x = state['csLinearVelocityVectorLocal'][cid][0]
            car_status.twist_local.linear.y = state['csLinearVelocityVectorLocal'][cid][1]
            car_status.twist_local.linear.z = state['csLinearVelocityVectorLocal'][cid][2]

            car_status.twist_local.angular.x = state['csAngularVelocityVectorLocal'][cid][0]
            car_status.twist_local.angular.y = state['csAngularVelocityVectorLocal'][cid][1]
            car_status.twist_local.angular.z = state['csAngularVelocityVectorLocal'][cid][2]

            car_status.wheel_angular_speed.append(state['csWheelAngularVelocityVector'][cid][0])
            car_status.wheel_angular_speed.append(state['csWheelAngularVelocityVector'][cid][1])
            car_status.wheel_angular_speed.append(state['csWheelAngularVelocityVector'][cid][2])
            car_status.wheel_angular_speed.append(state['csWheelAngularVelocityVector'][cid][3])

            car_status.slip_angle.append(state['csSlipAngle'][cid][0])
            car_status.slip_angle.append(state['csSlipAngle'][cid][1])
            car_status.slip_angle.append(state['csSlipAngle'][cid][2])
            car_status.slip_angle.append(state['csSlipAngle'][cid][3])

            car_status.slip_ratio.append(state['csSlipRatio'][cid][0])
            car_status.slip_ratio.append(state['csSlipRatio'][cid][1])
            car_status.slip_ratio.append(state['csSlipRatio'][cid][2])
            car_status.slip_ratio.append(state['csSlipRatio'][cid][3])

            car_status.tyre_slip.append(state['csTyreSlip'][cid][0])
            car_status.tyre_slip.append(state['csTyreSlip'][cid][1])
            car_status.tyre_slip.append(state['csTyreSlip'][cid][2])
            car_status.tyre_slip.append(state['csTyreSlip'][cid][3])

            car_status.linear_speed_mps = state['csLinearSpeedMPS'][cid]
            car_status.control_steer_angle = state['csControlSteerAngle'][cid]
            car_status.control_gas_norm = state['csControlGasNorm'][cid]
            car_status.control_brake_norm = state['csControlBrakeNorm'][cid]
            car_status.control_clutch_norm = state['csControlClutchNorm'][cid]
            car_status.control_gear = state['csControlGear'][cid]

            car_status.norm_spline_position = state['csNormSplinePosition'][cid]
            car_status.lap_count = state['csLapCount'][cid]
            car_status.current_lap_time = state['csLapTime'][cid]
            car_status.last_lap_time = state['csLastLap'][cid]
            car_status.best_lap_time = state['csBestLap'][cid]

            car_status.leaderboard_position = state['csLeaderboardPosition'][cid]

            car_status.is_lap_invalidated = bool(state['isLapInvalidated'][cid])
            car_status.is_car_in_pitlane = bool(state['isCarInPitlane'][cid])
            car_status.is_car_in_pit = bool(state['isCarInPit'][cid])
            car_status.is_ai_controlled = bool(state['isAIcontrolled'][cid])
            car_status.is_race_finished = bool(state['csRaceFinished'][cid])

            self.pubs_car_status[cid].publish(car_status)

        # self.pub_oppo_markers.publish(marker_array)

    def tf_broadcast(self, msg):
        tfs = TransformStamped()
        tfs.header = msg.header
        tfs.child_frame_id = msg.child_frame_id
        tfs.transform.translation.x = msg.pose.pose.position.x
        tfs.transform.translation.y = msg.pose.pose.position.y
        tfs.transform.translation.z = msg.pose.pose.position.z
        tfs.transform.rotation.x = msg.pose.pose.orientation.x
        tfs.transform.rotation.y = msg.pose.pose.orientation.y
        tfs.transform.rotation.z = msg.pose.pose.orientation.z
        tfs.transform.rotation.w = msg.pose.pose.orientation.w
        self._tf_publisher.sendTransform(tfs)


def main(args=None):

    rclpy.init(args=args)
    node = PubNode("pub_node")

    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
