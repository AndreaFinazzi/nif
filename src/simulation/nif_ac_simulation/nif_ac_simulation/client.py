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
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from nif_msgs.msg import ACTelemetryCarStatus
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy

HANDSHAKE_MSG       = "usrg.racing"
ADDR_SERVER         = ("143.248.100.101", 4444)
ADDR_CLIENT         = ("0.0.0.0", 4445)
BUFFER_SIZE         = 16000
CLIENT_TIMEOUT_S    = 1.0

R_FRAME_ODOM = "odom"
R_FRAME_BODY = "base_link"

TIMER_PERIOD_RECV_S = 0.01

class ACClientNode(rclpy.node.Node):
    def __init__(self, node_name: str, udp_client : SocketType) -> None:
        super().__init__(node_name)

        self.udp_client = udp_client
        self.is_first_call = true

        self.ego_odom = Odometry()
        self.ego_odom.header.frame_id = R_FRAME_ODOM
        self.ego_odom.child_frame_id = R_FRAME_BODY

        # Timers
        self.timer_recv = self.create_timer(TIMER_PERIOD_RECV_S, self.timer_recv_callback)

        # Publishers

        qos_car_count = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        ## TODO: should become a service
        self.pub_car_count = self.create_publisher(UInt8, '/ac/car_count', qos_car_count)
        self.pub_odom_ground_truth = self.create_publisher(Odometry, '/sensor/odom_ground_truth', rclpy.qos.qos_profile_sensor_data)
        self.pub_oppo_markers = self.create_publisher(MarkerArray, '/ac/vis/cars', rclpy.qos.qos_profile_sensor_data)
        self.pubs_car_status = []

        # TF Broadcaster
        self._tf_publisher = TransformBroadcaster(self)

        self.now = self.get_clock().now()

    def timer_recv_callback(self):
        try:
            data, addr = self.udp_client.recvfrom(BUFFER_SIZE)
            data_loaded = pickle.loads(data) #data loaded.

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
        self.tf_broadcast(self.ego_odom)


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
    # Create a UDP socket at client side
    udp_client = socket(family=AF_INET, type=SOCK_DGRAM)
    udp_client.bind(ADDR_CLIENT)

    udp_client.sendto(str.encode(HANDSHAKE_MSG), ADDR_SERVER)
    udp_client.settimeout(1.0)
    # Send to server using created UDP socket
    # UDPClientSocket.sendto(bytesToSend, ADDR_SERVER)

    rclpy.init(args=args)
    node = ACClientNode("ac_client_node", udp_client)

    try:
        rclpy.spin(node)
    finally:
        udp_client.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
