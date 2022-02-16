'''
@file   client.py
@author USRG @ KAIST, Andrea Finazzi
@date   2022-02-10
@brief  Client of the UDP to ROS2 interface for Assetto Corsa
'''

# Simulation state dict:
#
#
#    res = dict(
#        trackName = ac.getTrackName(0),
#        trackConfig = ac.getTrackConfiguration(0),
#        carsCount = carsCount,
#        driverNames = [ac.getDriverName(cid) for cid in range(carsCount)],
#        carNames = [ac.getCarName(cid) for cid in range(carsCount)],
#        csVel = [ac.getCarState(cid, acsys.CS.Velocity) for cid in range(carsCount)],
#        csLapTime = [ac.getCarState(cid, acsys.CS.LapTime) for cid in range(carsCount)],
#        csNormSplinePosition = [ac.getCarState(cid, acsys.CS.NormalizedSplinePosition) for cid in range(carsCount)],
#        csLapCount = [ac.getCarState(cid, acsys.CS.LapCount) for cid in range(carsCount)],
#        csLastLap = [ac.getCarState(cid, acsys.CS.LastLap) for cid in range(carsCount)],
#        csBestLap = [ac.getCarState(cid, acsys.CS.BestLap) for cid in range(carsCount)],
#        csWorldPosition = [ac.getCarState(cid, acsys.CS.WorldPosition) for cid in range(carsCount)],
#        csRaceFinished = [ac.getCarState(cid, acsys.CS.RaceFinished) for cid in range(carsCount)],
#        currentSplits = [ac.getCurrentSplits(cid) for cid in range(carsCount)],
#        isCarInPitlane = [ac.isCarInPitline(cid) for cid in range(carsCount)],
#        isCarInPit = [ac.isCarInPit(cid) for cid in range(carsCount)],
#        isConnected = [ac.isConnected(cid) for cid in range(carsCount)],
#        getCarBallast = [ac.getCarBallast(cid) for cid in range(carsCount)],
#        getCarTyreCompound = [ac.getCarTyreCompound(cid) for cid in range(carsCount)],
#        getFocusedCar = ac.getFocusedCar(),
#        getServerIP = ac.getServerIP(),
#        getServerHttpPort = ac.getServerHttpPort(),
#        getServerName = ac.getServerName(),
#    )

from http import server
from socket import *

import pickle
from matplotlib.transforms import Transform

from sympy import Quaternion
from nifpy_common_nodes.base_node import BaseNode

import rclpy
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion

HANDSHAKE_MSG       = "usrg.racing"
ADDR_SERVER         = ("143.248.100.101", 4444)
ADDR_CLIENT         = ("0.0.0.0", 4445)
BUFFER_SIZE         = 4092
CLIENT_TIMEOUT_S    = 1.0

R_FRAME_ODOM = "odom"
R_FRAME_BODY = "base_link"

class ACClientNode(rclpy.node.Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.ego_odom = Odometry()
        self.ego_odom.header.frame_id = R_FRAME_ODOM
        self.ego_odom.child_frame_id = R_FRAME_BODY

        # Timers
        self.timer = self.create_timer(self.timer_period, self.callback)

        # Publishers
        self.pub_odom_ground_truth = self.create_publisher(Odometry, '/sensor/odom_ground_truth', rclpy.qos.qos_profile_sensor_data)
        self.pub_oppo_markers = self.create_publisher(MarkerArray, '/ac/vis/cars', rclpy.qos.qos_profile_sensor_data)

        # TF Broadcaster
        self._tf_publisher = TransformBroadcaster(self)

    def parseState(self, state: dict):
        self.update_odom_ground_truth(state)
        self.update_oppo_markers(state)


    def update_odom_ground_truth(self, state: dict):
        self.ego_odom.pose.pose.position.x = state['csWorldPosition'][0][0]
        self.ego_odom.pose.pose.position.y = state['csWorldPosition'][0][1]
        self.ego_odom.pose.pose.position.z = state['csWorldPosition'][0][2]

        self.ego_odom.header.stamp = self.get_clock().now().to_msg()

        self.pub_odom_ground_truth.publish(self.ego_odom)
        self.tf_broadcast(self.ego_odom)


    def update_oppo_markers(self, state):
        for (cid, csWorldPosition) in enumerate(state['csWorldPosition']):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = R_FRAME_ODOM

            marker.pose.position.x = csWorldPosition[0]
            marker.pose.position.y = csWorldPosition[1]
            marker.pose.position.z = csWorldPosition[2]

            marker.scale.x = 10.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            marker.color.r = 0.0
            marker.color.b = 1.0
            marker.color.g = 1.0
            marker.color.a = 1.0
            # marker.scale.x = 1.0 + tracked_objects.perception_list[tracked_obj_idx].obj_velocity_in_local.linear.x;
            # marker.scale.y = 1.0;
            # marker.scale.z = 1.0;

            marker.id = cid
            marker.lifetime = Duration(seconds=0, nanoseconds=20000000).to_msg()

            self.pub_oppo_markers.publish(marker)


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
    node = ACClientNode("ac_client_node")

    # Temporarily replace ros spin with this loop
    # Should implement a timer later on.
    try:
        while True:
            try:
                data, addr = udp_client.recvfrom(BUFFER_SIZE)
                data_loaded = pickle.loads(data) #data loaded.
                node.parseState(data_loaded)
                # print(str(data_loaded))
            except timeout:
                print("Timeout")
    finally:
        udp_client.close()


    # rclpy.spin(drive)

if __name__ == '__main__':
    main()
