'''
@file   multi_vehicle_interface.py
@author USRG @ KAIST, Andrea Finazzi
@date   2022-02-10
@brief  Interface for LGSVL, multi-vehicle communication
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
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Vector3, Quaternion
from nif_msgs.msg import Perception3D, Perception3DArray


HANDSHAKE_MSG       = "usrg.racing"
ADDR_SERVER         = ("192.168.0.16", 4444)
ADDR_CLIENT         = ("0.0.0.0", 4444)
BUFFER_SIZE         = 4092
CLIENT_TIMEOUT_S    = 1.0

R_FRAME_ODOM = "odom"
R_FRAME_BODY = "base_link"

udp_client = socket(family=AF_INET, type=SOCK_DGRAM)

class ACClientNode(rclpy.node.Node):
    def __init__(self, node_name: str) -> None:
        super().__init__(node_name)
        self.ego_odom = Odometry()
        self.ego_odom.header.frame_id = R_FRAME_ODOM
        self.ego_odom.child_frame_id = R_FRAME_BODY

        # Timers
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Publishers
        self.pub_oppo_markers = self.create_publisher(Marker, '/lgsvl/oppo/vis', rclpy.qos.qos_profile_sensor_data)
        self.pub_oppo_perception = self.create_publisher(Perception3DArray, '/lgsvl/oppo', rclpy.qos.qos_profile_sensor_data)

        self.sub_ego_odom = self.create_subscription(Odometry, '/sensor/odom_ground_truth', self.ego_odom_callback, rclpy.qos.qos_profile_sensor_data)

        # TF Broadcaster
        self._tf_publisher = TransformBroadcaster(self)

    def publish_marker(self, perception_array_msg : Perception3DArray):
        for (cid, perception_item) in enumerate(perception_array_msg.perception_list):
            marker = Marker()
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.header.frame_id = R_FRAME_ODOM

            marker.pose = perception_item.detection_result_3d.center

            marker.scale.x = 6.0
            marker.scale.y = 3.0
            marker.scale.z = 1.0

            marker.color.r = 1.0
            marker.color.b = 0.0
            marker.color.g = 0.0
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

    def ego_odom_callback(self, msg : Odometry):
        data_string = pickle.dumps(msg, -1)
        udp_client.sendto(data_string, ADDR_SERVER)
        
    def timer_callback(self):
        try:
            data, addr = udp_client.recvfrom(BUFFER_SIZE)
            data_loaded = pickle.loads(data) #data loaded.
            
            perception_msg = self.oppo_odom_to_perception_msg(data_loaded)

            self.pub_oppo_perception.publish(perception_msg)
            self.publish_marker(perception_msg)
            # print(str(data_loaded))
        except timeout:
            print("Timeout")

    def oppo_odom_to_perception_msg(self, oppo_odom : Odometry) -> Perception3DArray:
        # TF and convert to Perception3DArray
        perception_msg = Perception3DArray()
        perception_msg.header = oppo_odom.header

        perception_item = Perception3D()
        perception_item.header = oppo_odom.header
        perception_item.id = 1
        perception_item.detection_result_3d.center = oppo_odom.pose.pose
        perception_item.obj_velocity_in_global = oppo_odom.twist.twist

        perception_msg.perception_list.append(perception_item)

        return perception_msg
# def convert_global_to_body(ego_odom, pose_in_global) -> Pose:

def main(args=None):
    # Create a UDP socket at client side
    udp_client.bind(ADDR_CLIENT)

    udp_client.settimeout(1.0)
    # Send to server using created UDP socket
    # UDPClientSocket.sendto(bytesToSend, ADDR_SERVER)

    rclpy.init(args=args)
    node = ACClientNode("lgsvl_multi_node")
    rclpy.spin(node)

    rclpy.shutdown()
    udp_client.close()

if __name__ == '__main__':
    main()
