'''
@file   client.py
@author USRG @ KAIST, Andrea Finazzi
@date   2022-02-10
@brief  Client of the UDP to ROS2 interface for Assetto Corsa
'''

'''
INPUT STRUCTURE OF AC INTERFACE
{
    steer: float32 [-600, 600]      --> Output of control safety layer || topic name : /joystick/steering_cmd
    gas: float32 [0, 100]           --> Output of control safety layer || topic name : /joystick/accelerator_cmd
    brake: float32 [0, 200000]      --> Output of control safety layer || topic name : /joystick/brake_cmd
    gear-down: bool
    geear-up: bool --> For gear shifting, we need to subscribe the current gear and compare with our target gear to keep the machinism of AV21 system. (It directly takes the target gear. UINT8)
}
'''

from http import server
import marshal
from socket import *

import pickle
from matplotlib.transforms import Transform

from sympy import Quaternion, true
from nifpy_common_nodes.base_node import BaseNode

import rclpy
from rclpy.duration import Duration
from std_msgs.msg import UInt8, Float32
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

class ACServerNode(rclpy.node.Node):
    def __init__(self, node_name: str, udp_server : SocketType, verbose_flg: bool) -> None:
        super().__init__(node_name)

        self.udp_server = udp_server
        self.verbose = verbose_flg
        self.is_first_call = true

        # Timers
        self.timer_recv = self.create_timer(TIMER_PERIOD_RECV_S, self.timer_callback)

        if(self.verbose):
            self.get_logger().info('Create AC server node : ROS2 --> UDP')
            self.get_logger().info('Sending commands every "%f"' % TIMER_PERIOD_RECV_S)

        qos_car_count = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        ## Independent subscriptions
        self.topic_name_steer_cmd = "/joystick/steering_cmd"
        self.topic_name_gas_cmd = "/joystick/accelerator_cmd"
        self.topic_name_brake_cmd = "/joystick/brake_cmd"
        self.topic_name_target_gear = "/joystick/gear_cmd"
        self.topic_name_current_gear = "TODO"

        self.sub_steer_cmd = self.create_subscription(Float32,topic_name_steer_cmd,self.steer_cmd_callback,10)
        self.sub_gas_cmd = self.create_subscription(Float32,topic_name_gas_cmd,self.gas_cmd_callback,10)
        self.sub_brake_cmd = self.create_subscription(Float32,topic_name_brake_cmd,self.brake_cmd_callback,10)
        self.sub_target_gear = self.create_subscription(UInt8,topic_name_target_gear,self.target_gear_callback,10)
        self.sub_current_gear = self.create_subscription(UInt8,topic_name_current_gear,self.current_gear_callback,10)

        self.latest_steer_cmd = None
        self.latest_gas_cmd = None
        self.latest_brake_cmd = None
        self.latest_gear_up_cmd = None
        self.latest_gear_down_cmd = None
        self.latest_target_gear = None
        self.latest_current_gear = None
        
        self.now = self.get_clock().now()

    def timer_callback(self):
        # Prepare the dict
        udp_dict = {"steer":self.latest_steer_cmd,
                    "gas": self.latest_gas_cmd,
                    "brake": self.latest_brake_cmd,
                    "gear-down":self.latest_gear_down_cmd,
                    "gear-up":self.latest_gear_up_cmd}

        udp_msg = pickle.dumps(udp_dict)
        udp_server.sendto(udp_msg, ADDR_SERVER)

    def steer_cmd_callback(self,msg):
        if(self.verbose):
            self.get_logger().info('Recieved steer cmd')
        
        # update latest steer cmd
        self.latest_steer_cmd = msg.data
        pass

    def gas_cmd_callback(self,msg):
        if(self.verbose):
            self.get_logger().info('Recieved gas cmd')
        # update latest gas cmd
        self.latest_gas_cmd = msg.data
        pass

    def brake_cmd_callback(self,msg):
        if(self.verbose):
            self.get_logger().info('Recieved brake cmd')
        # update latest brake cmd
        self.latest_brake_cmd = msg.data
        pass

    def target_gear_callback(self,msg):
        if(self.verbose):
            self.get_logger().info('Recieved target gear')
        # update latest target gear
        self.latest_target_gear = msg.data

        if(self.latest_target_gear - self.latest_current_gear > 0):
            # gear up
            self.latest_gear_up_cmd = True
            self.latest_gear_down_cmd = False
        elif(self.latest_target_gear == self.latest_current_gear):
            # stay
            self.latest_gear_up_cmd = False
            self.latest_gear_down_cmd = False
        else:
            # gear down
            self.latest_gear_up_cmd = False
            self.latest_gear_down_cmd = True
        pass

    def current_gear_callback(self,msg):
        if(self.verbose):
            self.get_logger().info('Recieved current gear')
        # update latest current gear
        self.latest_current_gear = msg.data
        pass

def main(args=None):
    # Create a UDP socket at client side
    udp_server = socket(family=AF_INET, type=SOCK_DGRAM)
    udp_server.bind(ADDR_CLIENT)

    udp_server.sendto(str.encode(HANDSHAKE_MSG), ADDR_SERVER)
    udp_server.settimeout(1.0)
    # Send to server using created UDP socket
    # UDPClientSocket.sendto(bytesToSend, ADDR_SERVER)

    rclpy.init(args=args)
    node = ACServerNode("ac_server_node", udp_server,true)

    try:
        rclpy.spin(node)
    finally:
        udp_server.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
