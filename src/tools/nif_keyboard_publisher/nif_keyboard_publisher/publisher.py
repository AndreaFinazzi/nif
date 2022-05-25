from ament_index_python import get_package_share_directory
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSLivelinessPolicy
from nifpy_common_nodes.base_node import BaseNode



import threading
import rclpy
import os

from std_msgs.msg import UInt8

NODE_NAME = "simple_publisher"


def handle_keyboard(publisher):
    while True:
        print('\n- RACE Menu -')
        print('   1. Command (Move along path race_line)')
        print('   2. Command (Move along path left_center_line)')
        print('   3. Command (Move along path center_line)')
        print('   4. Command (Move along path right_center_line)')
        print('   99. Exit')

        menu = input('Input the menu: ')

        if menu == '1':
            msg = UInt8()
            msg.data = 0
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.data))

        elif menu == '2':
            msg = UInt8()
            msg.data = 1
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.data))

        elif menu == '3':
            msg = UInt8()
            msg.data = 2
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.data))

        elif menu == '4':
            msg = UInt8()
            msg.data = 3
            publisher.publish(msg)
            print(" \n>>> command is published : {0}".format(msg.data))

        elif menu == '99':
            rclpy.shutdown()
            os._exit(1)


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(NODE_NAME)
    publisher = node.create_publisher(UInt8, "keystrike", 10)

    th = threading.Thread(target=handle_keyboard, args=(publisher,))
    th.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
