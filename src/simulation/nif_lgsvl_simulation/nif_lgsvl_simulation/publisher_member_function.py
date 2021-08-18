'''
@file   publisher_member_function.py
@auther USRG UGS
@date   2021-08-11
@brief  publisher node for lgsvl simulation with topic names matched to the real car
'''

import rclpy
import math
from rclpy.node import Node
from lgsvl_msgs.msg import VehicleControlData
from std_msgs.msg import Float32, Int8
from raptor_dbw_msgs.msg import SteeringReport, AcceleratorPedalReport, Brake2Report


class Drive(Node):
    def __init__(self):
        super().__init__('lgsvl_publisher')
        self.namespace = ''
        # self.namespace = self.get_namespace
        self.sub_accel = self.create_subscription(Float32, self.namespace + '/raptor_dbw_interface/accelerator_pedal_cmd', self.callback_accel, 1)
        self.sub_steer = self.create_subscription(Float32, self.namespace + '/raptor_dbw_interface/steering_cmd', self.callback_steer, 1)
        self.sub_brake = self.create_subscription(Float32, self.namespace + '/raptor_dbw_interface/brake_cmd', self.callback_brake, 1)
        self.sub_gear = self.create_subscription(Int8, self.namespace + '/raptor_dbw_interface/gear_cmd', self.callback_gear, 1)

        self.control_pub = self.create_publisher(VehicleControlData, self.namespace + '/sensor/control', 1)

        self.steering_report_pub = self.create_publisher(SteeringReport, self.namespace + '/raptor_dbw_interface/steering_report', 10)
        self.accel_pedal_report_pub = self.create_publisher(AcceleratorPedalReport, self.namespace + '/raptor_dbw_interface/accelerator_pedal_report', 10)
        self.brake2_report_pub = self.create_publisher(Brake2Report, self.namespace + '/raptor_dbw_interface/brake_2_report', 10)

        self.timer_period = 0.02  # seconds
        self.timer = self.create_timer(self.timer_period, self.callback)
        self.acceleration_pct = 0.0  # 0 to 1
        self.braking_pct = 0.0  # 0 to 1
        self.target_wheel_angle = 0.0  # radians
        self.target_wheel_angular_rate = 1.0  # radians / second
        self.gear = 1
        self.rolling_counter = 0

    def callback(self):
        message = VehicleControlData()
        message.acceleration_pct = self.acceleration_pct
        message.braking_pct = self.braking_pct
        message.target_wheel_angle = self.target_wheel_angle
        message.target_wheel_angular_rate = self.target_wheel_angular_rate
        message.target_gear = self.gear
        self.control_pub.publish(message)
        self.get_logger().debug('Publishing: Throttle {}'.format(message.acceleration_pct))
        self.get_logger().debug('Publishing: Steer {}'.format(message.target_wheel_angle))

        steer_report_msg = SteeringReport()
        steer_report_msg.header.stamp = self.get_clock().now().to_msg()
        steer_report_msg.rolling_counter = self.rolling_counter % 256
        steer_report_msg.steering_wheel_angle = (int(self.target_wheel_angle * 360 / math.pi)) / 2
        self.steering_report_pub.publish(steer_report_msg)

        accel_pedal_report_msg = AcceleratorPedalReport()
        accel_pedal_report_msg.header.stamp = self.get_clock().now().to_msg()
        accel_pedal_report_msg.rolling_counter = self.rolling_counter % 256
        accel_pedal_report_msg.pedal_output = self.acceleration_pct * 100
        self.accel_pedal_report_pub.publish(accel_pedal_report_msg)

        brake2_report_msg = Brake2Report()
        brake2_report_msg.header.stamp = self.get_clock().now().to_msg()
        brake2_report_msg.rolling_counter = self.rolling_counter % 256
        brake2_report_msg.front_brake_pressure = self.braking_pct * 3447379
        self.brake2_report_pub.publish(brake2_report_msg)

        self.rolling_counter = self.rolling_counter + 1

    def callback_accel(self, msg):
        self.acceleration_pct = msg.data

    def callback_steer(self, msg):
        self.target_wheel_angle = -msg.data * math.pi / 180

    def callback_brake(self, msg):
        self.braking_pct = msg.data / 3447379

    # uint8 GEAR_NEUTRAL = 0
    # uint8 GEAR_DRIVE = 1
    # uint8 GEAR_REVERSE = 2
    # uint8 GEAR_PARKING = 3
    # uint8 GEAR_LOW = 4
    def callback_gear(self, msg):

        if (msg.data > 0):
            self.gear = 1
        elif (msg.data == 0):
            self.gear = 0
        elif (msg.data == -1):
            self.gear = 2


def main(args=None):  # 0 to 1
    rclpy.init(args=args)
    drive = Drive()
    rclpy.spin(drive)


if __name__ == '__main__':
    main()
