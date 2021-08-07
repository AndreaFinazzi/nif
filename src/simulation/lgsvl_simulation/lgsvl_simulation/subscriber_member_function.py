import rclpy
import csv
import pandas as pd
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Imu, PointCloud2, NavSatFix
from lgsvl_msgs.msg import VehicleOdometry, Detection2DArray

class Drive(Node):
    def __init__(self):
        super().__init__('drive')
        self.msgtype = []
        self.namespace = '' #'/simulator/undergrads'
        # self.sub_gps_odometry = self.create_subscription(Odometry, '/simulator/car1/sensor/gps/odometry', self.callback, 10)
        # self.sub_imu = self.create_subscription(Imu, '/simulator/car1/sensor/imu', self.callback2, 10)
        # self.sub_radar= self.create_subscription(Odometry, '/simulator/car1/sensor/radar', self.callback, 10)

        # Camera subscriptions
        self.sub_camera_front_left= self.create_subscription(CompressedImage, self.namespace + '/sensor/camera_front_left', self.callback_camera_front_left, 10)
        self.sub_camera_front_right= self.create_subscription(CompressedImage, self.namespace + '/sensor/camera_front_right', self.callback_camera_front_right, 10)
        self.sub_camera_rear_left= self.create_subscription(CompressedImage, self.namespace + '/sensor/camera_rear_left', self.callback_camera_rear_left, 10)
        self.sub_camera_rear_right= self.create_subscription(CompressedImage, self.namespace + '/sensor/camera_rear_right', self.callback_camera_rear_right, 10)
        self.sub_camera_front_1= self.create_subscription(CompressedImage, self.namespace + '/sensor/camera_front_1', self.callback_camera_front_1, 10)
        self.sub_camera_front_2= self.create_subscription(CompressedImage, self.namespace + '/sensor/camera_front_2', self.callback_camera_front_2, 10)

        # 2D ground truth sensor subscriptions
        self.sub_2D_front_left= self.create_subscription(Detection2DArray, self.namespace + '/sensor/a2D_front_left', self.callback_2D_front_left, 10)
        self.sub_2D_front_right= self.create_subscription(Detection2DArray, self.namespace + '/sensor/a2D_front_right', self.callback_2D_front_right, 10)
        self.sub_2D_rear_left= self.create_subscription(Detection2DArray, self.namespace + '/sensor/a2D_rear_left', self.callback_2D_rear_left, 10)
        self.sub_2D_rear_right= self.create_subscription(Detection2DArray, self.namespace + '/sensor/a2D_rear_right', self.callback_2D_rear_right, 10)
        self.sub_2D_front_1= self.create_subscription(Detection2DArray, self.namespace + '/sensor/a2D_front_1', self.callback_2D_front_1, 10)
        self.sub_2D_front_2= self.create_subscription(Detection2DArray, self.namespace + '/sensor/a2D_front_2', self.callback_2D_front_2, 10)


        # Lidar subscriptions
        self.sub_laser_meter_flash_a= self.create_subscription(PointCloud2, self.namespace + '/sensor/laser_meter_flash_a', self.callback_laser_meter_flash_a, 10)
        self.sub_laser_meter_flash_b= self.create_subscription(PointCloud2, self.namespace + '/sensor/laser_meter_flash_b', self.callback_laser_meter_flash_b, 10)
        self.sub_laser_meter_flash_c= self.create_subscription(PointCloud2, self.namespace + '/sensor/laser_meter_flash_c', self.callback_laser_meter_flash_c, 10)


        # IMU subscriptions
        self.sub_imu_top= self.create_subscription(Imu, self.namespace + '/novatel_top/imu', self.callback_imu_top, 10)
        self.sub_imu_bottom= self.create_subscription(Imu, self.namespace + '/novatel_bottom/imu', self.callback_imu_bottom, 10)

        # Vehicle odometry subsciptions (includes front/rear wheel angles and velocity)
        self.sub_vehicleodometry= self.create_subscription(VehicleOdometry, self.namespace + '/vehicle/odometry', self.callback_vehicleodometry, 10)

        # GPS subscriptions
        self.sub_gps_top= self.create_subscription(NavSatFix, self.namespace + '/novatel_top/fix', self.callback_gps_top, 10)
        self.sub_gps_bottom= self.create_subscription(NavSatFix, self.namespace + '/novatel_bottom/fix', self.callback_gps_bottom, 10)

    # def callback(self, msg):
    #     self.get_logger().info('Subscribed GPS ODOM: {}'.format(msg.pose.pose.position.x))
    
    def callback_camera_front_left(self, msg):
        pass
        self.get_logger().info('Subscribed camera_front_left')

    def callback_camera_front_right(self, msg):
        pass
        self.get_logger().info('Subscribed camera_front_right')
    
    def callback_camera_rear_left(self, msg):
        pass
        self.get_logger().info('Subscribed camera_rear_left')

    def callback_camera_rear_right(self, msg):
        pass
        self.get_logger().info('Subscribed camera_rear_right')
    
    def callback_camera_front_1(self, msg):
        pass
        self.get_logger().info('Subscribed camera_front_1')

    def callback_camera_front_2(self, msg):
        pass
        self.get_logger().info('Subscribed camera_front_2')




    def callback_2D_front_left(self, msg):
        pass
        self.get_logger().info('Subscribed 2D_front_left ' + str(msg))

    def callback_2D_front_right(self, msg):
        pass
        self.get_logger().info('Subscribed 2D_front_right ' + str(msg))
    
    def callback_2D_rear_left(self, msg):
        pass
        self.get_logger().info('Subscribed 2D_rear_left '+ str(msg))

    def callback_2D_rear_right(self, msg):
        pass
        self.get_logger().info('Subscribed 2D_rear_right '+ str(msg))
    
    def callback_2D_front_1(self, msg):
        pass
        self.get_logger().info('Subscribed 2D_front_1 '+ str(msg))

    def callback_2D_front_2(self, msg):
        pass
        self.get_logger().info('Subscribed 2D_front_2 '+ str(msg))

    def callback_laser_meter_flash_a(self, msg):
        pass
        self.get_logger().info('Subscribed laser_meter_flash_a')
    
    def callback_laser_meter_flash_b(self, msg):
        pass
        self.get_logger().info('Subscribed laser_meter_flash_b')
    
    def callback_laser_meter_flash_c(self, msg):
        pass
        self.get_logger().info('Subscribed laser_meter_flash_c')



    def callback_imu_top(self, msg):
        pass
        self.get_logger().info('Subscribed imu_top')

    def callback_imu_bottom(self, msg):
        pass
        self.get_logger().info('Subscribed imu_bottom')



    def callback_vehicleodometry(self, msg):
        pass
        self.get_logger().info('Subscribed vehicleodometry')


    def callback_gps_top(self, msg):
        pass
        self.get_logger().info('Subscribed gps_top')

    def callback_gps_bottom(self, msg):
        pass
        self.get_logger().info('Subscribed gps_bottom latitude: ' + str(msg.latitude) + ' altitude: '+ str(msg.altitude))
        # WPT_CSV_PATH = "./src/subtest/wpt_data/wpt_from_GPS2.csv"
        # csv_data = pd.read_csv(WPT_CSV_PATH, sep=',', header=None)
        # wpts_x = csv_data.values[:,0]
        # wpts_y = csv_data.values[:,1]
        # f = open('./src/subtest/wpt_data/wpt_from_GPS.csv','a',newline='')
        # wr = csv.writer(f)
        # wr.writerow([str(msg.longitude),str(msg.latitude)])
        # f.close


def main(args=None):
    rclpy.init(args=args)
    drive = Drive()
    rclpy.spin(drive)


if __name__ == '__main__':
    main()
