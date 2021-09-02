# Copyright 2021 Matt Boler
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License. 

"""
TODO: At the moment we only support local NED coordinates
"""
import os

from timeit import default_timer as timer

import rclpy
from rclpy.node import Node

import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg
#import novatel_gps_msgs.msg
import novatel_oem7_msgs.msg

import numpy as np
import pymap3d as pm
from scipy.spatial.transform import Rotation

from . import utils
from typing import List

from ament_index_python.packages import get_package_share_directory

class PathServer(Node):
    """
    """
    def __init__(self) -> None:
        super().__init__('path_server_node')
        # Set your parameters here
        # A smarter man would have made a launch file for you, but here I am
        self.declare_parameter('map_path', 'maps/IMS/Pitlane/LatLon/inner_bounds.csv')
        #self.declare_parameter('pose_topic', '/novatel_bottom/inspva')
        self.declare_parameter('pose_topic','/novatel_bottom/inspva')
        self.declare_parameter('path_topic', '/target_path')
        self.declare_parameter('lookahead_distance', 100)
        self.declare_parameter('local_lat', 39.8125900071711)
        self.declare_parameter('local_lon', -86.3418060783425)
        # Load parameters
        pose_topic = self.get_parameter('pose_topic').value
        path_topic = self.get_parameter('path_topic').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        map_path = os.path.join(get_package_share_directory('path_server'), self.get_parameter('map_path').value)
        self.lat0 = self.get_parameter('local_lat').value
        self.lon0 = self.get_parameter('local_lon').value
        # Set everything and initialise
        print("--- Assembling Path Manager ---")
        print("Looking for map at: ", map_path)
        print("Looking: ", self.lookahead_distance, " meters ahead of the vehicle")
        self.PathManager = utils.PathManager(map_path=map_path, lat0=self.lat0, lon0=self.lon0)
        self.publisher = self.create_publisher(nav_msgs.msg.Path, path_topic, 1)
        self.subscription = self.create_subscription(novatel_oem7_msgs.msg.INSPVA, pose_topic, self.poseCallback, 1)
        print("--- Waiting for Poses ---")
        self.debug = False
    
    def poseCallback(self, 
        msg: novatel_oem7_msgs.msg.INSPVA
        ) -> None:
        ros_header = msg.header
        vehicle_pose = self.convertFromInspva(msg=msg)
        if (self.debug):
            print("Vehicle pose:")
            print(vehicle_pose.position)
            print(vehicle_pose.orientation)
        waypoints = self.PathManager.getNextPointsByDistance(vehicle_pose, self.lookahead_distance)
        if (self.debug):
            print("Waypoints:")
            print(waypoints)
        ros_path = self.convertToRosPath(waypoints, ros_header)
        if (self.debug):
            print("Ros path:")
            print(ros_path)
        self.publisher.publish(ros_path)

#    def poseCallback(self, 
#        msg: novatel_gps_msgs.msg.Inspva
#        ) -> None:
#        ros_header = msg.header
#        vehicle_pose = self.convertFromInspva(msg=msg)
#        if (self.debug):
#            print("Vehicle pose:")
#            print(vehicle_pose.position)
#            print(vehicle_pose.orientation)
#        waypoints = self.PathManager.getNextPointsByDistance(vehicle_pose, self.lookahead_distance)
#        if (self.debug):
#            print("Waypoints:")
#            print(waypoints)
#        ros_path = self.convertToRosPath(waypoints, ros_header)
#        if (self.debug):
#            print("Ros path:")
#            print(ros_path)
#        self.publisher.publish(ros_path)

#    def convertFromInspva(self,
#        msg: novatel_gps_msgs.msg.Inspva
#        ) -> utils.Pose:
#        n, e, d = pm.geodetic2ned(msg.latitude, msg.longitude, 0, self.lat0, self.lon0, 0)
#        yaw = np.deg2rad(msg.azimuth) # Might have the sign flipped here
#        position = np.asarray([[n], [e], [d]])
#        r = Rotation.from_euler('z', yaw)
#        return utils.Pose(position=position, orientation=r)

    def convertFromInspva(self,
        msg: novatel_oem7_msgs.msg.INSPVA
        ) -> utils.Pose:
        n, e, d = pm.geodetic2ned(msg.latitude, msg.longitude, 0, self.lat0, self.lon0, 0)
        yaw = np.deg2rad(msg.azimuth) # Might have the sign flipped here
        position = np.asarray([[n], [e], [d]])
        r = Rotation.from_euler('z', yaw)
        return utils.Pose(position=position, orientation=r)
    
    def convertToRosPose(self, 
        position: np.ndarray,
        yaw: float
        ) -> geometry_msgs.msg.Pose:
        # This is kinda ugly
        point = geometry_msgs.msg.Point()
        point.x = position[0]
        point.y = position[1]
        point.z = position[2]
        quat = geometry_msgs.msg.Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = np.sin(yaw / 2)
        quat.w = np.cos(yaw / 2)
        ros_pose = geometry_msgs.msg.Pose()
        ros_pose.position = point
        ros_pose.orientation = quat
        return ros_pose
    
    def convertToRosPoseStamped(self,
        position,
        yaw,
        header: std_msgs.msg.Header
        ) -> geometry_msgs.msg.PoseStamped:
        ros_pose = self.convertToRosPose(position, yaw)
        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose = ros_pose
        return pose_stamped

    def convertToRosPath(self, 
        path: np.ndarray, 
        header: std_msgs.msg.Header
        ) -> nav_msgs.msg.Path:
        ros_path = nav_msgs.msg.Path()
        ros_path.header = header
        for i in range(path.shape[1]):
            val = path[:, i]
            position = val[0:3]
            yaw = val[3]
            ros_pose_stamped = self.convertToRosPoseStamped(position, yaw, header)
            ros_path.poses.append(ros_pose_stamped)
        return ros_path


def main(args=None):
    rclpy.init(args=args)
    node = PathServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
