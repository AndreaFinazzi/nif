import math
import configparser
import time
import json
import datetime
import numpy as np
import sys, os

from ament_index_python import get_package_share_directory


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


lib_path_default = get_share_file('nif_multilayer_planning_nodes',
                                  "../../lib/GraphBasedLocalTrajectoryPlanner/graph_ltpl")

sys.path.insert(0, lib_path_default)
for dir_name in os.listdir(lib_path_default):
    dir_path = os.path.join(lib_path_default, dir_name)
    if os.path.isdir(dir_path):
        sys.path.insert(0, dir_path)
os.environ['OPENBLAS_NUM_THREADS'] = str(1)


# TODO : I don't know why but currently, we have to import the graph library in this order
from Graph_LTPL import Graph_LTPL
from imp_global_traj.src import *


from nif_msgs.msg import Perception3DArray
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from rclpy.node import Node
import rclpy


class GraphBasedPlanner(Node):

    def __init__(self):
        super().__init__('graph_based_planner_node')

        # Subscribers and Publisher
        self.local_maptrack_inglobal_pub = self.create_publisher(Path, 'topic_name_local_maptrack_inglobal', 10)
        self.veh_odom_sub = self.create_subscription(Odometry, 'topic_name_veh_odom', self.veh_odom_callback, 10)
        self.perception_result_sub = self.create_subscription(Perception3DArray, 'topic_name_perception_result', self.perception_result_callback, 10)

        self.current_veh_odom = None

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # top level path (module directory)
        toppath = "external/GraphBasedLocalTrajectoryPlanner"
        sys.path.append(toppath)

        track_param = configparser.ConfigParser()
        if not track_param.read(toppath + "/params/driving_task.ini"):
            raise ValueError('Specified online parameter config file does not exist or is empty!')

        track_specifier = json.loads(track_param.get('DRIVING_TASK', 'track'))

        # define all relevant paths
        path_dict = {'globtraj_input_path': toppath + "/inputs/traj_ltpl_cl/traj_ltpl_cl_" + track_specifier + ".csv",
                    'graph_store_path': toppath + "/inputs/stored_graph.pckl",
                    'ltpl_offline_param_path': toppath + "/params/ltpl_config_offline.ini",
                    'ltpl_online_param_path': toppath + "/params/ltpl_config_online.ini",
                    'log_path': toppath + "/logs/graph_ltpl/",
                    'graph_log_id': datetime.datetime.now().strftime("%Y_%m_%d__%H_%M_%S")
                    }

        # ----------------------------------------------------------------------------------------------------------------------
        # INITIALIZATION AND OFFLINE PART --------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------
        # intialize graph_ltpl-class
        self.ltpl_obj = Graph_LTPL(path_dict=path_dict, visual_mode=False, log_to_file=False)
        # calculate offline graph
        self.ltpl_obj.graph_init()
        # set start pose based on first point in provided reference-line
        self.refline = graph_ltpl.imp_global_traj.src.\
            import_globtraj_csv.import_globtraj_csv(import_path=path_dict['globtraj_input_path'])[0]

        self.pos_est = self.refline[0, :]
        self.heading_est = np.arctan2(np.diff(self.refline[0:2, 1]), np.diff(self.refline[0:2, 0])) - np.pi / 2
        self.vel_est = 0.0

        # set start pos
        self.ltpl_obj.set_startpos(pos_est=self.pos_est,
                            heading_est=self.heading_est)

        # ----------------------------------------------------------------------------------------------------------------------
        # ONLINE LOOP ----------------------------------------------------------------------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------------
        self.traj_set = {'straight': None}
        self.obj_list = []
        tic = time.time()

    def yaw_from_ros_quaternion(self, quat):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (quat.w * quat.x + quat.y * quat.z)
        t1 = +1.0 - 2.0 * (quat.x * quat.x + quat.y * quat.y)
        roll_x = math.atan2(t0, t1)
        return roll_x # in radians

    def veh_odom_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        self.current_veh_odom = msg
        self.pos_est = np.ndarray([self.current_veh_odom.pose.pose.position.x,
                                    self.current_veh_odom.pose.pose.position.y])
        self.vel_est = math.sqrt(pow(self.current_veh_odom.twist.twist.linear.x, 2)
                                + pow(self.current_veh_odom.twist.twist.linear.y, 2)
                                + pow(self.current_veh_odom.twist.twist.linear.z, 2))

    def perception_result_callback(self, msg):

        self.obj_list.clear()

        # TODO : From perception result, heading and velocity information should be added or extracted.
        # TODO : Check whether the heading is in global or body coordinate (Most likely, velocity is global but check once more)
        for perception_result in msg.perception_list:
            template_dict = {'X': perception_result.detection_result_3d.center.position.x,
                             'Y': perception_result.detection_result_3d.center.position.y,
                             'theta': yaw_from_ros_quaternion(perception_result.detection_result_3d.center.orientation),
                             'type': 'physical', 'id': perception_result.id, 'length': 5.0,
                             'v': perception_result.tracking_result_velocity_body_x_mps + self.vel_est}

            self.obj_list.append(template_dict)

    def timer_callback(self):

        # -- SELECT ONE OF THE PROVIDED TRAJECTORIES -----------------------------------------------------------------------
        # (here: brute-force, replace by sophisticated behavior planner)
        for sel_action_prev in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
            if sel_action_prev in self.traj_set.keys():
                break

        # -- CALCULATE PATHS FOR NEXT TIMESTAMP ----------------------------------------------------------------------------
        self.ltpl_obj.calc_paths(prev_action_id=sel_action_prev,
                            object_list=self.obj_list)

        self.traj_set = self.ltpl_obj.calc_vel_profile(pos_est=self.pos_est,
                                                        vel_est=self.vel_est)[0]

        for sel_action_current in ["right", "left", "straight", "follow"]:  # try to force 'right', else try next in list
            if sel_action_current in self.traj_set.keys():
                break

        maptrack_inglobal = self.traj_set.get(sel_action_current)

        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for idx in range(len(maptrack_inglobal[0])):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = maptrack_inglobal[0][idx][1] # for x
            pose.pose.position.y = maptrack_inglobal[0][idx][2] # for x
            print(pose.pose.position.x,pose.pose.position.y)
            msg.poses.append(pose)

        self.local_maptrack_inglobal_pub.publish(msg)

                                    
def main(args=None):
    rclpy.init(args=args)
    graph_based_planner_node = GraphBasedPlanner()
    rclpy.spin(graph_based_planner_node)
    graph_based_planner_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
