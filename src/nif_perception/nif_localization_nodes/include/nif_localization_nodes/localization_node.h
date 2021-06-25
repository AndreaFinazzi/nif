//
// Created by usrg on 6/23/21.
//

#ifndef ROS2MASTER_LOCALIZATION_NODE_H
#define ROS2MASTER_LOCALIZATION_NODE_H


#include "rclcpp/rclcpp.hpp"
#include "nif_common_nodes/base_node.h"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nif {
    namespace perception {

        class LocalizationNode : public nif::common::IBaseNode {

        public:

        protected:

        private:

            GRaceLineManager g_race_line_manager;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_data_sub;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
            rclcpp::Subscription<sensor_msgs::msg::Pointcloud2>::SharedPtr lidar_mesh_sub;
            rclcpp::Subscription<Polynomial>::SharedPtr track_boundaries_sub;


            void gpsDataCallback(nav_msgs::msg::Odometry::SharedPtr & msg);
            void lidarMeshCallback(sensor_msgs::msg::PointCloud2 & msg);

//            TODO: Polynomial type yet to be defined.
            void trackBoundariesCallback(Polynomial::SharedPtr & msg);

        };

    }
}

#endif //ROS2MASTER_LOCALIZATION_NODE_H
