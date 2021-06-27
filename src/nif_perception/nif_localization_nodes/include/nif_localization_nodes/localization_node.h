//
// Created by usrg on 6/23/21.
//

#ifndef ROS2MASTER_LOCALIZATION_NODE_H
#define ROS2MASTER_LOCALIZATION_NODE_H


#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_racing_line/racing_line_manager.h"

#include "rclcpp/rclcpp.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace nif {
    namespace perception {

        class LocalizationNode : public nif::common::IBaseNode {

        public:

        protected:

        private:

            nif::common::RacingLineManager racing_line_manager;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gps_data_sub;
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_mesh_sub;
            rclcpp::Subscription<nif::common::Polynomial>::SharedPtr track_boundaries_sub;

            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;

            void gpsDataCallback(nav_msgs::msg::Odometry::SharedPtr & msg);
            void lidarMeshCallback(sensor_msgs::msg::PointCloud2 & msg);

//            TODO: Polynomial type yet to be defined.
            void trackBoundariesCallback(nif::common::Polynomial::SharedPtr & msg);

        };

    }
}

#endif //ROS2MASTER_LOCALIZATION_NODE_H
