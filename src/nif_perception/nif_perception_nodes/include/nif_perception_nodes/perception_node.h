//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_PERCEPTION_NODE_H
#define ROS2MASTER_PERCEPTION_NODE_H


#include "rclcpp/rclcpp.hpp"
#include "nif_common_nodes/base_node.h"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

namespace nif {
    namespace perception {

        class PerceptionNode : public nif::common::IBaseNode {

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


            GRaceLineManager g_race_line_manager;
            PerceptionNeuralModel neural_model;
            SensorFusionManager sensor_fusion_manager;
            TrackingManager tracking_manager;


             message_filters::TimeSynchronizer<
                sensor_msgs::msg::Image,
                sensor_msgs::msg::Image,
                sensor_msgs::msg::Image,
                sensor_msgs::msg::Image,
                sensor_msgs::msg::Image,
                sensor_msgs::msg::Image,
                sensor_msgs::msg::PointCloud2,
                delphi_esr_msgs::msg::EsrTrack> sensors_message_filter;
//             sensors_message_filter(camera_NODENAME_sub x 6, lidar_mesh_sub, radar_mesh_sub, 3);
//             sync_.registerCallback(std::bind(&sensorMessageFilterCallback, this, std::placeholders::_1, std::placeholders::_2));

             ROS2Sub<Image> x 6 camera_NODENAME_sub;
             ROS2Sub<Pointcloud2> lidar_mesh_sub;
             ROS2Sub<RadarOutput> radar_mesh_sub;
             ROS2Sub<V2XMessageType> v2x_sub;
             ROS2Sub<Collection<OppoVehicleState>> opponents_state_pub;


            void sensorMessageFilterCallback(
                    const sensor_msgs::msg::Image::SharedPtr & msg_camera_0,
                    const sensor_msgs::msg::Image::SharedPtr & msg_camera_1,
                    const sensor_msgs::msg::Image::SharedPtr & msg_camera_2,
                    const sensor_msgs::msg::Image::SharedPtr & msg_camera_3,
                    const sensor_msgs::msg::Image::SharedPtr & msg_camera_4,
                    const sensor_msgs::msg::Image::SharedPtr & msg_camera_5,
                    const sensor_msgs::msg::PointCloud2::SharedPtr & msg_lidar_mesh,
                    const delphi_esr_msgs::msg::EsrTrack::SharedPtr & msg_radar_mesh
                    );

//            + v2XCallback(...): void
        };

    }
}


#endif //ROS2MASTER_PERCEPTION_NODE_H
