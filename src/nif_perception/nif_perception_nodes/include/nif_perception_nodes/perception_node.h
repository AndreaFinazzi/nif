//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_PERCEPTION_NODE_H
#define ROS2MASTER_PERCEPTION_NODE_H


#include "nif_common_nodes/i_base_node.h"
#include "nif_common/types.h"
#include "nif_tracking/tracking_manager.h"
#include "nif_sensor_fusion/sensor_fusion_manager.h"
#include "nif_racing_line/racing_line_manager.h"

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "delphi_esr_msgs/msg/esr_track.hpp"


namespace nif {
    namespace perception {

        class PerceptionNode : public nif::common::IBaseNode {

        public:

        protected:

        private:
            nif::common::RacingLineManager racing_line_manager;

            // TODO: Define precise type for this
//            PerceptionNeuralModel neural_model;

            nif::perception::SensorFusionManager sensor_fusion_manager;
            nif::perception::TrackingManager tracking_manager;


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

//          TODO : Define a better way to do this. Hardcoding the camera subs significantly reduces reusability.
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_front_left_sub;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_front_left_center_sub;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_front_right_center_sub;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_front_right_sub;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_rear_right_sub;
            rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_rear_left_sub;


            rclcpp::Subscription<delphi_esr_msgs::msg::EsrTrack> radar_mesh_sub;

//            TODO: still to decide whether to use v2x communication or not.
//            rclcpp::Subscription<V2XMessageType> v2x_sub;

             rclcpp::Subscription<nif::common::types::t_oppo_collection_states> opponents_state_pub;


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
