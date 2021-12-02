//
// Created by usrg on 30/11/21.
//

#ifndef IDM_BASED_ACC_NODE_H
#define IDM_BASED_ACC_NODE_H

#include <iostream>
#include "idm_acc_lib/c++/IDM.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include "nif_common_nodes/i_base_node.h"
#include "nif_msgs/msg/detected_object.hpp"
#include "nif_msgs/msg/detected_object_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace nif
{
    namespace control
    {
        class IDMACCNode : public nif::common::IBaseNode
        {

        public:
            explicit IDMACCNode(const std::string &node_name_);
            IDMACCNode(const std::string &node_name_,
                       const std::shared_ptr<IDM> idm_prt_);
            ~IDMACCNode();

            void detectionCallback(const nif_msgs::msg::DetectedObjectArray::SharedPtr det_msg);

        private:
            /* data */
            IDMACCNode();
            double m_acc_cmd;
            double m_veh_speed_mps;

            std::string m_config_file;

            nif_msgs::msg::DetectedObjectArray m_det_result;
            nav_msgs::msg::Odometry m_ego_odom;

            std::shared_ptr<IDM>
                m_idm_prt;

            // detection result subscriber
            rclcpp::Subscription<nif_msgs::msg::DetectedObjectArray>::SharedPtr
                m_detection_subscriber;
        };
    }
}

#endif // IDM_BASED_ACC_NODE_H
