//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#include "nif_localization_nodes/localization_node.h"

using namespace nif::perception;

LocalizationNode::LocalizationNode(std::string& node_name_)
  : LocalizationNode(node_name_, std::make_shared<LocalizationMinimal>()) {}

LocalizationNode::LocalizationNode(
    std::string& node_name_,
    std::shared_ptr<LocalizationMinimal> localization_algorithm_ptr)
  : m_localization_algorithm_ptr(localization_algorithm_ptr),
    IBaseNode(node_name_) {
  m_timer = this->create_wall_timer(
      10ms, std::bind(&LocalizationNode::timer_callback, this));

  m_veh_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(
      "nif/localization/veh_odom", 10);

  m_gps_horizontal_subscriber.subscribe(this, "/novatel/inspva1");
  m_gps_vertical_subscriber.subscribe(this, "/novatel/inspva2");

  m_gps_sync_ptr = std::make_shared<
      message_filters::TimeSynchronizer<novatel_gps_msgs::msg::Inspva,
                                        novatel_gps_msgs::msg::Inspva>>(
      m_gps_horizontal_subscriber, m_gps_vertical_subscriber, 10);

  m_gps_sync_ptr->registerCallback(std::bind(&LocalizationNode::syncGPSCallback,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2));
}

void LocalizationNode::syncGPSCallback(
    const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& gps_horizontal_ptr_,
    const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& gps_vertical_ptr_) {
  RCLCPP_INFO(this->get_logger(),
              "Synchronized the following gps timestamps: %u, %u",
              gps_horizontal_ptr_->header.stamp.sec,
              gps_vertical_ptr_->header.stamp.sec);

  this->m_localization_algorithm_ptr->setGPSHorizontalData(
      *gps_horizontal_ptr_);
  this->m_localization_algorithm_ptr->setGPSVerticalData(*gps_vertical_ptr_);
  m_veh_odom = this->m_localization_algorithm_ptr->getVehOdomByFusion();
}

void LocalizationNode::timer_callback() {
  // RCLCPP_INFO(this->get_logger(),
  //             "LocalizationNode odom update timer callback");
  m_veh_odom_publisher->publish(m_veh_odom);
}