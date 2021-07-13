//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#ifndef ROS2MASTER_LOCALIZATION_NODE_H
#define ROS2MASTER_LOCALIZATION_NODE_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_localization_minimal/localization_minimal.h"

#include "nav_msgs/msg/odometry.hpp"
#include "novatel_gps_msgs/msg/inspva.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rcutils/error_handling.h"
#include <chrono>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

namespace nif {
namespace perception {
using namespace std::chrono_literals;

class LocalizationNode : public nif::common::IBaseNode {
public:
  LocalizationNode(std::string& node_name_);
  LocalizationNode(
      std::string& node_name_,
      std::shared_ptr<LocalizationMinimal> localization_algorithm_ptr);
  ~LocalizationNode() {}

private:
  LocalizationNode();
  void timer_callback();
  void syncGPSCallback(
      const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& gps_horizontal_,
      const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& gps_vertical_);

  // TODO: not used function. @Andrea told that these functions should be
  // fixed or removed
  void initParameters() {}
  void getParameters() {}

  std::shared_ptr<LocalizationMinimal> m_localization_algorithm_ptr;
  nav_msgs::msg::Odometry m_veh_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_veh_odom_publisher;

  message_filters::Subscriber<novatel_gps_msgs::msg::Inspva>
      m_gps_horizontal_subscriber;
  message_filters::Subscriber<novatel_gps_msgs::msg::Inspva>
      m_gps_vertical_subscriber;
  std::shared_ptr<
      message_filters::TimeSynchronizer<novatel_gps_msgs::msg::Inspva,
                                        novatel_gps_msgs::msg::Inspva>>
      m_gps_sync_ptr;
  rclcpp::TimerBase::SharedPtr m_timer;
};

} // namespace perception
} // namespace nif

#endif // ROS2MASTER_LOCALIZATION_NODE_H
