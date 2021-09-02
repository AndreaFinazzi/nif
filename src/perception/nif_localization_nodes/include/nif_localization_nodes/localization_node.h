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
#include "tf2/LinearMath/Quaternion.h"
#include "rcutils/error_handling.h"
#include <chrono>
#include <boost/bind.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

namespace nif {
namespace perception {
using namespace std::chrono_literals;

class LocalizationNode : public nif::common::IBaseNode {
public:
  LocalizationNode(
      const std::string &node_name_,
      const std::shared_ptr<LocalizationMinimal> localization_algorithm_ptr =
          std::make_shared<LocalizationMinimal>());

  ~LocalizationNode() {}

private:
  LocalizationNode();

  void timerCallback();
  void syncGPSCallback(
      const novatel_gps_msgs::msg::Inspva::ConstSharedPtr &gps_horizontal_,
      const novatel_gps_msgs::msg::Inspva::ConstSharedPtr &gps_vertical_);

  void gpsHorizontalCallback(
      const novatel_gps_msgs::msg::Inspva::SharedPtr gps_horizontal_ptr_);

  void getENUfromNED(nav_msgs::msg::Odometry &ned_odom,
                     const double &ned_yaw_ra);

  void publishTransformStamped();

  // TODO: not used function. @Andrea told that these functions should be
  // fixed or removed
  void initParameters() {}
  void getParameters() {}

  void topGPSCallback(const novatel_gps_msgs::msg::Inspva::SharedPtr &gps_);
  void bottomGPSCallback(const novatel_gps_msgs::msg::Inspva::SharedPtr &gps_);

  std::shared_ptr<LocalizationMinimal> m_localization_algorithm_ptr;
  nav_msgs::msg::Odometry m_veh_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_veh_odom_publisher;

  message_filters::Subscriber<novatel_gps_msgs::msg::Inspva>
      m_gps_horizontal_subscriber;
  message_filters::Subscriber<novatel_gps_msgs::msg::Inspva>
      m_gps_vertical_subscriber;

  std::shared_ptr<message_filters::TimeSynchronizer<
      novatel_gps_msgs::msg::Inspva, novatel_gps_msgs::msg::Inspva>>
      m_gps_sync_ptr;

  rclcpp::Subscription<novatel_gps_msgs::msg::Inspva>::SharedPtr
      m_gps_horizontal_sub;

  rclcpp::TimerBase::SharedPtr m_timer;

  bool m_use_enu;

  // Transform:
  geometry_msgs::msg::TransformStamped transform_stamped;

  rclcpp::TimerBase::SharedPtr m_tf_timer;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
};

} // namespace perception
} // namespace nif

#endif // ROS2MASTER_LOCALIZATION_NODE_H3
