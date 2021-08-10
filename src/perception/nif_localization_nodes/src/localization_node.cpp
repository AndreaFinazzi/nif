//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#include "nif_localization_nodes/localization_node.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <utility>

using namespace nif::perception;
using namespace message_filters;
using namespace std::placeholders;

typedef sync_policies::ApproximateTime<novatel_gps_msgs::msg::Inspva,
                                       novatel_gps_msgs::msg::Inspva>
    MySyncPolicy;

LocalizationNode::LocalizationNode(const std::string& node_name_)
  : LocalizationNode(node_name_, std::make_shared<LocalizationMinimal>()) {}

LocalizationNode::LocalizationNode(
    const std::string& node_name_,
    const std::shared_ptr<LocalizationMinimal> localization_algorithm_ptr)
  : IBaseNode(node_name_),
    m_localization_algorithm_ptr(std::move(localization_algorithm_ptr))
    {

    this->m_veh_odom.header.frame_id = this->getBodyFrameId();
    this->m_veh_odom.child_frame_id  = this->getGlobalFrameId();


  m_timer = this->create_wall_timer(
      10ms, std::bind(&LocalizationNode::timer_callback, this));

  m_veh_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(
      "localization/ego_odom", 10);

  m_gps_horizontal_subscriber.subscribe(this, "gnss_01",
                                        rclcpp::QoS(10).get_rmw_qos_profile());
  m_gps_vertical_subscriber.subscribe(this, "gnss_02",
                                      rclcpp::QoS(10).get_rmw_qos_profile());

  m_gps_sync_ptr->registerCallback(
      boost::bind(&LocalizationNode::syncGPSCallback, this,
                  boost::placeholders::_1, boost::placeholders::_2));

  message_filters::TimeSynchronizer<novatel_gps_msgs::msg::Inspva,
                                    novatel_gps_msgs::msg::Inspva>
      m_gps_sync(m_gps_horizontal_subscriber, m_gps_vertical_subscriber, 10);

//  TODO REMEMBER that we may publish the same odom result in case we don't get data from the sensor
  m_gps_sync_ptr->registerCallback(std::bind(&LocalizationNode::syncGPSCallback,
                                             this,
                                             std::placeholders::_1,
                                             std::placeholders::_2));

  this->m_gps_horizontal_sub =
          this->create_subscription<novatel_gps_msgs::msg::Inspva>(
              "gnss_01",
              nif::common::constants::QOS_DEFAULT,
              std::bind(&LocalizationNode::gpsHorizontalCallback,
                        this,
                        std::placeholders::_1));
  // m_gps_sync_ptr =
  //     message_filters::TimeSynchronizer<novatel_gps_msgs::msg::Inspva,
  //                                       novatel_gps_msgs::msg::Inspva>(
  //         m_gps_horizontal_subscriber, m_gps_vertical_subscriber, 10);

  m_gps_sync.registerCallback(std::bind(&LocalizationNode::syncGPSCallback,
                                        this,
                                        std::placeholders::_1,
                                        std::placeholders::_2));

  // typedef message_filters::sync_policies::ApproximateTime<
  //     novatel_gps_msgs::msg::Inspva,
  //     novatel_gps_msgs::msg::Inspva>
  //     MySyncPolicy;
  // message_filters::Synchronizer<MySyncPolicy> img_sync(
  //     MySyncPolicy(10),
  //     *m_gps_horizontal_subscriber,
  //     *m_gps_vertical_subscriber);
  // // img_sync.setMaxIntervalDuration(rclcpp::Duration(3.0));
  // img_sync.registerCallback(&LocalizationNode::syncGPSCallback, this);
}

void LocalizationNode::syncGPSCallback(
    const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& gps_horizontal_ptr_,
    const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& gps_vertical_ptr_) {
  RCLCPP_DEBUG(this->get_logger(),
              "Synchronized the following gps timestamps: %u, %u",
              gps_horizontal_ptr_->header.stamp.sec,
              gps_vertical_ptr_->header.stamp.sec);

  this->m_localization_algorithm_ptr->setGPSHorizontalData(
      *gps_horizontal_ptr_);
  this->m_localization_algorithm_ptr->setGPSVerticalData(
      *gps_vertical_ptr_);
//  TODO reuse message, here we're COPYING (wasting time)
  this->m_veh_odom = this->m_localization_algorithm_ptr->getVehOdomByFusion();
  this->m_veh_odom.header.frame_id = this->getGlobalFrameId();
  this->m_veh_odom.child_frame_id = this->getBodyFrameId();
}

void LocalizationNode::timer_callback() {
  RCLCPP_DEBUG(this->get_logger(),
              "LocalizationNode odom update timer callback");
  m_veh_odom_publisher->publish(m_veh_odom);
}

void LocalizationNode::gpsHorizontalCallback(
        const novatel_gps_msgs::msg::Inspva::SharedPtr gps_horizontal_ptr_) {
    this->m_localization_algorithm_ptr->setGPSHorizontalData(
            *gps_horizontal_ptr_);

    this->m_veh_odom = this->m_localization_algorithm_ptr->getVehOdomByHorizontalGPS();
    this->m_veh_odom.header.frame_id = this->getGlobalFrameId();
    this->m_veh_odom.child_frame_id = this->getBodyFrameId();

    publishTransformStamped();
}

void LocalizationNode::publishTransformStamped() {
    static tf2_ros::TransformBroadcaster br(this);

    transform_stamped.header.stamp = this->now();
    transform_stamped.header.frame_id = this->getGlobalFrameId();
    transform_stamped.child_frame_id = this->getBodyFrameId();
    transform_stamped.transform.translation.x = m_veh_odom.pose.pose.position.x;
    transform_stamped.transform.translation.y = m_veh_odom.pose.pose.position.y;
    transform_stamped.transform.translation.z = m_veh_odom.pose.pose.position.z;
    tf2::Quaternion q;

    transform_stamped.transform.rotation.x = m_veh_odom.pose.pose.orientation.x;
    transform_stamped.transform.rotation.y = m_veh_odom.pose.pose.orientation.y;
    transform_stamped.transform.rotation.z = m_veh_odom.pose.pose.orientation.z;
    transform_stamped.transform.rotation.w = m_veh_odom.pose.pose.orientation.w;

    br.sendTransform(this->transform_stamped);
};