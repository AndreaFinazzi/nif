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

LocalizationNode::LocalizationNode(
    const std::string& node_name_,
    const std::shared_ptr<LocalizationMinimal> localization_algorithm_ptr)

  : IBaseNode(node_name_, common::NodeType::LOCALIZATION),
    m_localization_algorithm_ptr(localization_algorithm_ptr),
    m_use_enu(false),
    tf_broadcaster(std::make_shared<tf2_ros::TransformBroadcaster>(this))

{

//  this->declare_parameter("ecef_ref_lat", 0.0 );
//  this->declare_parameter("ecef_ref_lon", 0.0 );
//  this->declare_parameter("ecef_ref_hgt", 0.0 );
//  this->declare_parameter("use_enu", true);

//  double ecef_ref_lat, ecef_ref_lon, ecef_ref_hgt;
////  If cannot get parameters, throw exception
//  if (!(this->get_parameter("ecef_ref_lat", ecef_ref_lat) &&
//        this->get_parameter("ecef_ref_lon", ecef_ref_lon) &&
//        this->get_parameter("ecef_ref_hgt", ecef_ref_hgt) &&
//        this->get_parameter("use_enu", m_use_enu)
//        ))
//  {
//    RCLCPP_ERROR(this->get_logger(), "Can't get parameters during initialization, cannot proceed.");
//    throw rclcpp::exceptions::InvalidParametersException("LocalizationNode: get_parameter returned false.");
//  }

//TODO this mechanism should be moved to the interface
//Not catching on purpose, we want the node to fail initialization if he can't get the parameters
  auto ecef_ref_lat = this->get_global_parameter<double>("coordinates.ecef_ref_lat");
  auto ecef_ref_lon = this->get_global_parameter<double>("coordinates.ecef_ref_lon");
  auto ecef_ref_hgt = this->get_global_parameter<double>("coordinates.ecef_ref_hgt");
  m_use_enu = this->get_global_parameter<bool>("coordinates.use_enu");

  std::vector<double> ecef_ref_param = {ecef_ref_lat, ecef_ref_lon, ecef_ref_hgt};

  localization_algorithm_ptr->setENUReference(
      ecef_ref_param.at(0), ecef_ref_param.at(1), ecef_ref_param.at(2));

  RCLCPP_INFO(this->get_logger(), "ECEF reference set to lat: %f, lon: %f, height: %f",
              ecef_ref_param.at(0), ecef_ref_param.at(1), ecef_ref_param.at(2));
  RCLCPP_INFO(this->get_logger(), m_use_enu ? "Using ENU" : "Using NED");

  this->m_veh_odom.header.frame_id = this->getGlobalFrameId();
  this->m_veh_odom.child_frame_id  = this->getBodyFrameId();

  m_timer = this->create_wall_timer(
      10ms, std::bind(&LocalizationNode::timerCallback, this));

  m_tf_timer = this->create_wall_timer(
      10ms, std::bind(&LocalizationNode::publishTransformStamped, this));

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
  m_gps_sync.registerCallback(std::bind(&LocalizationNode::syncGPSCallback,
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

void LocalizationNode::timerCallback() {
  RCLCPP_DEBUG(this->get_logger(),
               "LocalizationNode odom update timer callback");
  m_veh_odom_publisher->publish(m_veh_odom);
//  this->publishTransformStamped();
}

void LocalizationNode::gpsHorizontalCallback(
        const novatel_gps_msgs::msg::Inspva::SharedPtr gps_horizontal_ptr_)
{
    this->m_localization_algorithm_ptr->setGPSHorizontalData(
            *gps_horizontal_ptr_);

    this->m_veh_odom = this->m_localization_algorithm_ptr->getVehOdomByHorizontalGPS();
    this->m_veh_odom.header.frame_id = this->getGlobalFrameId();
    this->m_veh_odom.child_frame_id = this->getBodyFrameId();


void LocalizationNode::getENUfromNED(nav_msgs::msg::Odometry &ned_odom,
                                     const double &ned_yaw_rad) {
  double temp;
  temp = ned_odom.pose.pose.position.x;
  ned_odom.pose.pose.position.x = ned_odom.pose.pose.position.y;
  ned_odom.pose.pose.position.y = temp;
  double yaw = (nif::common::constants::numeric::PI / 2.0 - ned_yaw_rad);
  tf2::Quaternion vehicle_quat;
  vehicle_quat.setRPY(0.0, 0.0, yaw);
  vehicle_quat = vehicle_quat.normalize();
  ned_odom.pose.pose.orientation.x = vehicle_quat.x();
  ned_odom.pose.pose.orientation.y = vehicle_quat.y();
  ned_odom.pose.pose.orientation.z = vehicle_quat.z();
  ned_odom.pose.pose.orientation.w = vehicle_quat.w();
}

void LocalizationNode::publishTransformStamped() {
    static tf2_ros::TransformBroadcaster transform_broadcaster(this);

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

//    this->tf_broadcaster->sendTransform(this->transform_stamped);
    transform_broadcaster.sendTransform(this->transform_stamped);
}