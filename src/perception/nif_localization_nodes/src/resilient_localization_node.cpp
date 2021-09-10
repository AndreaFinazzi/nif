//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 09/08/21.
//
#include "localization_ekf_nodes/resilient_localization_node.h"
#include "nif_frame_id/frame_id.h"

using namespace nif::localization::resilient;
using namespace nif::common::frame_id::localization;

// Constructor
ResilientLocalization::ResilientLocalization(const std::string &node_name_)
    : Node(node_name_) {
  this->declare_parameter<double>("thres_for_distance_error_flag", double(2.0));

  pubOuterError = this->create_publisher<std_msgs::msg::Float32>(
      "/error_outer_distance", nif::common::constants::QOS_SENSOR_DATA);
  pubOuterErrorFlag = this->create_publisher<std_msgs::msg::Bool>(
      "/Bool/outer_distance_error_high", nif::common::constants::QOS_SENSOR_DATA);

  subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry/ekf_estimated", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&ResilientLocalization::EKFOdometryCallback, this,
                std::placeholders::_1));

  subInnerDistance = this->create_subscription<std_msgs::msg::Float32>(
      "/geofence_inner_distance", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&ResilientLocalization::InnerGeofenceDistanceCallback, this,
                std::placeholders::_1));
  subOuterDistance = this->create_subscription<std_msgs::msg::Float32>(
      "/geofence_outer_distance", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&ResilientLocalization::OuterGeofenceDistanceCallback, this,
                std::placeholders::_1));

  subInnerWallDetection = this->create_subscription<std_msgs::msg::Float32>(
      "/detected_inner_distance", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&ResilientLocalization::InnerDetectedDistanceCallback, this,
                std::placeholders::_1));
  subOuterWallDetection = this->create_subscription<std_msgs::msg::Float32>(
      "/detected_outer_distance", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&ResilientLocalization::OuterDetectedDistanceCallback, this,
                std::placeholders::_1));

  respond();

  using namespace std::chrono_literals; // NOLINT
  timer_ = this->create_wall_timer(10ms, [this]() {
    
    double m_outer_error = m_geofence_outer_distance - m_detected_outer_distance;
    std_msgs::msg::Float32 OuterDistanceMsg;
    OuterDistanceMsg.data = m_outer_error;

    std_msgs::msg::Bool OuterDistanceHighErrorFlagMsg;
    OuterDistanceHighErrorFlagMsg.data = false; 

    if(fabs(m_outer_error) > m_ThresForDistanceErrorFlag && m_detected_outer_distance != 0.)
    {
      OuterDistanceHighErrorFlagMsg.data =  true;
    } 
    pubOuterError->publish(OuterDistanceMsg);
    pubOuterErrorFlag->publish(OuterDistanceHighErrorFlagMsg);
  });
}

ResilientLocalization::~ResilientLocalization() {}

void ResilientLocalization::respond() {
  this->get_parameter("thres_for_distance_error_flag", m_ThresForDistanceErrorFlag);
  // this->get_parameter("inner_geofence_filename", m_InnerGeoFenceFileName);
}

void ResilientLocalization::EKFOdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  m_veh_x = msg->pose.pose.position.x;
  m_veh_y = msg->pose.pose.position.y;

  tf2::Quaternion tf_quat;
  tf2::convert(msg->pose.pose.orientation, tf_quat);
  tf2::Matrix3x3 mat(tf_quat);
  mat.getRPY(m_veh_roll, m_veh_pitch, m_veh_yaw);
}

void ResilientLocalization::InnerGeofenceDistanceCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_geofence_inner_distance = msg->data;
}
void ResilientLocalization::OuterGeofenceDistanceCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_geofence_outer_distance = msg->data;
}
void ResilientLocalization::InnerDetectedDistanceCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_detected_inner_distance = msg->data;
}
void ResilientLocalization::OuterDetectedDistanceCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_detected_outer_distance = msg->data;
}
