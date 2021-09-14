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
    : IBaseNode(node_name_) {
  this->declare_parameter<double>("thres_for_distance_error_flag", double(2.0));
  this->declare_parameter<double>("thres_for_distance_to_wall", double(2.0));
  this->declare_parameter<int>("geofence_timeout_ms", 1000);

  pubOuterError = this->create_publisher<std_msgs::msg::Float32>(
      "/error_outer_distance", nif::common::constants::QOS_EGO_ODOMETRY);
  pubOuterErrorFlag = this->create_publisher<std_msgs::msg::Bool>(
      "/Bool/outer_distance_error_high",
      nif::common::constants::QOS_EGO_ODOMETRY);
  pubTooCloseToWallFlag = this->create_publisher<std_msgs::msg::Bool>(
      "/Bool/close_to_wall",
      nif::common::constants::QOS_EGO_ODOMETRY);

  subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      "/out_odometry_ekf_estimated", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&ResilientLocalization::EKFOdometryCallback, this,
                std::placeholders::_1));

  subOnTheTrack = this->create_subscription<std_msgs::msg::Bool>(
      "/Bool/on_the_track", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&ResilientLocalization::OnTheTrackCallback, this,
                std::placeholders::_1));

  subInnerDistance = this->create_subscription<std_msgs::msg::Float32>(
      "/geofence_inner_distance", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&ResilientLocalization::InnerGeofenceDistanceCallback, this,
                std::placeholders::_1));
  subOuterDistance = this->create_subscription<std_msgs::msg::Float32>(
      "/geofence_outer_distance", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&ResilientLocalization::OuterGeofenceDistanceCallback, this,
                std::placeholders::_1));

  subInnerWallDetection = this->create_subscription<std_msgs::msg::Float32>(
      "/detected_inner_distance", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&ResilientLocalization::InnerDetectedDistanceCallback, this,
                std::placeholders::_1));
  subOuterWallDetection = this->create_subscription<std_msgs::msg::Float32>(
      "/detected_outer_distance", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&ResilientLocalization::OuterDetectedDistanceCallback, this,
                std::placeholders::_1));

  respond();

  using namespace std::chrono_literals; // NOLINT
  timer_ = this->create_wall_timer(10ms, [this]() {
    if (bDetectedOuter && bGeofenceOuter)
    {
      // If the geofence data is too old, report error, but keep going.
      if (this->now() - m_geofence_outer_last_update_ >=
              this->m_geofence_timeout_ ||
          this->now() - m_detected_outer_last_update_ >=
              this->m_geofence_timeout_) {
        
        std::cout << "A :" << (this->now() - m_detected_outer_last_update_).nanoseconds() << std::endl;
        std::cout << "B :" << this->m_geofence_timeout_.nanoseconds() << std::endl;
        std::uint8_t tmp = 123;
        this->setNodeStatus((nif::common::NodeStatusCode) tmp);
      }

      double current = static_cast<double>(this->now().seconds()) +
                       static_cast<double>(this->now().nanoseconds()) * 1e-9;
      double geofence_time =
          static_cast<double>(m_detected_outer_last_update_.seconds()) +
          static_cast<double>(m_detected_outer_last_update_.nanoseconds()) *
              1e-9;


      double m_outer_error =
          m_geofence_outer_distance - m_detected_outer_distance;
      std_msgs::msg::Float32 OuterDistanceMsg;
      OuterDistanceMsg.data = m_outer_error;

      std_msgs::msg::Bool OuterDistanceHighErrorFlagMsg;
      OuterDistanceHighErrorFlagMsg.data = false;

      if (fabs(m_outer_error) > m_ThresForDistanceErrorFlag &&
          m_detected_outer_distance != 0. && m_on_the_track) {
        OuterDistanceHighErrorFlagMsg.data = true;
    }
    pubOuterError->publish(OuterDistanceMsg);
    pubOuterErrorFlag->publish(OuterDistanceHighErrorFlagMsg);

    std_msgs::msg::Bool TooCloseToWallMsg;
    TooCloseToWallMsg.data = false;
    if (fabs(m_detected_outer_distance) < m_ThresToWallDistance &&
        m_detected_outer_distance != 0. && m_on_the_track) {
      TooCloseToWallMsg.data = true;
    }
    pubTooCloseToWallFlag->publish(TooCloseToWallMsg);
    this->setNodeStatus(common::NODE_OK);
    }
  });

  this->setNodeStatus(common::NODE_INITIALIZED);
}

ResilientLocalization::~ResilientLocalization() {}

void ResilientLocalization::respond() {
    int geofence_timeout_ms = 1000;
  this->get_parameter("thres_for_distance_error_flag", m_ThresForDistanceErrorFlag);
  this->get_parameter("thres_for_distance_to_wall", m_ThresToWallDistance);
  this->get_parameter("geofence_timeout_ms", geofence_timeout_ms);

  m_geofence_timeout_ = rclcpp::Duration(geofence_timeout_ms * 1000000);

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
  m_geofence_inner_last_update_ = this->now();


}
void ResilientLocalization::OuterGeofenceDistanceCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_geofence_outer_distance = msg->data;
  m_geofence_outer_last_update_ = this->now();

  bGeofenceOuter = true;
}
void ResilientLocalization::InnerDetectedDistanceCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_detected_inner_distance = msg->data;
  m_detected_inner_last_update_ = this->now();
}
void ResilientLocalization::OuterDetectedDistanceCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_detected_outer_distance = msg->data;
  m_detected_outer_last_update_ = this->now();

  bDetectedOuter = true;
}

void ResilientLocalization::OnTheTrackCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  m_on_the_track = msg->data;
}