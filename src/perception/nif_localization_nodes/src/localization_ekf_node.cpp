﻿//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 09/02/21.
//

#include "localization_ekf_nodes/localization_ekf_node.h"
#include "nif_frame_id/frame_id.h"

using namespace nif::localization::ekf;
using namespace nif::localization::utils;
using namespace nif::common::frame_id::localization;

EKFLocalizer::EKFLocalizer(const std::string &node_name) : IBaseNode(node_name) {
  this->declare_parameter<double>("origin_lat", double(39.809786));
  this->declare_parameter<double>("origin_lon", double(-86.235148));

  m_origin_lat = this->get_global_parameter<double>("coordinates.ecef_ref_lat");
  m_origin_lon = this->get_global_parameter<double>("coordinates.ecef_ref_lon");
  const std::string& topic_ego_odometry =
      this->get_global_parameter<std::string>(
          nif::common::constants::parameters::names::TOPIC_ID_EGO_ODOMETRY);

  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();

  sub_gpslatlon = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
      "in_bestpos", qos,
      std::bind(&EKFLocalizer::GPSLATLONCallback, this, std::placeholders::_1));
  subINSPVA = this->create_subscription<novatel_oem7_msgs::msg::INSPVA>(
      "in_inspva", qos,
      std::bind(&EKFLocalizer::GPSINSPVACallback, this, std::placeholders::_1));

  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  sub_filtered_IMU.subscribe(this,
                             "in_imu", rmw_qos_profile);
  sub_filtered_Wheel.subscribe(this, "in_wheel_speed_report",
                               rmw_qos_profile);

  m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicyT>>(
      SyncPolicyT(10), sub_filtered_IMU, sub_filtered_Wheel);

  m_sync->registerCallback(std::bind(&EKFLocalizer::MessegefilteringCallback,
                                     this, std::placeholders::_1,
                                     std::placeholders::_2));

  pub_EKF_odometry =
      this->create_publisher<nav_msgs::msg::Odometry>(
          "out_odometry_ekf_estimated", qos);
  pub_bestpos_odometry =
      this->create_publisher<nav_msgs::msg::Odometry>(
          "out_odometry_bestpos", qos);

  broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  using namespace std::chrono_literals; // NOLINT
  timer_ = this->create_wall_timer(
      10ms, std::bind(&EKFLocalizer::timer_callback, this));

  // Set the ltp reference point
  nif::localization::utils::GeodeticConverter::GeoRef ref;
  ref.latitude = m_origin_lat;
  ref.longitude = m_origin_lon;
  ref.altitude = 0.;
  conv_.initializeReference(ref);

  RCLCPP_INFO(this->get_logger(), "START EKF NODE");
}

EKFLocalizer ::~EKFLocalizer(){};

void EKFLocalizer::respond() {
  this->get_parameter("origin_lat", m_origin_lat);
  this->get_parameter("origin_lon", m_origin_lon);
}

void EKFLocalizer::timer_callback() {
  run();
}

void EKFLocalizer::run() {

  if (bImuFirstCall && bGPS && bGPSHeading) {
    vel_and_yawRate = cv::Mat::zeros(3, 1, CV_64FC1);
    GPS_data = cv::Mat::zeros(3, 1, CV_64FC1);
    vel_and_yawRate.ptr<double>(0)[0] = m_dVelolcity_X; // vel_x
    vel_and_yawRate.ptr<double>(1)[0] = 0.0;            // vel_y
    vel_and_yawRate.ptr<double>(2)[0] = m_dIMU_yaw_rate;
    GPS_data.ptr<double>(0)[0] = m_dGPS_X;       // odom
    GPS_data.ptr<double>(1)[0] = m_dGPS_Y;       // odom
    GPS_data.ptr<double>(2)[0] = m_dGPS_Heading; // odom

    if (ImuTimeDouble != ImuPrevTimeDouble) {

      m_ekf.EKF_Predictionstep(m_ekf.m_xhat, m_ekf.m_Phat, vel_and_yawRate,
                               0.01);

      int quality = 4;
      if (gps_flag == true) {
        m_ekf.EKF_Correctionstep(m_ekf.m_xhat, m_ekf.m_Phat, quality, true,
                                 true,
                                 GPS_data); // check //true
      }

      nav_msgs::msg::Odometry ekf_odom_msg;
      ekf_odom_msg.header.stamp = this->now();
      ekf_odom_msg.header.frame_id = ODOM;
      ekf_odom_msg.child_frame_id = BASE_LINK;
      ekf_odom_msg.pose.pose.position.x = m_ekf.m_xhat.ptr<double>(0)[0];
      ekf_odom_msg.pose.pose.position.y = m_ekf.m_xhat.ptr<double>(1)[0];
      ekf_odom_msg.pose.pose.position.z = 0.0;
      tf2::Quaternion quat_ekf;
      geometry_msgs::msg::Quaternion quat_ekf_msg;
      double angle_z_rad = m_ekf.m_xhat.ptr<double>(2)[0];
      // std::cout << angle_z_rad * 180 / M_PI << std::endl;

      if (std::isnan(angle_z_rad)) {
        angle_z_rad = 0;
      }
      quat_ekf.setRPY(0, 0, angle_z_rad);
      quat_ekf.normalize();
      quat_ekf_msg = tf2::toMsg(quat_ekf);
      ekf_odom_msg.pose.pose.orientation = quat_ekf_msg;
      pub_EKF_odometry->publish(ekf_odom_msg);

      geometry_msgs::msg::TransformStamped nav_base_tf{};
      nav_base_tf.transform.translation.x = ekf_odom_msg.pose.pose.position.x;
      nav_base_tf.transform.translation.y = ekf_odom_msg.pose.pose.position.y;
      nav_base_tf.transform.translation.z = ekf_odom_msg.pose.pose.position.z;
      nav_base_tf.transform.rotation.w = ekf_odom_msg.pose.pose.orientation.w;
      nav_base_tf.transform.rotation.x = ekf_odom_msg.pose.pose.orientation.x;
      nav_base_tf.transform.rotation.y = ekf_odom_msg.pose.pose.orientation.y;
      nav_base_tf.transform.rotation.z = ekf_odom_msg.pose.pose.orientation.z;
      nav_base_tf.header.stamp = this->now();
      nav_base_tf.header.frame_id = ODOM;
      nav_base_tf.child_frame_id = BASE_LINK;
      broadcaster_->sendTransform(nav_base_tf);

      gps_flag = false;

      ImuPrevTimeDouble = ImuTimeDouble;
      VehVelocityPrevTimeDouble = VehVelocityTimeDouble;
    } else {
      // RCLCPP_WARN(this->get_logger(), "[Imu / Wheel speed] is not updated");
      return;
    }

  } else {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for -[/novatel_bottom/bestpos]");
    RCLCPP_DEBUG(this->get_logger(), "            -[/novatel_bottom/inspva]");
    RCLCPP_DEBUG(this->get_logger(), "            -[/novatel_bottom/imu/data]");
    RCLCPP_DEBUG(this->get_logger(),
                "            -[/raptor_dbw_interface/wheel_speed_report]");
  }
}
void EKFLocalizer::GPSLATLONCallback(
    const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg) {
  nif::localization::utils::GeodeticConverter::GeoRef currentGPS;
  currentGPS.latitude = (double)msg->lat;
  currentGPS.longitude = (double)msg->lon;
  // Currently ignore altitude for the most part and just track x/y
  currentGPS.altitude = 0.;

  nif::localization::utils::GeodeticConverter::CartesianPoint ltp_pt;
  conv_.geodetic2Ned(currentGPS, ltp_pt);

  nav_msgs::msg::Odometry ltp_odom;

  ltp_odom.header.frame_id = "odom";
  ltp_odom.header.stamp = this->now();
  ltp_odom.child_frame_id = BASE_LINK;

  m_dGPS_X = ltp_pt.x;
  m_dGPS_Y = -ltp_pt.y;

  ltp_odom.pose.pose.position.x = m_dGPS_X;
  // We convert from NED to FLU
  ltp_odom.pose.pose.position.y = m_dGPS_Y;
  ltp_odom.pose.pose.position.z = -ltp_pt.z;

  ltp_odom.header = msg->header;
  ltp_odom.pose.pose.orientation.w = 1;
  ltp_odom.pose.pose.position.x = m_dGPS_X;
  ltp_odom.pose.pose.position.y = m_dGPS_Y;
  ltp_odom.pose.pose.position.z = 0;

  tf2::Quaternion quat_ekf;
  geometry_msgs::msg::Quaternion quat_ekf_msg;
  quat_ekf.setRPY(m_dGPS_roll, 0, m_dGPS_Heading);
  quat_ekf.normalize();
  quat_ekf_msg = tf2::toMsg(quat_ekf);
  ltp_odom.pose.pose.orientation = quat_ekf_msg;

  pub_bestpos_odometry->publish(ltp_odom);

  gps_flag = true;
  bGPS = true;
  ////////////////////////////////////////////////////////////////////////////////////
}

void EKFLocalizer::GPSINSPVACallback(
    const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg) {
  double yaw = (-msg->azimuth) * nif::common::constants::DEG2RAD;
  m_dGPS_Heading = yaw;
  m_dGPS_roll = msg->roll * nif::common::constants::DEG2RAD;
  bGPSHeading = true;
}

void EKFLocalizer::MessegefilteringCallback(
    const sensor_msgs::msg::Imu ::ConstSharedPtr &imu_msg,
    const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr
        &wheel_speed_msg) {

  m_dIMU_yaw_rate = imu_msg->angular_velocity.z;
  m_dVelolcity_X =
      (wheel_speed_msg->front_right + wheel_speed_msg->front_left) / 2 *
      nif::common::constants::KPH2MS;

  auto ImuCurrentTime = rclcpp::Time(imu_msg->header.stamp);
  if (!bImuFirstCall) {
    bImuFirstCall = true;
    ImuPrevTimeDouble =
        static_cast<double>(ImuCurrentTime.seconds()) +
        static_cast<double>(ImuCurrentTime.nanoseconds()) * 1e-9;
    return;
  }
  ImuTimeDouble = static_cast<double>(ImuCurrentTime.seconds()) +
                  static_cast<double>(ImuCurrentTime.nanoseconds()) * 1e-9;
};