//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 09/02/21.
// 

#ifndef EKF_LOCALIZER_NODE_H
#define EKF_LOCALIZER_NODE_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_localization_minimal/localization_minimal.h"

// inlcude ROS library
#include "rclcpp/clock.hpp"
#include "utils/geodetic_conv.h"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <novatel_oem7_msgs/msg/bestpos.hpp>
#include <novatel_oem7_msgs/msg/inspva.hpp>
#include <novatel_oem7_msgs/msg/bestvel.hpp>
#include <opencv2/opencv.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdio.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <time.h>

#include "localization_ekf_nodes/c_ekf.h"

namespace nif {
namespace localization {
namespace ekf {
class EKFLocalizer : public nif::common::IBaseNode {
public:
  EKFLocalizer(const std::string &node_name);
  ~EKFLocalizer();

  // void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
  // void
  // VehStateCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr
  // msg);
  void GPSOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timer_callback();
  void GPSLATLONCallback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg);
  void BOTTOMINSPVACallback(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg);
  void TOPINSPVACallback(const novatel_oem7_msgs::msg::INSPVA::SharedPtr msg);

  void BESTVELCallback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg);
  void MessegefilteringCallback(
      const sensor_msgs::msg::Imu ::ConstSharedPtr &imu_msg,
      const raptor_dbw_msgs::msg::WheelSpeedReport::ConstSharedPtr
          &wheel_speed_msg);

private:
  void respond();
  void run();

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr
      sub_vehstate;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;

  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr
      sub_gpslatlon;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr subINSPVA;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr subTOPINSPVA;

  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr subBESTVEL;

      rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_EKF_odometry;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_bestpos_odometry;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

  using SyncPolicyT = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::Imu, raptor_dbw_msgs::msg::WheelSpeedReport>;
  message_filters::Subscriber<sensor_msgs::msg::Imu> sub_filtered_IMU;
  message_filters::Subscriber<raptor_dbw_msgs::msg::WheelSpeedReport>
      sub_filtered_Wheel;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicyT>> m_sync;

  c_ekf m_ekf;
  nif::localization::utils::GeodeticConverter conv_;

  double m_origin_lat;
  double m_origin_lon;

  double m_dGPS_X;
  double m_dGPS_Y;
  double m_dGPS_Z;
  double m_dGPS_Heading;
  double m_dGPS_roll;
  double m_dGPS_Heading_prev;
  bool bGPS;
  bool bGPSHeading;

  double m_dIMU_yaw_rate;
  double m_dVelolcity_X;

  bool bImuFirstCall;
  double ImuTimeDouble;
  double ImuPrevTimeDouble;

  bool bVehVelocityFirstCall;
  double VehVelocityTimeDouble;
  double VehVelocityPrevTimeDouble;

  double VehVelocity_dt;

  // distance comparison between current point and gps_ref
  double minimum_value = 0.0;
  int ref_index = 0;
  cv::Mat vel_and_yawRate, GPS_data;

  double update_threshold_velocity = 1.4;

  double gps_count_prev = 0.0;
  double gps_count_curr = 0.0;
  bool gps_flag;
  bool heading_flag;

  bool m_use_inspva_heading;
  bool m_inspva_heading_init = false;

  double m_bestvel_heading_update_thres;

  double vel_y;

  int countpos = 0;

  double hcount = 0.0;
  double hbias = 0.0;

  rclcpp::Time imu_time_last_update;
  rclcpp::Time bestpos_time_last_update;

  rclcpp::Duration gps_timeout = rclcpp::Duration(1, 0);
};
} // namespace ekf
} // namespace localization
} // namespace nif

#endif // EKF_LOCALIZER_NODE_H
