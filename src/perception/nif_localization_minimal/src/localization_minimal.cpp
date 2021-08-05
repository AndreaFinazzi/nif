//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/12/21.
//

#include "nif_localization_minimal/localization_minimal.h"

LocalizationMinimal::LocalizationMinimal() {
  m_geo_converter_ptr = std::make_shared<GeodeticConverter>(
      39.8125900071711, -86.3418060783425, 0);
}

LocalizationMinimal::~LocalizationMinimal() {}

nav_msgs::msg::Odometry LocalizationMinimal::getVehOdomByFusion() {
  linearFusion();
  return m_veh_odom_fused;
}

void LocalizationMinimal::linearFusion() {
  // position linear fusion
  m_veh_odom_fused.pose.pose.position.x =
      (m_veh_odom_horizontal.pose.pose.position.x +
       m_veh_odom_vertical.pose.pose.position.x) /
      2.0;
  m_veh_odom_fused.pose.pose.position.y =
      (m_veh_odom_horizontal.pose.pose.position.y +
       m_veh_odom_vertical.pose.pose.position.y) /
      2.0;
  m_veh_odom_fused.pose.pose.position.z =
      (m_veh_odom_horizontal.pose.pose.position.z +
       m_veh_odom_vertical.pose.pose.position.z) /
      2.0;

  // orientation linear fusion
  m_veh_odom_fused.pose.pose.orientation.x =
      (m_veh_odom_horizontal.pose.pose.orientation.x +
       m_veh_odom_vertical.pose.pose.orientation.x) /
      2.0;
  m_veh_odom_fused.pose.pose.orientation.y =
      (m_veh_odom_horizontal.pose.pose.orientation.y +
       m_veh_odom_vertical.pose.pose.orientation.y) /
      2.0;
  m_veh_odom_fused.pose.pose.orientation.z =
      (m_veh_odom_horizontal.pose.pose.orientation.z +
       m_veh_odom_vertical.pose.pose.orientation.z) /
      2.0;
  m_veh_odom_fused.pose.pose.orientation.w =
      (m_veh_odom_horizontal.pose.pose.orientation.w +
       m_veh_odom_vertical.pose.pose.orientation.w) /
      2.0;

  // heading linear fusion
  m_heading_rad_gps_fused =
      m_heading_rad_gps_horizontal + m_heading_rad_gps_vertical;
  m_heading_deg_gps_fused =
      m_heading_deg_gps_horizontal + m_heading_deg_gps_vertical;
}

void LocalizationMinimal::testGPSHorizontalData() {
  //   gps_horizontal_data_.latitude = 39.81184488617023;
  //   gps_horizontal_data_.longitude = -86.34178892423053;

  double latitude = 39.81184488617023;
  double longitude = -86.34178892423053;

  double roll = 0;
  double pitch = 0;
  double azimuth = 178.12580144670395;

  m_geo_converter_ptr->geodetic2Ned(
      latitude,
      longitude,
      0.0,
      &m_veh_odom_horizontal.pose.pose.position.x,
      &m_veh_odom_horizontal.pose.pose.position.y,
      &m_veh_odom_horizontal.pose.pose.position.z);

  std::cout << m_veh_odom_horizontal.pose.pose.position.x << " "
            << m_veh_odom_horizontal.pose.pose.position.y << " "
            << m_veh_odom_horizontal.pose.pose.position.z << std::endl;

  // TODO : Not sure about passing the roll,pitch. Maybe we can
  // just pass zero.
  tf2::Quaternion vehicle_quat;
  vehicle_quat.setRPY(roll * nif::common::constants::DEG2RAD,
                      pitch * nif::common::constants::DEG2RAD,
                      azimuth * nif::common::constants::DEG2RAD);
  vehicle_quat = vehicle_quat.normalize();
  m_veh_odom_horizontal.pose.pose.orientation.x = vehicle_quat.x();
  m_veh_odom_horizontal.pose.pose.orientation.y = vehicle_quat.y();
  m_veh_odom_horizontal.pose.pose.orientation.z = vehicle_quat.z();
  m_veh_odom_horizontal.pose.pose.orientation.w = vehicle_quat.w();

  m_heading_deg_gps_horizontal = azimuth;
  m_heading_rad_gps_horizontal =
      m_heading_deg_gps_horizontal * nif::common::constants::DEG2RAD;
}

void LocalizationMinimal::setGPSHorizontalData(
    const novatel_gps_msgs::msg::Inspva& gps_horizontal_data_) {
  m_geo_converter_ptr->geodetic2Ned(
      gps_horizontal_data_.latitude,
      gps_horizontal_data_.longitude,
      0.0,
      &m_veh_odom_horizontal.pose.pose.position.x,
      &m_veh_odom_horizontal.pose.pose.position.y,
      &m_veh_odom_horizontal.pose.pose.position.z);

  // TODO : Not sure about passing the roll,pitch. Maybe we can
  // just pass zero.
  tf2::Quaternion vehicle_quat;
  vehicle_quat.setRPY(gps_horizontal_data_.roll,
                      gps_horizontal_data_.pitch,
                      gps_horizontal_data_.azimuth);
  vehicle_quat = vehicle_quat.normalize();
  m_veh_odom_horizontal.pose.pose.orientation.x = vehicle_quat.x();
  m_veh_odom_horizontal.pose.pose.orientation.y = vehicle_quat.y();
  m_veh_odom_horizontal.pose.pose.orientation.z = vehicle_quat.z();
  m_veh_odom_horizontal.pose.pose.orientation.w = vehicle_quat.w();

  m_heading_deg_gps_horizontal = gps_horizontal_data_.azimuth;
  m_heading_rad_gps_horizontal =
      m_heading_deg_gps_horizontal * nif::common::constants::DEG2RAD;
}

void LocalizationMinimal::setGPSVerticalData(
    const novatel_gps_msgs::msg::Inspva& gps_vertical_data_) {
  m_geo_converter_ptr->geodetic2Ned(gps_vertical_data_.latitude,
                                    gps_vertical_data_.longitude,
                                    0.0,
                                    &m_veh_odom_vertical.pose.pose.position.x,
                                    &m_veh_odom_vertical.pose.pose.position.y,
                                    &m_veh_odom_vertical.pose.pose.position.z);

  // TODO : Not sure about passing the roll,pitch. Maybe we can just pass zero.
  tf2::Quaternion vehicle_quat;
  vehicle_quat.setRPY(gps_vertical_data_.roll,
                      gps_vertical_data_.pitch,
                      gps_vertical_data_.azimuth);
  vehicle_quat = vehicle_quat.normalize();
  m_veh_odom_vertical.pose.pose.orientation.x = vehicle_quat.x();
  m_veh_odom_vertical.pose.pose.orientation.y = vehicle_quat.y();
  m_veh_odom_vertical.pose.pose.orientation.z = vehicle_quat.z();
  m_veh_odom_vertical.pose.pose.orientation.w = vehicle_quat.w();

  m_heading_deg_gps_vertical = gps_vertical_data_.azimuth;
  m_heading_rad_gps_vertical =
      m_heading_deg_gps_vertical * nif::common::constants::DEG2RAD;
}