//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/12/21.
//

#include "nif_localization_minimal/localization_minimal.h"

LocalizationMinimal::LocalizationMinimal() {
  m_geo_converter_ptr = std::make_shared<GeodeticConverter>();
}

LocalizationMinimal::~LocalizationMinimal() {}

void LocalizationMinimal::setGPSHorizontalData(
    novatel_gps_msgs::msg::Inspva& gps_horizontal_data_) {
  m_geo_converter_ptr->geodetic2Ned(
      gps_horizontal_data_.latitude,
      gps_horizontal_data_.longitude,
      0.0,
      &m_veh_odom_horizontal.pose.pose.position.x,
      &m_veh_odom_horizontal.pose.pose.position.y,
      &m_veh_odom_horizontal.pose.pose.position.z);

  // TODO : Not sure about passing the roll,pitch. Maybe we can just pass zero.
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
    novatel_gps_msgs::msg::Inspva& gps_vertical_data_) {
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