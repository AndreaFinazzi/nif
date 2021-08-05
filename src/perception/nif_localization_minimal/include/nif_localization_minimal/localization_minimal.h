//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/12/21.
//

#ifndef NIF_LOCALIZATION_MINIMAL_H
#define NIF_LOCALIZATION_MINIMAL_H

#include "nav_msgs/msg/odometry.hpp"
#include "nif_common/constants.h"
#include "nif_frame_id/frame_id.h"
#include "nif_utils/geodetic_converter.h"
#include "novatel_gps_msgs/msg/inspva.hpp"
#include "tf2/LinearMath/Quaternion.h"

class LocalizationMinimal {
private:
  std::shared_ptr<GeodeticConverter> m_geo_converter_ptr;

  novatel_gps_msgs::msg::Inspva m_gps_horizontal, m_gps_vertical;
  nav_msgs::msg::Odometry m_veh_odom_horizontal, m_veh_odom_vertical,
      m_veh_odom_fused;

  double m_heading_rad_gps_horizontal, m_heading_rad_gps_vertical,
      m_heading_rad_gps_fused;
  double m_heading_deg_gps_horizontal, m_heading_deg_gps_vertical,
      m_heading_deg_gps_fused;

public:
  LocalizationMinimal();

  ~LocalizationMinimal();

  void setGPSHorizontalData(
      const novatel_gps_msgs::msg::Inspva& gps_horizontal_data_);

  void testGPSHorizontalData();
  void
  setGPSVerticalData(const novatel_gps_msgs::msg::Inspva& gps_vertical_data_);

  void linearFusion();

  nav_msgs::msg::Odometry getVehOdomByHorizontalGPS() {
    return m_veh_odom_horizontal;
  }
  nav_msgs::msg::Odometry getVehOdomByVerticalGPS() {
    return m_veh_odom_vertical;
  }
  nav_msgs::msg::Odometry getVehOdomByFusion();
};

#endif // NIF_LOCALIZATION_MINIMAL_H
