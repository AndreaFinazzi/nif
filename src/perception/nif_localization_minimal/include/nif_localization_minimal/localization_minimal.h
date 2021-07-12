//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/12/21.
//

#ifndef NIF_LOCALIZATION_MINIMAL_H
#define NIF_LOCALIZATION_MINIMAL_H

#include "nav_msgs/msg/odometry.hpp"
#include "nif_frame_id/frame_id.h"
#include "nif_utils/geodetic_converter.h"

class LocalizationMinimal {
private:
  /* data */

  std::shared_ptr<GeodeticConverter> m_geo_converter_ptr;

public:
  LocalizationMinimal();
  ~LocalizationMinimal();

  void setGPSHorizontalData();
  void setGPSVerticalData();
};

#endif // NIF_LOCALIZATION_MINIMAL_H
