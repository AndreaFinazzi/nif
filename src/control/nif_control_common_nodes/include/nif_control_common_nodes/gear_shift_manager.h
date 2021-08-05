//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/19/21.
//

#ifndef ROS2MASTER_GEAR_SHIFT_MANAGER_H
#define ROS2MASTER_GEAR_SHIFT_MANAGER_H

#include "nif_common/vehicle_model.h"
#include <algorithm>
#include <chrono>
#include <ctime>
#include <iostream>
#include <sys/time.h>

class GearShiftManager {
private:
  int max_gear;
  int min_gear;
  int current_gear;
  int current_rpm;
  double current_vel;
  int desired_gear;
  double desired_vel;      // from the longitudinal controller, velocity command
  bool is_upshift_require; // if gear upshifting is needed, set to True
  bool is_downshift_require; // if gear downshifting is needed, set to True
  bool is_stay_gear_require;
  double rpm_gain_for_upshift =
      0.9; // It must be lower or equal to 1.0 (percentage)
  double rpm_gain_for_downshift =
      1.1; // It must be greater or equal to 1.0 (percentage)
  double last_gearshift_timestamp_sec = 0.0;
  int last_desired_gear;
  bool hold_gearshift_flg = false;

public:
  void setRpmGainForUpshift(double rpmGainForUpshift);
  void setRpmGainForDownshift(double rpmGainForDownshift);

public:
  int getMaxGear() const;
  void setMaxGear(int maxGear);
  int getMinGear() const;
  void setMinGear(int minGear);
  void setCurrentRpm(int currentRpm);
  void setCurrentVel(double currentVel);
  void setDesiredVel(double desiredVel);
  void setCurrentGear(int currentGear);
  bool isUpshiftRequire() const;
  bool isDownshiftRequire() const;
  bool isStayGearRequire() const;

  /**
   * returns desired gear position.
   * @return int
   */
  int getDesiredGear(double desiredVel,
                     double currentVel,
                     int currentGear,
                     int currentRpm);

  // Document from the teams' gitlab
  // https://docs.google.com/document/d/1gJYag8tVhtnSl3MjWvsVMBEfaR9WXndrybvJtXeLbU8/edit#

  // Redline of the engine is around 7000rpm. However, we will never reach those
  // rpm while driving. At the moment, it is limited in Motec to around 4000rpm.
  // Once we get up to speed, that limit can be moved to a higher value.

public:
  GearShiftManager();
  GearShiftManager(int max_gear, int min_gear);
  ~GearShiftManager();

  void init();
  void init(int max_gear, int min_gear);
};

#endif // ROS2MASTER_GEAR_SHIFT_MANAGER_H
