//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/19/21.
//

#ifndef ROS2MASTER_GEAR_SHIFT_MANAGER_H
#define ROS2MASTER_GEAR_SHIFT_MANAGER_H

#include "nif_common/vehicle_model.h"
#include <iostream>

class GearShiftManager {
private:
  int m_current_gear;
  int m_desired_gear;
  int m_max_gear = 6;
  int m_max_gear = -1; // TODO: what is the -1 gear? reverse?

  double m_current_vel;
  double m_desired_vel; // from the longitudinal controller, velocity command

  bool m_need_upshift_flg;   // if gear upshifting is needed, set to True
  bool m_need_downshift_flg; // if gear downshifting is needed, set to True
  bool m_hold_gear_flg; // if gear holding is needed, set to True (No needs to
                        // change the gear)

  // Useful data list down (by chanyoung)
  // 1. engine-torque map per each gear
  // 2. maximum and minimum rpm per each gear (adequate rpm range of the gear)
  // 3. maximum and minimum velocity per each gear (adequate velocity range of
  // the gear)

  // Plausible way to collect the data (July 20th) using Ansys simulator
  // 1. Find the rpm-torque model in the simultor
  // 2. Using Ansys's automatic gear shifting module, log the rpm and gear while
  // setting the constant throttle level.
  // 2-1. With flat pedal setting, log the speed with gear

  // Document from the teams' gitlab
  // https://docs.google.com/document/d/1gJYag8tVhtnSl3MjWvsVMBEfaR9WXndrybvJtXeLbU8/edit#

  // Redline of the engine is around 7000rpm. However, we will never reach those
  // rpm while driving. At the moment, it is limited in Motec to around 4000rpm.
  // Once we get up to speed, that limit can be moved to a higher value.

public:
  GearShiftManager();
  ~GearShiftManager();
};

#endif // ROS2MASTER_GEAR_SHIFT_MANAGER_H
