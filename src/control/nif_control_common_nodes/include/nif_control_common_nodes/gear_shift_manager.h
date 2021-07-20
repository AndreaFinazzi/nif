//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/19/21.
//

#ifndef ROS2MASTER_GEAR_SHIFT_MANAGER_H
#define ROS2MASTER_GEAR_SHIFT_MANAGER_H

#include <iostream>

class GearShiftManager {
private:
  int m_current_gear;
  int m_desired_gear;
  int m_max_gear = 6;
  int m_max_gear = -1; // TODO: what is the -1 gear? reverse?

public:
  GearShiftManager();
  ~GearShiftManager();
};

#endif // ROS2MASTER_GEAR_SHIFT_MANAGER_H
