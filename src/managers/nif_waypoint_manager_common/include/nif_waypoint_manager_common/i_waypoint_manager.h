//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#ifndef NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H
#define NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H

#include "c_wpt.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

class IWaypointManager {
public:
  IWaypointManager() {}
  IWaypointManager(string wpt_file_path_);
  void updateCurrentPose(nav_msgs::msg::Odometry& ego_vehicle_odom){};

  nav_msgs::msg::Path& getWPTInNavMsg();
  c_wpt& getWPT();

  nav_msgs::msg::Path& getMapTrackInBody();
  nav_msgs::msg::Path& getMapTrackInGlobal();

  virtual void update(nav_msgs::msg::Odometry& ego_vehicle_odom,
                      nav_msgs::msg::Path& local_path) = 0;
  virtual void update(nav_msgs::msg::Odometry& ego_vehicle_odom,
                      nav_msgs::msg::Path& local_path,
                      nav_msgs::msg::Path& global_path) = 0;

private:
  c_wpt m_wpt;
};

#endif // NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H
