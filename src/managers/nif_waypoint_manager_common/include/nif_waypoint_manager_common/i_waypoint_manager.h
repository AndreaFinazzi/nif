//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#ifndef NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H
#define NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"

class IWaypointManager
{
public:
  virtual std::vector<nav_msgs::msg::Path> & getRacingLines();

  virtual void update(nav_msgs::msg::Odometry & ego_vehicle_odom){};

  virtual void update(nav_msgs::msg::Odometry & ego_vehicle_odom, nav_msgs::msg::Path & local_path ) {};

  virtual void update(nav_msgs::msg::Odometry & ego_vehicle_odom, nav_msgs::msg::Path & local_path,  nav_msgs::msg::Path & global_path) {};

  nav_msgs::msg::Path & getMapTrackInBody();
  nav_msgs::msg::Path & getMapTrackInGlobal();

};

#endif // NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H
