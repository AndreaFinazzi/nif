//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#ifndef NIF_WAYPOINT_MANAGER_MISSION_H
#define NIF_WAYPOINT_MANAGER_MISSION_H

#include "memory"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string>
#include <vector>

#include "nif_common_nodes/i_base_node.h"
#include "nif_utils/frenet_path_generator.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"

using namespace std;
using namespace Frenet;

class WaypointManagerMission {
public:
  WaypointManagerMission(const string &rl_wpt_file_path_,
                         const string &pit_wpt_file_path_,
                         const string &body_frame_id_,
                         const string &global_frame_id_,
                         const double &spline_interval_);

  void setCurrentOdometry(const nav_msgs::msg::Odometry &ego_vehicle_odom);
  void setSystemStatus(const nif_msgs::msg::SystemStatus &sys_status);
  void calcMapTrack();

  void genCandidates();
  nav_msgs::msg::Path getDesiredMapTrackInGlobal() {
    calcMapTrack();
    return m_map_track_path_global;
  }
  nav_msgs::msg::Path getDesiredMapTrackInBody() {
    calcMapTrack();
    return m_map_track_path_body;
  }

private:
  WaypointManagerMission() {}

  string m_rl_wpt_file_path;
  string m_pit_wpt_file_path;
  string m_body_frame_id_str;
  string m_global_frame_id_str;
  double m_spline_interval;

  bool m_odom_first_callbacked;
  bool m_on_track_flg = true;

  double m_planning_vel_default;      // in [m/s]
  double m_left_side_sampling_width;  // in [m]
  double m_right_side_sampling_width; // in [m]
  double m_sampling_width_d;          // in [m]

  double m_planning_t = 4.4;

  nav_msgs::msg::Odometry m_cur_odom;
  nif_msgs::msg::SystemStatus m_cur_sys_status;
  nif_msgs::msg::MissionStatus m_cur_mission_code;

  nav_msgs::msg::Path m_map_track_path_global;
  nav_msgs::msg::Path m_map_track_path_body;

  std::shared_ptr<IWaypointManager> m_rl_wpt_manager;
  std::shared_ptr<IWaypointManager> m_pit_wpt_manager;

  // NOTE : only working in the body coordinate
  std::shared_ptr<FrenetPathGenerator> m_frenet_generator;

  std::vector<double> splined_x, splined_y, splined_yaw;
  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D_xy;
};

#endif // NIF_WAYPOINT_MANAGER_MISSION_H
