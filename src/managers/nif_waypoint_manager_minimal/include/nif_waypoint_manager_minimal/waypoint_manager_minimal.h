//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#ifndef NIF_WAYPOINT_MANAGER_MINIMAL_H
#define NIF_WAYPOINT_MANAGER_MINIMAL_H

#include <iostream>
#include <string>
#include <vector>

#include "nif_waypoint_manager_common/i_waypoint_manager.h"

#include "nav_msgs/msg/path.hpp"

class WaypointManagerMinimal : public IWaypointManager
{
public:
  WaypointManagerMinimal() {}
  WaypointManagerMinimal(WaypointManagerMinimal& wpt_manager_);
  WaypointManagerMinimal(string wpt_file_path_,
  string wpt_alias = "",
  double wpt_interval = 0.5);
  ~WaypointManagerMinimal();

  void loadWPTFile(
      string wpt_file_path_,
      double wpt_interval); // load wpt file & assign racing_line_inglobal

  const string getWptFilePath() const {
    return m_wpt_file_path;
  }
  const string getWptAlias() const {
    return m_wpt_alias;
  }
  const int getWptTotalLength() const {
    return m_wpt_total_length;
  }
  const int getCurrentIdx() const {
    return m_current_wpt_idx;
  }
  const double getWptInterval() const {
    return m_wpt_interval;
  }
  const nav_msgs::msg::Path getWptInGlobal() const {
    return m_wpt_inglobal;
  }
  const nav_msgs::msg::Path getMapTrackInGlobal() const {
    return m_map_track_inglobal;
  }
  const nav_msgs::msg::Path getMapTrackInBody() const {
    return m_map_track_inbody;
  }
  const vector<double>& getWptYawInGlobalRad() const {
    return m_wpt_yaw_inglobal_rad;
  }
  const vector<double>& getWptYawInGlobalDeg() const {
    return m_wpt_yaw_inglobal_deg;
  }
  const vector<double>& getMapTrackYawInGlobalRad() const {
    return m_map_track_yaw_inglobal_rad;
  }
  const vector<double>& getMapTrackYawInGlobalDeg() const {
    return m_map_track_yaw_inglobal_deg;
  }
  const vector<double>& getMapTrackYawInBodyRad() const {
    return m_map_track_yaw_inbody_rad;
  }
  const vector<double>& getMapTrackYawInBodyDeg() const {
    return m_map_track_yaw_inbody_deg;
  }

  void setWptInterval(double interval_) {
    m_wpt_interval = interval_;
  }

private:
  string m_wpt_file_path = "";
  string m_wpt_alias = "";
  int m_wpt_total_length; // number of wpt in the wpt file
  int m_current_wpt_idx;
  double m_wpt_interval = 0.5;        // in meter
  int m_map_track_length_in_idx;      // how long for map track in index level
  double m_map_track_length_in_meter; // how long for map track in meter

  nav_msgs::msg::Path m_wpt_inglobal;
  nav_msgs::msg::Path m_map_track_inglobal;
  nav_msgs::msg::Path m_map_track_inbody;
  vector<double> m_wpt_yaw_inglobal_rad;
  vector<double> m_wpt_yaw_inglobal_deg;
  vector<double> m_map_track_yaw_inglobal_rad;
  vector<double> m_map_track_yaw_inglobal_deg;
  vector<double> m_map_track_yaw_inbody_rad;
  vector<double> m_map_track_yaw_inbody_deg;
};

#endif //NIF_WAYPOINT_MANAGER_MINIMAL_H
