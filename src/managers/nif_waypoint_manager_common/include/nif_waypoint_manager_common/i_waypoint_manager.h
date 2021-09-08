//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#ifndef NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H
#define NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "nif_waypoint_manager_common/c_wpt.h"

class IWaypointManager {
public:
  IWaypointManager() {}
  IWaypointManager(const vector<string>& wpt_file_path_list_,
                   const string& body_frame_id_,
                   const string& global_frame_id_,
                   const int& spline_interval_);

  vector<c_wpt>& getListOfWPT() {
    return m_wpt_list;
  }
  c_wpt& getDesiredWPT() {
    return c_desired_wpt;
  }
  nav_msgs::msg::Path& getDesiredWPTInNavMsg() {
    return m_desired_wpt_in_nav_path;
  }
  nav_msgs::msg::Path& getDesiredMapTrackInBody() {
    return m_desired_maptrack_in_body;
  }
  nav_msgs::msg::Path& getDesiredMapTrackInGlobal() {
    return m_desired_maptrack_in_global;
  }
  geometry_msgs::msg::PoseStamped
  getPtBodytoGlobal(geometry_msgs::msg::PoseStamped& point_in_body_);

  nav_msgs::msg::Path getPathBodytoGlobal(nav_msgs::msg::Path& path_in_body_);

  geometry_msgs::msg::PoseStamped getPtGlobaltoBody(
      const geometry_msgs::msg::PoseStamped& point_in_global_) const;

  nav_msgs::msg::Path getPathGlobaltoBody(nav_msgs::msg::Path& path_in_global_);

  void setCurrentOdometry(const nav_msgs::msg::Odometry& ego_vehicle_odom);

  void setSizeOfMapTrack(int size_of_map_track_) {
    m_size_of_map_track = size_of_map_track_;
  }

  void resetDesiredWPT();

  void setCurrentIdx(const nav_msgs::msg::Path& reference_path,
                     const nav_msgs::msg::Odometry& ego_vehicle_odom);

  unsigned int getCurrentIdx(const nav_msgs::msg::Path& reference_path,
                             const nav_msgs::msg::Odometry& ego_vehicle_odom);

  int getWPTIdx(nav_msgs::msg::Path& reference_path,
                geometry_msgs::msg::PoseStamped& target_pose);

  nav_msgs::msg::Path calcMapTrackInGlobal(nav_msgs::msg::Path& reference_path_,
                                           unsigned int current_idx_) const;

  nav_msgs::msg::Path calcMapTrackInBody(nav_msgs::msg::Path& reference_path_);

  virtual void updateDesiredWPT(nav_msgs::msg::Path& local_path_in_body);

private:
  vector<c_wpt> m_wpt_list;
  vector<int>
      m_current_idx_list; // current idxs with repective to the multiple wpts

  c_wpt c_default_wpt; // default wpt when the waypoint file is loaded.
  c_wpt c_desired_wpt; // dynamically updated from the planning node and any
                       // other else

  nav_msgs::msg::Path m_default_wpt_in_nav_path; // default wpt path in nav_msgs
  nav_msgs::msg::Path
      m_desired_wpt_in_nav_path; // dynamically updated path (if not updated,
                                 // set as a default one)

  string m_body_frame_id, m_global_frame_id; // frame_id

  vector<nav_msgs::msg::Path> m_maptrack_in_body_list;   // map_track
  vector<nav_msgs::msg::Path> m_maptrack_in_global_list; // map_track

  nav_msgs::msg::Path
      m_desired_maptrack_in_body; // dynamically updated path (if not updated,
                                  // set as a default one)
  nav_msgs::msg::Path
      m_desired_maptrack_in_global; // dynamically updated path (if not updated,
                                    // set as a default one)
  nav_msgs::msg::Odometry
      m_current_pose; // current vehicle pose, this should be updated as fast as
                      // possible in the node subscriber

  unsigned int m_current_idx =
      0; // current idx with repective to the desired wpt
  unsigned int m_size_of_map_track =
      100; // size of map track which is set by the user
           // (yaml or set?), currently set as a default 100

  double m_current_roll_deg = 0.0;  // current pose (quartonion to eular)
  double m_current_roll_rad = 0.0;  // current pose (quartonion to eular)
  double m_current_pitch_deg = 0.0; // current pose (quartonion to eular)
  double m_current_pitch_rad = 0.0; // current pose (quartonion to eular)
  double m_current_yaw_deg = 0.0;   // current pose (quartonion to eular)
  double m_current_yaw_rad = 0.0;   // current pose (quartonion to eular)
  double m_spline_interval = 0.0;
};

#endif // NIF_WAYPOINT_MANAGER_COMMON_I_WAYPOINT_MANAGER_H
