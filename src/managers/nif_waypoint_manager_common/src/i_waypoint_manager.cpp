//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#include "nif_waypoint_manager_common/i_waypoint_manager.h"
#include <tf2/LinearMath/Quaternion.h>

IWaypointManager::IWaypointManager(string& wpt_yaml_path_,
                                   string& body_frame_id_,
                                   string& global_frame_id_) {
  // TODO: load yaml file and load&save the wpt using "c_wpt"
  // Finally, vector of "c_wpt" should be stored in the m_wpt_list

  m_body_frame_id = body_frame_id_;
  m_global_frame_id = global_frame_id_;

  // yaml file example
  // default wpt and candidates wpt
}

void IWaypointManager::updateCurrentPose(
    nav_msgs::msg::Odometry& ego_vehicle_odom) {
  m_current_pose = ego_vehicle_odom;

  // calc heading(yaw) in rad and deg
  tf2::Quaternion quat(m_current_pose.pose.pose.orientation.x,
                       m_current_pose.pose.pose.orientation.y,
                       m_current_pose.pose.pose.orientation.z,
                       m_current_pose.pose.pose.orientation.w);

  tf2::Matrix3x3(quat).getRPY(
      m_current_roll_rad, m_current_pitch_rad, m_current_yaw_rad);

  m_current_idx_list.clear();

  for (int wpt_idx = 0; wpt_idx < m_wpt_list.size(); wpt_idx++) {
    nav_msgs::msg::Path wpt_in_nav_path =
        (m_wpt_list[wpt_idx].getWPTinNavPath());
    int current_idx = getCurrentIdx(wpt_in_nav_path, ego_vehicle_odom);

    m_current_idx_list.push_back(current_idx);
    m_maptrack_in_global_list.push_back(
        setMapTrackInGlobal(wpt_in_nav_path, current_idx));
    m_maptrack_in_body_list.push_back(
        setMapTrackInBody(m_maptrack_in_global_list[-1]));
  }
}

void IWaypointManager::setCurrentIdx(
    nav_msgs::msg::Path& reference_path,
    nav_msgs::msg::Odometry& ego_vehicle_odom) {
  int current_idx;
  double min_dist = INFINITY;
  for (int pt_idx = 0; pt_idx < reference_path.poses.size(); pt_idx++) {
    double dist = sqrt(pow(ego_vehicle_odom.pose.pose.position.x -
                               reference_path.poses[pt_idx].pose.position.x,
                           2) +
                       pow(ego_vehicle_odom.pose.pose.position.y -
                               reference_path.poses[pt_idx].pose.position.y,
                           2));
    if (min_dist > dist) {
      min_dist = dist;
      current_idx = pt_idx;
    }
  }
  m_current_idx = current_idx;
}

int IWaypointManager::getCurrentIdx(nav_msgs::msg::Path& reference_path,
                                    nav_msgs::msg::Odometry& ego_vehicle_odom) {
  setCurrentIdx(reference_path, ego_vehicle_odom);
  return m_current_idx;
}

nav_msgs::msg::Path
IWaypointManager::setMapTrackInGlobal(nav_msgs::msg::Path& reference_path_,
                                      int current_idx_) {
  //   If the vehicle is on the end of the wpt, current idx is going to
  //   zero again. Make sure that there is no memory overflow when you access
  //   based on this current idx

  nav_msgs::msg::Path map_track_in_global;

  if (current_idx_ + m_size_of_map_track < reference_path_.poses.size()) {
    map_track_in_global.poses = vector<geometry_msgs::msg::PoseStamped>(
        reference_path_.poses.begin() + current_idx_,
        reference_path_.poses.begin() + current_idx_ + m_size_of_map_track);
  } else {
    // from current idx to end
    map_track_in_global.poses = vector<geometry_msgs::msg::PoseStamped>(
        reference_path_.poses.begin() + current_idx_,
        reference_path_.poses.end());
    // from 0 to left size
    map_track_in_global.poses.insert(map_track_in_global.poses.end(),
                                     reference_path_.poses.begin(),
                                     reference_path_.poses.begin() +
                                         (current_idx_ + m_size_of_map_track -
                                          reference_path_.poses.size()));
  }
  return map_track_in_global;
}

nav_msgs::msg::Path
IWaypointManager::setMapTrackInBody(nav_msgs::msg::Path& map_track_in_global_) {
  nav_msgs::msg::Path map_track_in_body;
  map_track_in_body.header.frame_id = m_body_frame_id;
  map_track_in_body = convertPathGlobaltoBody(map_track_in_global_);
  return map_track_in_body;
}

geometry_msgs::msg::PoseStamped IWaypointManager::convertPtBodytoGlobal(
    geometry_msgs::msg::PoseStamped& point_in_body_) {
  geometry_msgs::msg::PoseStamped point_in_global;
  point_in_global.header.frame_id = m_global_frame_id;
  point_in_global.pose.position.x = m_current_pose.pose.pose.position.x +
      point_in_body_.pose.position.x * cos(m_current_yaw_rad) -
      point_in_body_.pose.position.y * sin(m_current_yaw_rad);
  point_in_global.pose.position.y = m_current_pose.pose.pose.position.y +
      point_in_body_.pose.position.x * sin(m_current_yaw_rad) +
      point_in_body_.pose.position.y * cos(m_current_yaw_rad);
  point_in_global.pose.position.z =
      m_current_pose.pose.pose.position.y + point_in_body_.pose.position.z;
  return point_in_global;
}

geometry_msgs::msg::PoseStamped IWaypointManager::convertPtGlobaltoBody(
    geometry_msgs::msg::PoseStamped& point_in_global_) {
  geometry_msgs::msg::PoseStamped point_in_body;
  point_in_body.header.frame_id = m_body_frame_id;
  point_in_body.pose.position.x = cos(-1 * m_current_yaw_rad) *
          (point_in_global_.pose.position.x -
           m_current_pose.pose.pose.position.x) -
      sin(-1 * m_current_yaw_rad) *
          (point_in_global_.pose.position.y -
           m_current_pose.pose.pose.position.y);
  point_in_body.pose.position.y = sin(-1 * m_current_yaw_rad) *
          (point_in_global_.pose.position.x -
           m_current_pose.pose.pose.position.x) +
      cos(-1 * m_current_yaw_rad) *
          (point_in_global_.pose.position.y -
           m_current_pose.pose.pose.position.y);
  point_in_body.pose.position.z =
      point_in_global_.pose.position.z - m_current_pose.pose.pose.position.z;
  return point_in_body;
}

nav_msgs::msg::Path
IWaypointManager::convertPathBodytoGlobal(nav_msgs::msg::Path& path_in_body_) {
  nav_msgs::msg::Path path_in_global;
  path_in_global.header.frame_id = m_global_frame_id;
  for (int pt_idx = 0; pt_idx < path_in_body_.poses.size(); pt_idx++) {
    path_in_global.poses.push_back(
        convertPtBodytoGlobal(path_in_body_.poses[pt_idx]));
  }
  return path_in_global;
}

nav_msgs::msg::Path IWaypointManager::convertPathGlobaltoBody(
    nav_msgs::msg::Path& path_in_global_) {
  nav_msgs::msg::Path path_in_body;
  path_in_body.header.frame_id = m_body_frame_id;
  for (int pt_idx = 0; pt_idx < path_in_global_.poses.size(); pt_idx++) {
    path_in_body.poses.push_back(
        convertPtBodytoGlobal(path_in_global_.poses[pt_idx]));
  }
  return path_in_body;
}

void IWaypointManager::updateDesiredWPT(
    nav_msgs::msg::Odometry& ego_vehicle_odom,
    nav_msgs::msg::Path& local_path) {}