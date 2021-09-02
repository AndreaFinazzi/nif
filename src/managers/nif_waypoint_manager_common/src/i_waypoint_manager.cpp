//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#include "nif_waypoint_manager_common/i_waypoint_manager.h"
#include <tf2/LinearMath/Quaternion.h>

IWaypointManager::IWaypointManager(const vector<string>& wpt_file_path_list_,
                                   const string& body_frame_id_,
                                   const string& global_frame_id_)
    : m_body_frame_id(body_frame_id_),
      m_global_frame_id(global_frame_id_) {

  if (wpt_file_path_list_.empty()) {
    throw std::runtime_error("wpt_file_path_list_ is empty. WaypointManager cannot be initialized.");
  }

  m_wpt_list.clear();

  bool wpt_3d_flg = false;
  bool spline_flg = true;

  for (const auto & wpt_file_idx : wpt_file_path_list_) {
    c_wpt obj(wpt_file_idx,
              "",
              m_global_frame_id,
              wpt_3d_flg,
              spline_flg,
              0.5);
    m_wpt_list.push_back(obj);
  }

  if (m_wpt_list.size() > 0) {
    c_default_wpt = m_wpt_list[0];
    c_desired_wpt = c_default_wpt;
  } else {
    throw std::out_of_range("Waypoint is not properly loaded.");
  }

  // defualt desired wpt is set to the wpt which is firstly loaded.
  m_default_wpt_in_nav_path = c_default_wpt.getWPTinNavPath();
  m_desired_wpt_in_nav_path = c_desired_wpt.getWPTinNavPath();
}

void IWaypointManager::setCurrentOdometry(
    const nav_msgs::msg::Odometry& ego_vehicle_odom) {
  m_current_pose = ego_vehicle_odom;

  // calc heading(yaw) in rad and deg
  tf2::Quaternion quat(m_current_pose.pose.pose.orientation.x,
                       m_current_pose.pose.pose.orientation.y,
                       m_current_pose.pose.pose.orientation.z,
                       m_current_pose.pose.pose.orientation.w);
  tf2::Matrix3x3(quat).getRPY(
      m_current_roll_rad, m_current_pitch_rad, m_current_yaw_rad);

  int current_idx = getCurrentIdx(m_desired_wpt_in_nav_path, ego_vehicle_odom);
  m_desired_maptrack_in_global =
      calcMapTrackInGlobal(m_desired_wpt_in_nav_path, current_idx);
  m_desired_maptrack_in_body =
      getPathGlobaltoBody(m_desired_maptrack_in_global);
}

void IWaypointManager::setCurrentIdx(
    const nav_msgs::msg::Path &reference_path,
    const nav_msgs::msg::Odometry &ego_vehicle_odom)
{
  unsigned int current_idx = 0;
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
  if (current_idx > reference_path.poses.size()) {
    throw std::length_error("I_WPT_MANAGER:Current index is not set properly.");
    return;
  }
  m_current_idx = current_idx;
}

unsigned int IWaypointManager::getCurrentIdx(
    const nav_msgs::msg::Path &reference_path,
    const nav_msgs::msg::Odometry &ego_vehicle_odom) {
  setCurrentIdx(reference_path, ego_vehicle_odom);
  return m_current_idx;
}

int IWaypointManager::getWPTIdx(nav_msgs::msg::Path& reference_path,
                                geometry_msgs::msg::PoseStamped& target_pose) {
  int target_idx = 0;
  double min_dist = INFINITY;
  for (int pt_idx = 0; pt_idx < reference_path.poses.size(); pt_idx++) {
    double dist = sqrt(pow(target_pose.pose.position.x -
                               reference_path.poses[pt_idx].pose.position.x,
                           2) +
                       pow(target_pose.pose.position.y -
                               reference_path.poses[pt_idx].pose.position.y,
                           2));
    if (min_dist > dist) {
      min_dist = dist;
      target_idx = pt_idx;
    }
  }
  return target_idx;
}

nav_msgs::msg::Path IWaypointManager::calcMapTrackInGlobal(
    nav_msgs::msg::Path& reference_full_path_, unsigned int current_idx_) const {
  //   If the vehicle is on the end of the wpt, current idx is going to
  //   zero again. Make sure that there is no memory overflow when you access
  //   based on this current idx

  if (m_size_of_map_track > reference_full_path_.poses.size()) {
    throw std::length_error("I_WPT_MANAGER : maptrack size is larger than the "
                            "full reference path.");
  }

  nav_msgs::msg::Path map_track_in_global;

  if (current_idx_ + m_size_of_map_track < reference_full_path_.poses.size()) {
    map_track_in_global.poses = vector<geometry_msgs::msg::PoseStamped>(
        reference_full_path_.poses.begin() + current_idx_,
        reference_full_path_.poses.begin() + current_idx_ +
            m_size_of_map_track);
  } else {
    // from current idx to end
    map_track_in_global.poses = vector<geometry_msgs::msg::PoseStamped>(
        reference_full_path_.poses.begin() + current_idx_,
        reference_full_path_.poses.end());
    // from 0 to left size
    map_track_in_global.poses.insert(map_track_in_global.poses.end(),
                                     reference_full_path_.poses.begin(),
                                     reference_full_path_.poses.begin() +
                                         (current_idx_ + m_size_of_map_track -
                                          reference_full_path_.poses.size()));
  }
  return map_track_in_global;
}

nav_msgs::msg::Path IWaypointManager::calcMapTrackInBody(
    nav_msgs::msg::Path& map_track_in_global_) {
  nav_msgs::msg::Path map_track_in_body;
  map_track_in_body.header.frame_id = m_body_frame_id;
  map_track_in_body = getPathGlobaltoBody(map_track_in_global_);
  return map_track_in_body;
}

geometry_msgs::msg::PoseStamped IWaypointManager::getPtBodytoGlobal(
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

geometry_msgs::msg::PoseStamped IWaypointManager::getPtGlobaltoBody(
    const geometry_msgs::msg::PoseStamped &point_in_global_) const {
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
IWaypointManager::getPathBodytoGlobal(nav_msgs::msg::Path& path_in_body_) {
  nav_msgs::msg::Path path_in_global;
  path_in_global.header.frame_id = m_global_frame_id;
  for (auto & pose : path_in_body_.poses) {
    path_in_global.poses.push_back(
        getPtBodytoGlobal(pose));
  }
  return path_in_global;
}

nav_msgs::msg::Path
IWaypointManager::getPathGlobaltoBody(nav_msgs::msg::Path& path_in_global_) {
  nav_msgs::msg::Path path_in_body;
  path_in_body.header.frame_id = m_body_frame_id;
  for (auto & pose : path_in_global_.poses) {
    path_in_body.poses.push_back(
        getPtGlobaltoBody(pose));
  }
  return path_in_body;
}

void IWaypointManager::resetDesiredWPT() {
  c_desired_wpt = c_default_wpt;
  m_desired_wpt_in_nav_path = m_default_wpt_in_nav_path;
}

void IWaypointManager::updateDesiredWPT(
    nav_msgs::msg::Path& local_path_in_body) {
  nav_msgs::msg::Path local_path_in_global =
      getPathBodytoGlobal(local_path_in_body);
  int start_wpt_idx =
      getWPTIdx(m_desired_wpt_in_nav_path, local_path_in_global.poses[0]);
  int end_wpt_idx =
      getWPTIdx(m_desired_wpt_in_nav_path, local_path_in_global.poses[-1]);

  double dist = sqrt(pow(local_path_in_global.poses[0].pose.position.x -
                             local_path_in_global.poses[-1].pose.position.x,
                         2) +
                     pow(local_path_in_global.poses[0].pose.position.y -
                             local_path_in_global.poses[-1].pose.position.y,
                         2));

  if (dist < 2.0) {
    // local path distance is too short to calculate the both index. In this
    // case, it can be inverted. Ignore this case.
  } else {
    if (end_wpt_idx > start_wpt_idx) {
      vector<geometry_msgs::msg::PoseStamped> updated_poses;
      updated_poses = vector<geometry_msgs::msg::PoseStamped>(
          m_desired_wpt_in_nav_path.poses.begin(),
          m_desired_wpt_in_nav_path.poses.begin() + start_wpt_idx - 1);
      updated_poses.insert(updated_poses.end(),
                           local_path_in_global.poses.begin(),
                           local_path_in_global.poses.end());
      updated_poses.insert(updated_poses.end(),
                           m_desired_wpt_in_nav_path.poses.begin() +
                               end_wpt_idx,
                           m_desired_wpt_in_nav_path.poses.end() - 1);
      m_desired_wpt_in_nav_path.poses = updated_poses;
      setCurrentIdx(m_desired_wpt_in_nav_path, m_current_pose);
    } else {
      vector<geometry_msgs::msg::PoseStamped> updated_poses;
      int closest_to_origin_wpt_idx_in_local_wpt =
          getWPTIdx(local_path_in_global, m_desired_wpt_in_nav_path.poses[0]);
      updated_poses = vector<geometry_msgs::msg::PoseStamped>(
          local_path_in_global.poses.begin() +
              closest_to_origin_wpt_idx_in_local_wpt - 1,
          local_path_in_global.poses.end());
      updated_poses.insert(updated_poses.end(),
                           m_desired_wpt_in_nav_path.poses.begin() +
                               end_wpt_idx,
                           m_desired_wpt_in_nav_path.poses.end());
      updated_poses.insert(updated_poses.end(),
                           local_path_in_global.poses.begin(),
                           local_path_in_global.poses.begin() +
                               closest_to_origin_wpt_idx_in_local_wpt - 1);
      setCurrentIdx(m_desired_wpt_in_nav_path, m_current_pose);
    }
  }
}
