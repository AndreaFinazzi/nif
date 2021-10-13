//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#include "nif_waypoint_manager_mission_selective/waypoint_manager_mission_selective.h"

using namespace std;

WaypointManagerMissionSelective::WaypointManagerMissionSelective(
    const string &config_wpt_file_path_, const string &body_frame_id_,
    const string &global_frame_id_, const double &spline_interval_) {
  YAML::Node config = YAML::LoadFile(config_wpt_file_path_);

  if (!config["racine_lines_file_paths"]) {
    throw std::runtime_error("Racing line config file is not properly settup.");
  }
  if (!config["pit_lines_file_path"]) {
    throw std::runtime_error("Pit line config file is not properly settup.");
  }

  std::string original_raceline_file_path =
      config["racine_lines_file_paths"]["origin_rl"].as<std::string>();
  std::string outter_raceline_file_path =
      config["racine_lines_file_paths"]["outter_rl"].as<std::string>();
  std::string center_raceline_file_path =
      config["racine_lines_file_paths"]["center_rl"].as<std::string>();
  std::string inner_raceline_file_path =
      config["racine_lines_file_paths"]["inner_rl"].as<std::string>();
  std::string pit_file_path = config["pit_lines_file_path"].as<std::string>();

  if (original_raceline_file_path.empty() ||
      outter_raceline_file_path.empty() || center_raceline_file_path.empty() ||
      inner_raceline_file_path.empty() || pit_file_path.empty()) {
    throw std::runtime_error("Waypoint file path is empty.");
  }

  vector<string> origin_rl_file_path_vec_tmp{original_raceline_file_path};
  vector<string> outter_rl_file_path_vec_tmp{outter_raceline_file_path};
  vector<string> center_rl_file_path_vec_tmp{center_raceline_file_path};
  vector<string> inner_rl_file_path_vec_tmp{inner_raceline_file_path};
  vector<string> pit_file_path_vec_tmp{pit_file_path};

  m_origin_rl_wpt_manager = std::make_shared<IWaypointManager>(
      origin_rl_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
  m_outter_rl_wpt_manager = std::make_shared<IWaypointManager>(
      outter_rl_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
  m_center_rl_wpt_manager = std::make_shared<IWaypointManager>(
      center_rl_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
  m_inner_rl_wpt_manager = std::make_shared<IWaypointManager>(
      inner_rl_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));

  m_wpt_manager_vec.push_back(m_origin_rl_wpt_manager);
  m_wpt_manager_vec.push_back(m_center_rl_wpt_manager);
  m_wpt_manager_vec.push_back(m_inner_rl_wpt_manager);
  m_wpt_manager_vec.push_back(m_center_rl_wpt_manager);
  m_wpt_manager_vec.push_back(m_origin_rl_wpt_manager);
  m_wpt_manager_vec.push_back(m_outter_rl_wpt_manager);

  m_cur_wpt_manager_idx = 0;

  m_pit_wpt_manager = std::make_shared<IWaypointManager>(
      pit_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
}

void WaypointManagerMissionSelective::setCurrentOdometry(
    const nav_msgs::msg::Odometry &ego_vehicle_odom) {
  m_cur_odom = ego_vehicle_odom;

  // m_wpt_manager_vec[m_cur_wpt_manager_idx]->setCurrentOdometry(
  //     ego_vehicle_odom);
  // m_pit_wpt_manager->setCurrentOdometry(ego_vehicle_odom);
  if (!m_odom_first_callbacked)
    m_odom_first_callbacked = true;
}

void WaypointManagerMissionSelective::setSystemStatus(
    const nif_msgs::msg::SystemStatus &sys_status) {
  m_cur_sys_status = sys_status;
  m_cur_mission_code = sys_status.mission_status;
}

void WaypointManagerMissionSelective::setObstacle(
    const visualization_msgs::msg::MarkerArray &obj_array) {
  m_obstacles = obj_array;
}

bool WaypointManagerMissionSelective::checkCollision(
    const visualization_msgs::msg::Marker &obj,
    const nav_msgs::msg::Path &path_in_body) const {
  if (obj.pose.position.x < 0.0) {
    return false;
  } else {
    for (const auto &pose : path_in_body.poses) {
      double dist = sqrt(pow(pose.pose.position.x - obj.pose.position.x, 2) +
                         pow(pose.pose.position.y - obj.pose.position.y, 2));

      if (dist < m_collision_check_dist) {
        // TODO : decide later
        // if object is close to the vehicle,
        // check heading term??
        return true;
      }
    }
  }
}

void WaypointManagerMissionSelective::calcMapTrack() {
  if (m_odom_first_callbacked) {
    if (m_cur_mission_code.mission_status_code ==
        m_cur_mission_code.MISSION_PIT_IN) {
      m_pit_wpt_manager->setCurrentOdometry(m_cur_odom);
      m_map_track_path_global = m_pit_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

    } else if (m_cur_mission_code.mission_status_code ==
               m_cur_mission_code.MISSION_STANDBY) {
      // NOTE : check out of track
      if (m_on_track_flg) {
        // if the vehicle is on the track
        bool collision_free_path_exist = false;
        for (int cnt = 0; cnt < m_wpt_manager_vec.size(); cnt++) {
          bool collision = false;
          int wpt_manager_idx =
              (m_cur_wpt_manager_idx + cnt) % m_wpt_manager_vec.size();
          m_wpt_manager_vec[wpt_manager_idx]->setCurrentOdometry(
              m_cur_odom);
          m_map_track_path_global =
              m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInGlobal();
          m_map_track_path_body =
              m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInBody();

          // collision check
          // pylon scale : 1m
          for (auto &marker : m_obstacles.markers) {
            if (checkCollision(marker, m_map_track_path_body)) {
              collision = true;
              break;
            }
          }
          if (!collision) {
            m_cur_wpt_manager_idx = wpt_manager_idx;
            collision_free_path_exist = true;
            break;
          }
        }
        if (!collision_free_path_exist) {
          // no collision free path, stop the car
          nav_msgs::msg::Path empty_path;
          m_map_track_path_global = empty_path;
          m_map_track_path_global.header.frame_id = "odom";
          m_map_track_path_body = empty_path;
          m_map_track_path_body.header.frame_id = "base_link";
        } else {
          m_map_track_path_global = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                        ->getDesiredMapTrackInGlobal();
          m_map_track_path_body = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                      ->getDesiredMapTrackInBody();
        }
      } else {
        // if the vehicle is out of the track
        m_pit_wpt_manager->setCurrentOdometry(m_cur_odom);
        m_map_track_path_global =
            m_pit_wpt_manager->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();
      }
    } else if (m_cur_mission_code.mission_status_code ==
                   m_cur_mission_code.MISSION_PIT_OUT ||
               m_cur_mission_code.mission_status_code ==
                   m_cur_mission_code.MISSION_PIT_STANDBY ||
               m_cur_mission_code.mission_status_code ==
                   m_cur_mission_code.MISSION_PIT_INIT) {
      m_pit_wpt_manager->setCurrentOdometry(m_cur_odom);
      m_map_track_path_global = m_pit_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

    } else if (m_cur_mission_code.mission_status_code ==
               m_cur_mission_code.MISSION_SLOW_DRIVE) {
      bool collision_free_path_exist = false;
      for (int cnt = 0; cnt < m_wpt_manager_vec.size(); cnt++) {
        bool collision = false;
        int wpt_manager_idx =
            (m_cur_wpt_manager_idx + cnt) % m_wpt_manager_vec.size();
        m_wpt_manager_vec[wpt_manager_idx]->setCurrentOdometry(
            m_cur_odom);
        m_map_track_path_global =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInGlobal();
        m_map_track_path_body =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInBody();

        // collision check
        // pylon scale : 1m
        for (auto &marker : m_obstacles.markers) {
          if (checkCollision(marker, m_map_track_path_body)) {
            collision = true;
            break;
          }
        }
        if (!collision) {
          m_cur_wpt_manager_idx = wpt_manager_idx;
          collision_free_path_exist = true;
          break;
        }
      }
      if (!collision_free_path_exist) {
        // no collision free path, stop the car
        nav_msgs::msg::Path empty_path;
        m_map_track_path_global = empty_path;
        m_map_track_path_global.header.frame_id = "odom";
        m_map_track_path_body = empty_path;
        m_map_track_path_body.header.frame_id = "base_link";
      } else {
        m_map_track_path_global = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                      ->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                    ->getDesiredMapTrackInBody();
      }
    } else {
      // nominal case
      bool collision_free_path_exist = false;
      for (int cnt = 0; cnt < m_wpt_manager_vec.size(); cnt++) {
        bool collision = false;
        int wpt_manager_idx =
            (m_cur_wpt_manager_idx + cnt) % m_wpt_manager_vec.size();
        m_wpt_manager_vec[wpt_manager_idx]->setCurrentOdometry(
            m_cur_odom);
        m_map_track_path_global =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInGlobal();
        m_map_track_path_body =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInBody();

        // collision check
        // pylon scale : 1m
        for (auto &marker : m_obstacles.markers) {
          if (checkCollision(marker, m_map_track_path_body)) {
            collision = true;
            break;
          }
        }
        if (!collision) {
          m_cur_wpt_manager_idx = wpt_manager_idx;
          collision_free_path_exist = true;
          break;
        }
      }
      if (!collision_free_path_exist) {
        // no collision free path, stop the car
        nav_msgs::msg::Path empty_path;
        m_map_track_path_global = empty_path;
        m_map_track_path_global.header.frame_id = "odom";
        m_map_track_path_body = empty_path;
        m_map_track_path_body.header.frame_id = "base_link";
      } else {
        m_map_track_path_global = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                      ->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                    ->getDesiredMapTrackInBody();
      }
    }

  } else {
    // odom is not setted yet
  }
}