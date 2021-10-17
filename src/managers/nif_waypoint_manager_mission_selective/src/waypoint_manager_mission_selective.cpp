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

WaypointManagerMissionSelective::WaypointManagerMissionSelective(
    const string &race_line_path_, const string &center_line_path_,
    const string &pitlane_line_path_, const string &body_frame_id_,
    const string &global_frame_id_, const double &spline_interval_) {

  vector<string> origin_rl_file_path_vec_tmp{race_line_path_};
  vector<string> outter_rl_file_path_vec_tmp{race_line_path_};
  vector<string> center_rl_file_path_vec_tmp{center_line_path_};
  vector<string> inner_rl_file_path_vec_tmp{center_line_path_};
  vector<string> pit_file_path_vec_tmp{pitlane_line_path_};

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
  calcMapTrack();
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

collide_info WaypointManagerMissionSelective::checkCollision(
    const visualization_msgs::msg::Marker &obj,
    const nav_msgs::msg::Path &path_in_body) const {
  collide_info output;
  output.collision_flg = false;
  output.collision_idx = INFINITY;
  if (obj.pose.position.x < 0.0) {
    return output;
  } else {
    int cnt = 0;
    for (const auto &pose : path_in_body.poses) {
      double dist = sqrt(pow(pose.pose.position.x - obj.pose.position.x, 2) +
                         pow(pose.pose.position.y - obj.pose.position.y, 2));

      cnt++;
      if (dist < m_collision_check_dist) {
        // TODO : decide later
        // if object is close to the vehicle,
        // check heading term??
        output.collision_flg = true;
        output.collision_idx = cnt;
        return output;
      }
    }
  }
  return output;
}

void WaypointManagerMissionSelective::calcMapTrack() {

  int before_idx = m_cur_wpt_manager_idx;

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
        std::vector<int> progress_vec;
        progress_vec.resize(m_wpt_manager_vec.size());

        for (int wpt_idx = 0; wpt_idx < m_wpt_manager_vec.size(); wpt_idx++) {
          int wpt_manager_idx =
              (m_cur_wpt_manager_idx + wpt_idx) % m_wpt_manager_vec.size();
          m_wpt_manager_vec[wpt_manager_idx]->setCurrentOdometry(m_cur_odom);
          m_map_track_path_global =
              m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInGlobal();
          m_map_track_path_body =
              m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInBody();

          // collision check
          // pylon scale : 1m
          int min_progree_idx = 10000000;
          bool collision = false;
          for (auto &marker : m_obstacles.markers) {
            collide_info out = checkCollision(marker, m_map_track_path_body);
            if (out.collision_flg) {
              collision = true;
              if (min_progree_idx > out.collision_idx) {
                min_progree_idx = out.collision_idx;
              }
            }
          }

          if (!collision) {
            // collision free path exist
            collision_free_path_exist = true;
            m_cur_wpt_manager_idx = wpt_idx;
            break;
          }
          progress_vec.push_back(min_progree_idx);
        }

        if (!collision_free_path_exist) {
          // no collision free path, stop the car

          int max_progress_wpt_idx =
              std::max_element(progress_vec.begin(), progress_vec.end()) -
              progress_vec.begin();

          int max_progess =
              *max_element(std::begin(progress_vec), std::end(progress_vec));
          int cur_wpt_progress = progress_vec[m_cur_wpt_manager_idx];

          // TODO : should be tuned
          if (abs(max_progess - cur_wpt_progress) > 10) {
            m_cur_wpt_manager_idx = max_progress_wpt_idx;
          } else {
            // no lane change
          }
          m_map_track_path_global = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                        ->getDesiredMapTrackInGlobal();
          m_map_track_path_body = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                      ->getDesiredMapTrackInBody();
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
      std::vector<int> progress_vec;
      progress_vec.resize(m_wpt_manager_vec.size());

      for (int wpt_idx = 0; wpt_idx < m_wpt_manager_vec.size(); wpt_idx++) {
        int wpt_manager_idx =
            (m_cur_wpt_manager_idx + wpt_idx) % m_wpt_manager_vec.size();
        m_wpt_manager_vec[wpt_manager_idx]->setCurrentOdometry(m_cur_odom);
        m_map_track_path_global =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInGlobal();
        m_map_track_path_body =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInBody();

        // collision check
        // pylon scale : 1m
        int min_progree_idx = 10000000;
        bool collision = false;
        for (auto &marker : m_obstacles.markers) {
          collide_info out = checkCollision(marker, m_map_track_path_body);
          if (out.collision_flg) {
            collision = true;
            if (min_progree_idx > out.collision_idx) {
              min_progree_idx = out.collision_idx;
            }
          }
        }
        if (!collision) {
          // collision free path exist
          collision_free_path_exist = true;
          m_cur_wpt_manager_idx = wpt_idx;
          break;
        } else {
          if (wpt_idx == 0) {
            std::cout << "current wpt has collision" << std::endl;
          }
        }
        progress_vec.push_back(min_progree_idx);
      }

      if (!collision_free_path_exist) {
        // no collision free path, stop the car
        std::cout << "no collision free path, stop the car" << std::endl;

        int max_progress_wpt_idx =
            std::max_element(progress_vec.begin(), progress_vec.end()) -
            progress_vec.begin();

        int max_progess =
            *max_element(std::begin(progress_vec), std::end(progress_vec));
        int cur_wpt_progress = progress_vec[m_cur_wpt_manager_idx];

        // TODO : should be tuned
        if (abs(max_progess - cur_wpt_progress) > 10) {
          m_cur_wpt_manager_idx = max_progress_wpt_idx;
        } else {
          // no lane change
        }
        m_map_track_path_global = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                      ->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                    ->getDesiredMapTrackInBody();
      } else {
        m_map_track_path_global = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                      ->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                    ->getDesiredMapTrackInBody();
      }
    } else {
      // if the vehicle is on the track
      bool collision_free_path_exist = false;
      std::vector<int> progress_vec;
      progress_vec.resize(m_wpt_manager_vec.size());

      for (int wpt_idx = 0; wpt_idx < m_wpt_manager_vec.size(); wpt_idx++) {
        bool collision = false;
        int wpt_manager_idx =
            (m_cur_wpt_manager_idx + wpt_idx) % m_wpt_manager_vec.size();
        m_wpt_manager_vec[wpt_manager_idx]->setCurrentOdometry(m_cur_odom);
        m_map_track_path_global =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInGlobal();
        m_map_track_path_body =
            m_wpt_manager_vec[wpt_manager_idx]->getDesiredMapTrackInBody();

        // collision check
        // pylon scale : 1m
        int min_progree_idx = 10000000;
        for (auto &marker : m_obstacles.markers) {
          collide_info out = checkCollision(marker, m_map_track_path_body);
          if (out.collision_flg) {
            collision = true;
            if (min_progree_idx > out.collision_idx) {
              min_progree_idx = out.collision_idx;
            }
          }
        }
        if (!collision) {
          // collision free path exist
          collision_free_path_exist = true;
          m_cur_wpt_manager_idx = wpt_idx;
          break;
        }
        progress_vec.push_back(min_progree_idx);
      }

      if (!collision_free_path_exist) {
        // no collision free path, stop the car

        int max_progress_wpt_idx =
            std::max_element(progress_vec.begin(), progress_vec.end()) -
            progress_vec.begin();

        int max_progess =
            *max_element(std::begin(progress_vec), std::end(progress_vec));
        int cur_wpt_progress = progress_vec[m_cur_wpt_manager_idx];

        // TODO : should be tuned
        if (abs(max_progess - cur_wpt_progress) > 10) {
          m_cur_wpt_manager_idx = max_progress_wpt_idx;
        } else {
          // no lane change
        }
        m_map_track_path_global = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                      ->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_wpt_manager_vec[m_cur_wpt_manager_idx]
                                    ->getDesiredMapTrackInBody();
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

  if (m_cur_wpt_manager_idx - before_idx != 0) {
    std::cout << "----------------" << std::endl;
    std::cout << "wapoint changed" << std::endl;
    std::cout << "before_idx : " << before_idx << std::endl;
    std::cout << "m_cur_wpt_manager_idx : " << m_cur_wpt_manager_idx
              << std::endl;
    std::cout << "mission status : " << m_cur_mission_code.mission_status_code
              << std::endl;
  } else {
    // std::cout << "-" << std::endl;
  }
}