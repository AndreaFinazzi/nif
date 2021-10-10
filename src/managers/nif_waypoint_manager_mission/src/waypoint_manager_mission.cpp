//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#include "nif_waypoint_manager_mission/waypoint_manager_mission.h"

WaypointManagerMission::WaypointManagerMission(const string &rl_wpt_file_path_,
                                               const string &pit_wpt_file_path_,
                                               const string &body_frame_id_,
                                               const string &global_frame_id_,
                                               const double &spline_interval_)
    : m_rl_wpt_file_path(rl_wpt_file_path_),
      m_pit_wpt_file_path(pit_wpt_file_path_),
      m_body_frame_id_str(body_frame_id_),
      m_global_frame_id_str(global_frame_id_),
      m_spline_interval(spline_interval_) {

  m_map_track_path_global.header.frame_id = m_global_frame_id_str;
  m_map_track_path_body.header.frame_id = m_body_frame_id_str;
  m_collision_avoidance_path_body.header.frame_id = m_body_frame_id_str;

  m_frenet_generator = std::make_shared<FrenetPathGenerator>();
  m_frenet_cost_calculator =
      std::make_shared<nif::planning::cost_calculator::costCalculator>();

  vector<string> rl_file_path_vec_tmp{m_rl_wpt_file_path};
  vector<string> pit_file_path_vec_tmp{m_pit_wpt_file_path};

  m_odom_first_callbacked = false;

  m_rl_wpt_manager = std::make_shared<IWaypointManager>(
      rl_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
  m_pit_wpt_manager = std::make_shared<IWaypointManager>(
      pit_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
}

void WaypointManagerMission::setCurrentOdometry(
    const nav_msgs::msg::Odometry &ego_vehicle_odom) {
  m_cur_odom = ego_vehicle_odom;
  m_rl_wpt_manager->setCurrentOdometry(ego_vehicle_odom);
  m_pit_wpt_manager->setCurrentOdometry(ego_vehicle_odom);
  if (!m_odom_first_callbacked)
    m_odom_first_callbacked = true;
}

void WaypointManagerMission::setSystemStatus(
    const nif_msgs::msg::SystemStatus &sys_status) {
  m_cur_sys_status = sys_status;
  m_cur_mission_code = sys_status.mission_status;
}

void WaypointManagerMission::calcMapTrack() {
  if (m_odom_first_callbacked) {
    if (m_cur_mission_code.mission_status_code ==
        m_cur_mission_code.MISSION_PIT_IN) {
      m_map_track_path_global = m_pit_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

      // ///////////////////////
      // NOTE : collision check
      // NOTE : Need to load frenet generator default params. it can cause an
      // error.
      // ///////////////////////
      std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
          m_frenet_generator->applyCubicSpliner_2d_ros(m_map_track_path_body,
                                                       1.0);
      double cte = -1 * splined_y[0];
      double target_vel =
          m_planning_vel_default + m_cur_odom.twist.twist.linear.x; // [m/s]
      std::tuple<std::shared_ptr<FrenetPath>,
                 std::vector<std::shared_ptr<FrenetPath>>>
          frenet_path_generation_result =
              m_frenet_generator->calcOptimalFrenetPathByMode(
                  FRENET_GEN_MODE::MULTIPLE_LAT_FPS, cubic_spliner_2D_xy, 0.0,
                  0.0, 0.0, m_cur_odom.twist.twist.linear.x, 0.0, 0.0, 0.0,
                  m_left_side_sampling_width, m_right_side_sampling_width,
                  target_vel, 0.0, 0.0, m_sampling_width_d, m_planning_t, 0.1);
      std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
          std::get<1>(frenet_path_generation_result);
      // NOTE : must call the function "setReferencePath" beforehand
      // "setFrenetPathArray"
      this->m_frenet_cost_calculator->setReferencePath(m_map_track_path_body);
      this->m_frenet_cost_calculator->setFrenetPathArray(frenet_paths);
      m_collision_avoidance_fp_body_ptr =
          this->m_frenet_cost_calculator->getMincostFrenetPath();

    } else if (m_cur_mission_code.mission_status_code ==
               m_cur_mission_code.MISSION_STANDBY) {
      // NOTE : check out of track
      if (m_on_track_flg) {
        // if the vehicle is on the track
        m_map_track_path_global =
            m_rl_wpt_manager->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_rl_wpt_manager->getDesiredMapTrackInBody();
      } else {
        // if the vehicle is out of the track
        m_map_track_path_global =
            m_pit_wpt_manager->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

        // NOTE : collision check
        // std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
        //     frenet_path_generator->applyCubicSpliner_2d_ros(
        //         m_map_track_path_body, 1.0);
      }
    } else if (m_cur_mission_code.mission_status_code ==
                   m_cur_mission_code.MISSION_PIT_OUT ||
               m_cur_mission_code.mission_status_code ==
                   m_cur_mission_code.MISSION_PIT_STANDBY ||
               m_cur_mission_code.mission_status_code ==
                   m_cur_mission_code.MISSION_PIT_INIT) {
      m_map_track_path_global = m_pit_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

      // NOTE : collision check
      // std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
      //     frenet_path_generator->applyCubicSpliner_2d_ros(m_map_track_path_body,
      //                                                     1.0);

    } else if (m_cur_mission_code.mission_status_code ==
               m_cur_mission_code.MISSION_SLOW_DRIVE) {
      m_map_track_path_global = m_rl_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_rl_wpt_manager->getDesiredMapTrackInBody();
    } else if (m_cur_mission_code.mission_status_code ==
               m_cur_mission_code.MISSION_COLLISION_AVOIDNACE) {
      m_map_track_path_global = m_rl_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_rl_wpt_manager->getDesiredMapTrackInBody();

      // : collision check
      // std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
      //     frenet_path_generator->applyCubicSpliner_2d_ros(m_map_track_path_body,
      //                                                     1.0);

    } else {
      // nominal case
      m_map_track_path_global = m_rl_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_rl_wpt_manager->getDesiredMapTrackInBody();
    }

  } else {
    // odom is not setted yet
  }
}