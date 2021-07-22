#ifndef PURE_PURSUIT_H
#define PURE_PURSUIT_H

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_common/vehicle_model.h"
#include "nif_utils/utils.h"

// C++ includes
#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <mutex>
#include <thread>
#include <vector>

class PurePursuit {
public:
  PurePursuit(double min_look_ahead_dist_,
              double max_look_ahead_dist_,
              double lookahead_speed_ratio_,
              bool use_lpf_flg_,
              double lfp_gain_,
              bool is_steer_sign_invert);
  ~PurePursuit() {}

private:
  nif::common::msgs::Odometry m_ego_pose_in_global;

  double m_veh_current_velocity_mps = 0.0;

  double m_veh_cmd_steer_rad =
      0.0; // wheel ratation angle command in radian (current)
  double m_veh_cmd_prev_steer_rad =
      0.0; // wheel ratation angle command in radian (previous)

  double m_veh_cmd_steerwheel_rad =
      0.0; // Steering wheel ratation angle command in radian (current) -->
           // m_veh_cmd_steer_rad * steering ratio
  double m_veh_cmd_prev_steerwheel_rad =
      0.0; // Steering wheel ratation angle command in radian (previous)
           // m_veh_cmd_prev_steer_rad * steering ratio

  double m_max_lookahead_dist, m_min_lookahead_dist;
  double m_lookahead_speed_ratio, m_lookahead_dist;
  double m_tau; // time constant w.r.t. perception, planning (delay compensation)
  double m_T;   // time delay w.r.t. low-level actuator (delay compensation)
  double m_del_lookahead_dist; // increment of lookahead dist during delay compensation

  double m_control_pt_curvature_magnitude; // Curvature magnitude upto control
                                           // point
  int m_control_pt_curvature_sign;         // Curvature sign upto control point
  int m_control_pt_idx; // control point index in map track path

  double m_current_crosstrack_error = 0.0; // cross-track error in meter
  double m_lookahead_lateral_error =
      0.0; // lateral-wise displacement btw lookahead point and control point

  nav_msgs::msg::Path m_map_track_in_global;
  geometry_msgs::msg::PoseStamped m_lookahead_pt_in_global,
      m_lookahead_pt_in_body;
  geometry_msgs::msg::PoseStamped m_control_pt_in_global, m_control_pt_in_body;

  double m_lpf_gain;
  bool m_use_lpf_flg;

  bool m_steer_sign_flip_flg;

private:
  PurePursuit() {}

public:
  void calcLookAheadIndex(const nav_msgs::msg::Path& map_track_path_,
                          const double& lookahead_dist_);
  void setCrossTrackError(nav_msgs::msg::Path& map_track_in_global_);
  void setLookAheadLateralError(const nav_msgs::msg::Path& map_track_in_global_,
                                const int lookahead_idx);

  void calcLookAheadDist(); // using odometry twist linear velocity
  void calcLookAheadDist(double cur_vel_);
  void calcSteerCmd();
  void calcCurvature();
  void
  setVehicleStatus(nif::common::msgs::Odometry
                       ego_odom_); // set both (vehicle status and position)

  double delayCompensation(double lookahead_dist_, double cur_vel_);
  double getLookAheadDist() {
    return m_lookahead_dist;
  }
  double getCurvature() {
    return m_control_pt_curvature_magnitude;
  }
  double getSteerCmd() {
    calcLookAheadDist();
    calcLookAheadIndex(m_map_track_in_global, m_lookahead_dist);
    calcCurvature();
    calcSteerCmd();
    return m_veh_cmd_steer_rad;
  }
  void setMapTrack(const nav_msgs::msg::Path& map_track_path_) {
    m_map_track_in_global = map_track_path_;
  }
  int getLookAheadIndex() {
    return m_control_pt_idx;
  }
  nav_msgs::msg::Path getMapTrack() {
    return m_map_track_in_global;
  }
  void setLPFGain(double gain_) {
    m_lpf_gain = gain_;
  }
  double getLPFGain() {
    return m_lpf_gain;
  }
  double getCrossTrackError() {
    return m_current_crosstrack_error;
  }
  double getLookAheadLateralError() {
    return m_lookahead_lateral_error;
  }

  void setLookAheadSpeedRatio(double lookahead_speed_ratio_) {
    m_lookahead_speed_ratio = lookahead_speed_ratio_;
  }
  double getLookAheadSpeedRatio() {
    return m_lookahead_speed_ratio;
  }

  void setMinLookaheadDist(double min_la_dist_) {
    m_min_lookahead_dist = min_la_dist_;
  }
  double getMinLookaheadDist() {
    return m_min_lookahead_dist;
  }
  void setMaxLookaheadDist(double max_la_dist_) {
    m_max_lookahead_dist = max_la_dist_;
  }
  double getMaxLookaheadDist() {
    return m_max_lookahead_dist;
  }
};
#endif // PURE_PURSUIT_H
