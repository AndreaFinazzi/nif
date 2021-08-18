#include "nif_control_pure_pursuit_nodes/pure_pursuit.h"

PurePursuit::PurePursuit(double min_lookahead_dist_ = 10.0,
                         double max_lookahead_dist_ = 50.0,
                         double lookahead_speed_ratio_ = 1.0,
                         bool use_lpf_flg_ = false,
                         double lfp_gain_ = 0.5,
                         bool is_steer_sign_invert = true)
  : m_min_lookahead_dist(min_lookahead_dist_),
    m_max_lookahead_dist(max_lookahead_dist_),
    m_lookahead_speed_ratio(lookahead_speed_ratio_),
    m_lpf_gain(lfp_gain_),
    m_use_lpf_flg(use_lpf_flg_),
    m_steer_sign_flip_flg(is_steer_sign_invert) {}

void PurePursuit::calcLookAheadDist(double cur_vel_) {
  // TODO: do something algorithmic
  // Case 1. speed proportional lookahead distance settup
  m_lookahead_dist = cur_vel_ * m_lookahead_speed_ratio;

  std::clamp(m_lookahead_dist, m_min_lookahead_dist, m_max_lookahead_dist);
}

void PurePursuit::calcLookAheadDist() {
  double lookahead_dist;
  // TODO: do something algorithmic
  // Case 1. speed proportional lookahead distance settup
  double cur_vel_ = sqrt(pow(m_ego_pose_in_global.twist.twist.linear.x, 2) +
                         pow(m_ego_pose_in_global.twist.twist.linear.y, 2));
  lookahead_dist = cur_vel_ * m_lookahead_speed_ratio;
  m_lookahead_dist = lookahead_dist;

  std::clamp(m_lookahead_dist, m_min_lookahead_dist, m_max_lookahead_dist);
}

void PurePursuit::calcCurvature() {
  double curvature;
  double alpha;
  double curvature_sign;
  // TODO: do something algorithmic
  geometry_msgs::msg::PoseStamped ego_pose_in_body;
  double dist_to_control_pt =
      nif::common::utils::geometry::calEuclideanDistance(ego_pose_in_body,
                                                         m_control_pt_in_body);

  double eps = m_control_pt_in_body.pose.position.x > 0 ? 0.001 : -0.001;

  // TODO : Something is wrong... very sharp u-turn case? We need backup plan,
  // not just tun off the controller (Maybe we can cope with this kind of
  // issues in control safety layer?)

  alpha = atan2(m_control_pt_in_body.pose.position.y,
                m_control_pt_in_body.pose.position.x + eps);

  curvature = 1.0 /
      (fabs(dist_to_control_pt / (2 * sin(alpha))) +
       nif::common::constants::numeric::EPSILON);
  curvature_sign = alpha < 0 ? -1.0 : 1.0;
  m_control_pt_curvature_magnitude = curvature;
  m_control_pt_curvature_sign = curvature_sign;
}

void PurePursuit::setVehicleStatus(nif::common::msgs::Odometry ego_odom_) {
  m_ego_pose_in_global = ego_odom_;
  m_veh_current_velocity_mps =
      sqrt(pow(m_ego_pose_in_global.twist.twist.linear.x, 2) +
           pow(m_ego_pose_in_global.twist.twist.linear.y, 2));
}

void PurePursuit::calcLookAheadIndex(
    const nav_msgs::msg::Path& map_track_path_in_global_,
    const double& lookahead_dist_) {
  // TODO: Search API similar to scipy (finding closest point)
//  m_lookahead_pt_in_body.pose.position.x = lookahead_dist_;
  m_lookahead_pt_in_body.pose.position.x = 15.0;
  m_lookahead_pt_in_body.pose.position.y = 0.0;
  m_lookahead_pt_in_body.pose.position.z = 0.0;
  m_lookahead_pt_in_body.pose.orientation.x = 0.0;
  m_lookahead_pt_in_body.pose.orientation.y = 0.0;
  m_lookahead_pt_in_body.pose.orientation.z = 0.0;
  m_lookahead_pt_in_body.pose.orientation.w = 1.0;

  // Using utils, convert coordination btw body and global
  m_lookahead_pt_in_global =
      nif::common::utils::coordination::getPtBodytoGlobal(
          m_ego_pose_in_global, m_lookahead_pt_in_body);

  // closest point search (TODO: should be updated with api)
  double min_dist = nif::common::constants::numeric::INF;
  int min_dist_idx = 0;
  for (int idx = 0; idx < map_track_path_in_global_.poses.size(); idx++) {
    double dist =
        sqrt(pow(m_lookahead_pt_in_global.pose.position.x -
                     map_track_path_in_global_.poses[idx].pose.position.x,
                 2) +
             pow(m_lookahead_pt_in_global.pose.position.y -
                     map_track_path_in_global_.poses[idx].pose.position.y,
                 2));
    if (dist < min_dist) {
      min_dist = dist;
      min_dist_idx = idx;
    }
  }
  m_control_pt_idx = min_dist_idx;
  m_control_pt_in_global = map_track_path_in_global_.poses[m_control_pt_idx];
  m_control_pt_in_body = nif::common::utils::coordination::getPtGlobaltoBody(
      m_ego_pose_in_global, m_control_pt_in_global);
}

void PurePursuit::setCrossTrackError(
    nav_msgs::msg::Path& map_track_in_global_) {
  geometry_msgs::msg::PoseStamped map_track_pt_idx_zero_in_body;
  // Using utils, convert coordination btw body and global
  map_track_pt_idx_zero_in_body =
      nif::common::utils::coordination::getPtGlobaltoBody(
          m_ego_pose_in_global, map_track_in_global_.poses[0]);

  // TODO: sign should be checked
  m_current_crosstrack_error = map_track_pt_idx_zero_in_body.pose.position.y;
}

void PurePursuit::setLookAheadLateralError(
    const nav_msgs::msg::Path& map_track_in_global_, const int lookahead_idx) {
  m_control_pt_in_global = map_track_in_global_.poses[lookahead_idx];

  // Using utils, convert coordination btw body and global
  m_control_pt_in_body = nif::common::utils::coordination::getPtGlobaltoBody(
      m_ego_pose_in_global, m_control_pt_in_global);

  // TODO: sign should be checked
  m_lookahead_lateral_error = m_control_pt_in_body.pose.position.y;
}

void PurePursuit::calcSteerCmd() {
  m_veh_cmd_steer_rad = m_control_pt_curvature_sign *
      atan2(nif::common::vehicle_param::VEH_WHEEL_BASE,
            2 * m_control_pt_curvature_magnitude -
                nif::common::vehicle_param::VEH_WHEEL_BASE);

  if (m_use_lpf_flg == true) {
    m_veh_cmd_steer_rad = (1.0 - m_lpf_gain) * m_veh_cmd_steer_rad +
        m_lpf_gain * m_veh_cmd_prev_steer_rad;
  }
  m_veh_cmd_steerwheel_rad =
      m_veh_cmd_steer_rad * nif::common::vehicle_param::STEERING_RATIO;

  if (m_steer_sign_flip_flg) {
    m_veh_cmd_steerwheel_rad = -m_veh_cmd_steerwheel_rad;
  }

  // store previous control command
  m_veh_cmd_prev_steer_rad = m_veh_cmd_steer_rad;
  m_veh_cmd_prev_steerwheel_rad =
      m_veh_cmd_prev_steer_rad * nif::common::vehicle_param::STEERING_RATIO;
}
