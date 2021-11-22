//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#ifndef FRENET_BASED_PREDICTOR_H
#define FRENET_BASED_PREDICTOR_H

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/types.h"
#include "nif_waypoint_manager_common/c_wpt.h"

#include <array>

using namespace nif::common::msgs;

namespace nif {
namespace perception {

/**
 * TODO: define precisely its behavior
 * OpponentPredictor is responsible for assigning an ID to each detected object
 * (from PerceptionNode);
 */
class OpponentPredictor {
public:
  OpponentPredictor(std::string target_ref_file_path);
  ~OpponentPredictor();

  void setOpponentStatus(
      const nif::common::msgs::PerceptionResult::SharedPtr oppo_status,
      const nav_msgs::msg::Odometry &ego_status);
  nif::common::msgs::NIF_Trajectory getPredictiveTrajectory() {
    return m_predictive_oppo_trajectory;
  }

private:
  nav_msgs::msg::Path m_target_path;
  double m_target_path_sampling_interval = 2.0;
  PerceptionResult m_opponent_status;
  NIF_Trajectory m_predictive_oppo_trajectory;
  std::string m_target_ref_file_path;

  nav_msgs::msg::Odometry m_ego_status;

  bool init_success_flg;
  c_wpt *obj_tmp;
};

} // namespace perception
} // namespace nif
#endif // FRENET_BASED_PREDICTOR_H
