//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_dynamic_planning_nodes/dynamic_planning_node.h"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"
#include <stdlib.h>

using namespace nif::planning;
using namespace std;

DynamicPlannerNode::DynamicPlannerNode(const std::string& node_name_,
                                       const std::string& planning_config_file_,
                                       const int& num_cadidate_paths_)
  : IBaseNode(node_name_, common::NodeType::PLANNING) {
  m_config_load_success = false;
  m_planning_config_file_path = planning_config_file_;

  // Load param
  loadConfig(planning_config_file_);
  m_config_load_success = true;
  m_det_callback_first_run = true;
  m_oppo_pred_callback_first_run = true;
  m_timer_callback_first_run = true;

  // Init trajectories
  initTrajectory();

  // Init spliner & spline modeling for every overtaking path candidates
  for (int candidate_idx = 0; candidate_idx < m_num_overtaking_candidates;
       candidate_idx++) {
    auto wpt_xy =
        loadCSVfile(m_overtaking_candidates_file_path_vec[candidate_idx]);
    auto path_x_vec = get<0>(wpt_xy);
    auto path_y_vec = get<1>(wpt_xy);
    auto splined_result = m_frenet_generator_ptr->apply_cubic_spliner(
        path_x_vec, path_y_vec, 1.0);

    m_overtaking_candidates_spline_data_vec.push_back(splined_result);
  }

  // Initialize subscribers & publisher
  m_det_sub = this->create_subscription<nif_msgs::msg::Perception3D>(
      "det_topic_name",
      common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::detectionResultCallback,
                this,
                std::placeholders::_1));
  m_maptrack_body_sub = this->create_subscription<nav_msgs::msg::Path>(
      "maptrack_body_topic_name",
      common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::mapTrackBodyCallback,
                this,
                std::placeholders::_1));
  m_maptrack_global_sub = this->create_subscription<nav_msgs::msg::Path>(
      "maptrack_global_topic_name",
      common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::mapTrackGlobalCallback,
                this,
                std::placeholders::_1));
  m_oppo_pred_sub = this->create_subscription<nif_msgs::msg::DynamicTrajectory>(
      "oppo_traj_topic_name",
      common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::predictionResultCallback,
                this,
                std::placeholders::_1));

  m_planner_timer = this->create_wall_timer(
      20ms, std::bind(&DynamicPlannerNode::timer_callback, this)); // 50 hz
}

void DynamicPlannerNode::loadConfig(const std::string& planning_config_file_) {
  m_planning_config_file_path =
      this->get_parameter("planning_config_file_path").as_string();

  if (m_planning_config_file_path.empty())
    throw std::runtime_error(
        "Parameter m_planning_config_file_path not declared, or empty.");

  RCLCPP_INFO(get_logger(),
              "Loading planning params: %s",
              m_planning_config_file_path.c_str());

  YAML::Node config = YAML::LoadFile(m_planning_config_file_path);

  if (!config["path_candidates_param"]) {
    throw std::runtime_error(
        "path_candidates_param field not defined in config file.");
  }
  if (!config["planning_params"]) {
    throw std::runtime_error(
        "planning_params field not defined in config file.");
  }

  YAML::Node path_candidates_params = config["path_candidates_param"];

  m_overtaking_candidates_file_path_vec =
      path_candidates_params["overtaking_candidate_path_array"]
          .as<std::vector<std::string>>();
  m_overtaking_candidates_alias_vec =
      path_candidates_params["overtaking_candidate_path_alias_array"]
          .as<std::vector<std::string>>();

  // Size check (file_path - path_alias)
  if (m_overtaking_candidates_file_path_vec.size() !=
      m_overtaking_candidates_alias_vec.size()) {
    throw std::runtime_error(
        "path_candidates_param is not properly setted. Check config file.");
  }

  m_num_overtaking_candidates = m_overtaking_candidates_file_path_vec.size();

  YAML::Node planning_params = config["planning_params"];
  m_config_planning_horizon =
      planning_params["planning_horizon_t"].as<double>();
  m_config_planning_dt = planning_params["planning_dt"].as<double>();
  m_config_max_accel = planning_params["max_accel"].as<double>();
  m_config_overtaking_longitudinal_margin =
      planning_params["overtaking_longitudinal_margin"].as<double>();
  m_config_overtaking_lateral_margin =
      planning_params["overtaking_lateral_margin"].as<double>();
  m_config_merging_longitudinal_margin =
      planning_params["merging_longitudinal_margin"].as<double>();

  // minimal checking
  if (m_config_planning_dt <= 0.0) {
    throw std::runtime_error(
        "m_config_planning_dt can not be less than zero. Check config file.");
  }

  if (m_config_planning_horizon < m_config_planning_dt) {
    throw std::runtime_error("m_config_planning_horizon can not be shorter "
                             "than m_config_planning_dt. Check config file.");
  }

  if (m_config_max_accel <= 0.0) {
    throw std::runtime_error(
        "m_config_max_accel can not be less than zero. Check config file.");
  }

  if (m_config_overtaking_longitudinal_margin <= 0.0 ||
      m_config_overtaking_lateral_margin <= 0.0 ||
      m_config_merging_longitudinal_margin <= 0.0) {
    throw std::runtime_error(
        "Safety margin can not be less than zero. Check config file.");
  }
}

tuple<vector<double>, vector<double>>
DynamicPlannerNode::loadCSVfile(const std::string& wpt_file_path_) {
  ifstream inputFile(wpt_file_path_);
  vector<double> vec_x, vec_y;
  while (inputFile) {
    string s;
    if (!getline(inputFile, s))
      break;
    if (s[0] != '#') {
      istringstream ss(s);
      int cnt = 0;
      bool nan_flg = false;
      while (ss) {
        string line;
        if (!getline(ss, line, ','))
          break;
        try {
          if (cnt == 0) {
            vec_x.push_back(stof(line));
          } else if (cnt == 1) {
            vec_y.push_back(stof(line));
          }
        } catch (const invalid_argument e) {
          cout << "NaN found in file " << wpt_file_path_ << endl;
          e.what();
          nan_flg = true;
        }
        cnt++;
      }
    }
  }

  if (!inputFile.eof()) {
    cerr << "Could not read file " << wpt_file_path_ << "\n";
    __throw_invalid_argument("File not found.");
  }

  if (vec_x.size() == 0 || vec_y.size() == 0 ||
      (vec_x.size() != vec_y.size())) {
    __throw_invalid_argument("WPT SIZE ERROR.");
  }

  return std::make_tuple(vec_x, vec_y);
}

void DynamicPlannerNode::detectionResultCallback(
    const nif_msgs::msg::Perception3D::SharedPtr msg) {
  if (m_det_callback_first_run) {
    m_cur_det = *msg;
    m_det_callback_first_run = false;
  } else {
    m_prev_det = m_cur_det;
    m_cur_det = *msg;
  }
}
void DynamicPlannerNode::mapTrackBodyCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  m_maptrack_body = *msg;
}
void DynamicPlannerNode::mapTrackGlobalCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  m_maptrack_global = *msg;
}
void DynamicPlannerNode::predictionResultCallback(
    const nif_msgs::msg::DynamicTrajectory::SharedPtr msg) {
  if (m_oppo_pred_callback_first_run) {
    m_cur_oppo_pred_result = *msg;
    m_oppo_pred_callback_first_run = false;
  } else {
    m_prev_oppo_pred_result = m_cur_oppo_pred_result;
    m_cur_oppo_pred_result = *msg;
  }
}

void DynamicPlannerNode::timer_callback() {
  if (m_timer_callback_first_run) {
    m_cur_decision_type = PLANNING_DECISION_TYPE::STRAIGHT;
    m_cur_overtaking_stage = PLANNING_STAGE_TYPE::DRIVING;
    m_timer_callback_first_run = false;
  }

  m_prev_decision_type = m_cur_decision_type;
  m_prev_overtaking_stage = m_cur_overtaking_stage;
  m_prev_ego_planned_result_body = m_cur_ego_planned_result_body;
  m_prev_ego_planned_result_global = m_cur_ego_planned_result_global;

  if (m_emergency_flg) {
    // TODO : do something, safe stop, emergency trajectory planning
    m_cur_decision_type = PLANNING_DECISION_TYPE::ESTOP;
    m_cur_overtaking_stage = PLANNING_STAGE_TYPE::DRIVING;

    // TODO : temporary just sending empty traj
    initTrajectory(); // all path points are assigned with zero
    publishTrajectory();
    return;
  }

  if (m_cur_det.id == -1) {
    // TODO : invalid or empty detection result --> this should be implemented
    // in the perception side
    m_cur_decision_type = PLANNING_DECISION_TYPE::STRAIGHT;
    m_cur_overtaking_stage = PLANNING_STAGE_TYPE::DRIVING;

    // By pass maptrack to velocity planner
    // TODO : currently timestamp arry is uniformly sampled based on planning
    // horizon
    // In the velocity planner side, just drive as fast as possible when
    // decision type is straight.
    m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
    m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
    publishTrajectory();

  } else {
    // valid detection, there is a opponent in front.
    if (!m_overtake_allowed_flg) {
      // No need to calculate for overtaking
      // ACC, stay behind of the opponent
      m_cur_decision_type = PLANNING_DECISION_TYPE::FOLLOW;
      m_cur_overtaking_stage = PLANNING_STAGE_TYPE::DRIVING;

      // TODO : ACC when the decision type is FOLLOW
      m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
      m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
      publishTrajectory();
    } else {
      // TODO : previous driving mode should be integrated somehow.
      // Posible cases
      // 1. Keep racing line and chase the opponent as fast as we can.
      // 2. If we are close or faster enough, start to overtake. Checking right
      // and left side if the previous driving mode was STRAIGHT / FOLLOW /.
      // else, (ex: the previous mode was already START_OVERTAKING,
      // SIDE_BY_SIDE), check the collision based on that.
    }
  }
}

void DynamicPlannerNode::publishTrajectory() {
  m_cur_ego_planned_result_body.header.stamp = this->now();
  m_cur_ego_planned_result_global.header.stamp = this->now();

  m_cur_ego_planned_result_body.trajectory_type =
      nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;
  m_cur_ego_planned_result_global.trajectory_type =
      nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;

  if (m_cur_decision_type == PLANNING_DECISION_TYPE::STRAIGHT) {
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_STRAIGHT;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_STRAIGHT;
  } else if (m_cur_decision_type == PLANNING_DECISION_TYPE::FOLLOW) {
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_FOLLOW;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_FOLLOW;
  } else if (m_cur_decision_type == PLANNING_DECISION_TYPE::RIGHT) {
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_RIGHT;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_RIGHT;
  } else if (m_cur_decision_type == PLANNING_DECISION_TYPE::LEFT) {
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_LEFT;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_LEFT;
  } else {
    // defualt : estop
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_ESTOP;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_ESTOP;
  }

  if (m_cur_overtaking_stage == PLANNING_STAGE_TYPE::DRIVING) {
    m_cur_ego_planned_result_body.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_DRIVING;
    m_cur_ego_planned_result_global.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_DRIVING;
  } else if (m_cur_overtaking_stage == PLANNING_STAGE_TYPE::START_OVERTAKING) {
    m_cur_ego_planned_result_body.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_START_OVERTAKING;
    m_cur_ego_planned_result_global.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_START_OVERTAKING;
  } else if (m_cur_overtaking_stage == PLANNING_STAGE_TYPE::SIDE_BY_SIDE) {
    m_cur_ego_planned_result_body.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_SIDE_BY_SIDE;
    m_cur_ego_planned_result_global.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_SIDE_BY_SIDE;
  } else if (m_cur_overtaking_stage == PLANNING_STAGE_TYPE::FINISH_OVERTAKING) {
    m_cur_ego_planned_result_body.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_FINISH_OVERTAKING;
    m_cur_ego_planned_result_global.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_FINISH_OVERTAKING;
  } else {
    // defualt : ABORT_OVERTAKING
    m_cur_ego_planned_result_body.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_ABORT_OVERTAKING;
    m_cur_ego_planned_result_global.planning_stage_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_STAGE_ABORT_OVERTAKING;
  }

  m_ego_traj_body_pub->publish(m_cur_ego_planned_result_body);
  m_ego_traj_global_pub->publish(m_cur_ego_planned_result_global);
}

void DynamicPlannerNode::initTrajectory() {
  // Init output message (frame_id, reserve size)
  m_cur_ego_planned_result_body.header.frame_id =
      nif::common::frame_id::localization::BASE_LINK;
  m_prev_ego_planned_result_body.header.frame_id =
      nif::common::frame_id::localization::BASE_LINK;
  m_cur_ego_planned_result_global.header.frame_id =
      nif::common::frame_id::localization::ODOM;
  m_prev_ego_planned_result_global.header.frame_id =
      nif::common::frame_id::localization::ODOM;
  m_planned_traj_len = int(m_config_planning_horizon / m_config_planning_dt);
  nav_msgs::msg::Path init_path_body, init_path_global;
  for (int i = 0; i < m_planned_traj_len; i++) {
    geometry_msgs::msg::PoseStamped zero_pt_body, zero_pt_global;
    zero_pt_body.header.frame_id =
        nif::common::frame_id::localization::BASE_LINK;
    zero_pt_global.header.frame_id = nif::common::frame_id::localization::ODOM;

    init_path_body.poses.push_back(zero_pt_body);
    init_path_global.poses.push_back(zero_pt_global);

    m_cur_ego_planned_result_body.trajectory_timestamp_array.push_back(
        i * m_config_planning_dt);
    m_cur_ego_planned_result_global.trajectory_timestamp_array.push_back(
        i * m_config_planning_dt);
  }
  m_cur_ego_planned_result_body.trajectory_path = init_path_body;
  m_cur_ego_planned_result_global.trajectory_path = init_path_global;
}

bool DynamicPlannerNode::decisionMaking() {
  bool overtake_flg;

  //

  return overtake_flg;
}