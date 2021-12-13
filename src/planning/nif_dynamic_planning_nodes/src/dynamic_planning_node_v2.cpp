//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_common/constants.h"
#include "nif_dynamic_planning_nodes/dynamic_planning_node.h"
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

  // Init output trajectories
  initOutputTrajectory();

  // Init spliner & spline modeling for racing line
  auto racingline_xy = loadCSVfile(m_racingline_file_path);
  auto racingline_x_vec = get<0>(racingline_xy);
  auto racingline_y_vec = get<1>(racingline_xy);
  m_racingline_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      racingline_x_vec, racingline_y_vec, m_config_spline_interval);

  m_racingline_x_vec = get<0>(m_racingline_spline_data);
  m_racingline_y_vec = get<1>(m_racingline_spline_data);
  m_racingline_path_pc = genPointCloudFromVec(get<0>(m_racingline_spline_data),
                                              get<1>(m_racingline_spline_data));
  m_racineline_path_kdtree.setInputCloud(m_racingline_path_pc);

  m_racingline_path = xyyawVec2Path(get<0>(m_racingline_spline_data),
                                    get<1>(m_racingline_spline_data),
                                    get<2>(m_racingline_spline_data));

  // Init spliner & spline modeling for every overtaking path candidates
  for (int candidate_idx = 0; candidate_idx < m_num_overtaking_candidates;
       candidate_idx++) {
    auto wpt_xy =
        loadCSVfile(m_overtaking_candidates_file_path_vec[candidate_idx]);
    auto path_x_vec = get<0>(wpt_xy);
    auto path_y_vec = get<1>(wpt_xy);
    auto splined_result = m_frenet_generator_ptr->apply_cubic_spliner(
        path_x_vec, path_y_vec, m_config_spline_interval);

    m_overtaking_candidates_spline_data_vec.push_back(splined_result);

    auto pc =
        genPointCloudFromVec(get<0>(splined_result), get<1>(splined_result));
    m_overtaking_candidates_path_pc_vec.push_back(pc);

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(pc);
    m_overtaking_candidates_path_kdtree_vec.push_back(kdtree);

    auto candidate_path = xyyawVec2Path(
        get<0>(splined_result), get<1>(splined_result), get<2>(splined_result));

    m_overtaking_candidates_path_vec.push_back(candidate_path);
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

  m_racingline_file_path =
      path_candidates_params["racingline_path"].as<std::string>();
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

  m_config_spline_interval = planning_params["splining_interval"].as<double>();
  m_config_follow_enable_dist =
      planning_params["follow_enable_dist"].as<double>();
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
    m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
    m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
    m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
    m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
    m_timer_callback_first_run = false;
    publishTrajectory();
    return;
  }

  // update ego odometry
  m_ego_odom = this->getEgoOdometry();

  m_prev_decision = m_cur_decision;
  m_prev_overtaking_action = m_cur_overtaking_action;
  m_prev_ego_planned_result_body = m_cur_ego_planned_result_body;
  m_prev_ego_planned_result_global = m_cur_ego_planned_result_global;

  if (m_emergency_flg) {
    // TODO : do something, safe stop, emergency trajectory planning
    m_cur_decision = PLANNING_DECISION_TYPE::ESTOP;
    m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
    // temporary just sending empty traj
    nav_msgs::msg::Path empty_path;
    m_cur_ego_planned_result_body.trajectory_path = empty_path;
    m_cur_ego_planned_result_global.trajectory_path = empty_path;
    publishTrajectory();
    return;
  }

  if (m_cur_det.id == -1) {
    // NO OPPONENT CASE
    // TODO : empty detection result --> this should be implemented
    // in the perception side
    m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
    m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;

    // By pass maptrack to velocity planner
    // currently timestamp arry is uniformly sampled based on planning
    // horizon.In the velocity planner side, just drive as fast as possible when
    // decision type is straight.
    m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
    m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;

  } else {
    //////////////////////
    // DECION AND ACITION
    //////////////////////

    // Before comparing the progress

    auto progress_diff = calcProgressDiff(m_ego_odom.pose.pose,
                                          m_cur_det.detection_result_3d.center,
                                          m_racineline_path_kdtree);

    if (!m_overtake_allowed_flg) {
      /*
    STATUS DESCRIPTION :
      1. There is a opponent in front.
      2. Overtaking is not allowed by mission manager

    POSSIBLE DECISIONS(previous and current) AND ACTIONS
      CASE 1
        PREVIOUS DECISIONS  : STRAIGHT / FOLLOW
        DECISIONS           : STRAIGHT / FOLLOW / ESTOP
        ACTIONS             : DRIVING

        ** PAIRING (prev - cur-decision - action)
          1. (STRAIGHT - STRAIGHT - DRIVING)
          2. (STRAIGHT - FOLLOW - DRIVING)
          3. (FOLLOW - STRAIGHT - DRIVING)
          4. (FOLLOW - FOLLOW - DRIVING)
          5. (STRAIGHT - ESTOP - DRIVING)
          6. (FOLLOW - ESTOP - DRIVING)

      CASE 2
        PREVIOUS DECISIONS  : RIGHT / LEFT
        DECISIONS           : RIGHT / LEFT / ESTOP
        ACTIONS             : SIDE-BY-SIDE

        ** PAIRING (prev - cur-decision - action)
          1. (RIGHT - RIGHT - SIDE-BY-SIDE)
          2. (LEFT - LEFT - SIDE-BY-SIDE)
          3. (RIGHT - ESTOP - DRIVING)
          4. (LEFT - ESTOP - DRIVING)

      CASE 3
        PREVIOUS DECISIONS  : RIGHT / LEFT
        DECISIONS           : STRAIGHT / FOLLOW / ESTOP
        ACTIONS             : FINISH OT / ABORT OT

        ** PAIRING (prev - cur-decision - action)
          1. (RIGHT - STRAIGHT - FINISH OT)
          2. (RIGHT - FOLLOW - ABORT OT)
          3. (LEFT - STRAIGHT - FINISH OT)
          4. (LEFT - FOLLOW - ABORT OT)
          5. (RIGHT - ESTOP - DRIVING)
          6. (LEFT - ESTOP - DRIVING)
    */

      if (m_prev_decision == PLANNING_DECISION_TYPE::STRAIGHT ||
          m_prev_decision == PLANNING_DECISION_TYPE::FOLLOW) {
        ///////////////
        // CASE 1 CODE
        ///////////////

        // decision making (via progress comparison)
        if (progress_diff < 0.0) {
          // Ego vehicle is in front of the opponent.
          m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
          // by passing maptrack
          m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
          m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
        } else if (progress_diff < m_config_follow_enable_dist) {
          // close enougth to enable the ACC
          m_cur_decision = PLANNING_DECISION_TYPE::FOLLOW;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
          // TODO : by passing maptrack, ACC should be handled in the control
          // side
          m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
          m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
        } else {
          // far from the opponent
          m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
          m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
          m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
        }
      } else if (m_prev_decision == PLANNING_DECISION_TYPE::RIGHT ||
                 m_prev_decision == PLANNING_DECISION_TYPE::LEFT) {
        ///////////////////
        // CASE 2 & 3 CODE
        ///////////////////

        if (progress_diff < 0.0 &&
            m_config_merging_longitudinal_margin < abs(progress_diff)) {
          // ego vehicle is in front of the opponent and the distance is safe
          // enough to merge to the racing line
          m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::FINISH_OVERTAKING;
          // TODO : assign path, merging path is needed
          // Generate trajectory segment from current odom to racing line.
          auto progressNcte = calcProgressNCTE(m_ego_odom.pose.pose,
                                               m_racineline_path_kdtree,
                                               m_racingline_path_pc);

          std::tuple<std::shared_ptr<FrenetPath>,
                     std::vector<std::shared_ptr<FrenetPath>>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths(
                      get<1>(progressNcte),            // current_position_d
                      get<0>(progressNcte),            // current_position_s
                      0.0,                             // current_velocity_d
                      m_ego_odom.twist.twist.linear.x, // current_velocity_s
                      0.0,                             // current_acceleration_d
                      get<4>(m_racingline_spline_data), // cubic_spliner_2D
                      m_config_planning_horizon,
                      m_config_planning_horizon + 0.01,
                      m_config_planning_dt,
                      0.0,    // left margin
                      0.0001, // right margin
                      0.1);   // sampling width dt

          std::shared_ptr<FrenetPath>& predicted_frenet_path =
              std::get<0>(frenet_path_generation_result);

          m_cur_ego_planned_result_body.trajectory_path.poses.clear();
          m_cur_ego_planned_result_global.trajectory_path.poses.clear();

          if (!predicted_frenet_path->points_x().empty()) {
            for (int i = 0; i < predicted_frenet_path->points_x().size(); i++) {
              geometry_msgs::msg::PoseStamped ps;
              std_msgs::msg::Header header;
              header.frame_id = common::frame_id::localization::ODOM;
              ps.pose.position.x = predicted_frenet_path->points_x()[i];
              ps.pose.position.y = predicted_frenet_path->points_y()[i];

              m_cur_ego_planned_result_global.trajectory_path.poses.push_back(
                  ps);
              m_cur_ego_planned_result_body.trajectory_timestamp_array
                  .push_back(predicted_frenet_path->time()[i]);
              m_cur_ego_planned_result_global.trajectory_timestamp_array
                  .push_back(predicted_frenet_path->time()[i]);
            }

            m_cur_ego_planned_result_body.trajectory_path =
                nif::common::utils::coordination::getPathGlobaltoBody(
                    m_ego_odom,
                    m_cur_ego_planned_result_global.trajectory_path);

            // TODO : Stitch with the racing line
          }
        } else {
          bool is_side_by_side_available = false;
          // TODO : Need to decide SIDE-BY-SIDE or ABORT OT

          if (is_side_by_side_available) {
            m_cur_decision = m_prev_decision;
            m_cur_overtaking_action = PLANNING_ACTION_TYPE::SIDE_BY_SIDE;
            // TODO : assign path
          } else {
            m_cur_decision = PLANNING_DECISION_TYPE::FOLLOW;
            m_cur_overtaking_action = PLANNING_ACTION_TYPE::ABORT_OVERTAKING;
            // TODO : by passing maptrack, ACC should be handled in the
            // control side
            m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
            m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
          }
        }

      } else {
        /////////
        // ESTOP
        /////////

        m_cur_decision = PLANNING_DECISION_TYPE::ESTOP;
        m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
        // TODO : temporary just sending empty traj
        nav_msgs::msg::Path empty_path;
        m_cur_ego_planned_result_body.trajectory_path = empty_path;
        m_cur_ego_planned_result_global.trajectory_path = empty_path;
        publishTrajectory();
      }
    } else {
      /*
    STATUS DESCRIPTION :
      1. There is a opponent in front.
      2. Overtaking is allowed by mission manager

    POSSIBLE DECISIONS(previous and current) AND ACTIONS
      CASE 1
        PREVIOUS DECISIONS  : STRAIGHT
        DECISIONS           : STRAIGHT / FOLLOW / ESTOP
        ACTIONS             : DRIVING

      ** PAIRING (prev - cur-decision - action)
        1. (STRAIGHT - STRAIGHT - DRIVING)
        2. (STRAIGHT - FOLLOW - DRIVING)
        3. (STRAIGHT - ESTOP - DRIVING)

      CASE 2
        PREVIOUS DECISIONS  : FOLLOW
        DECISIONS           : FOLLOW / RIGHT / LEFT / ESTOP
        ACTIONS             : START OT / DRIVING

      ** PAIRING (prev - cur-decision - action)
        1. (FOLLOW - FOLLOW - DRIVING)
        2. (FOLLOW - RIGHT/LEFT - START OT)
        3. (FOLLOW - ESTOP - DRIVING)

      CASE 3
        PREVIOUS DECISIONS  : RIGHT / LEFT
        DECISIONS           : RIGHT / LEFT / ESTOP
        ACTIONS             : SIDE-BY-SIDE / DRIVING

      ** PAIRING (prev - cur-decision - action)
        1. (RIGHT - RIGHT - SIDE-BY-SIDE)
        2. (LEFT - LEFT - SIDE-BY-SIDE)
        3. (RIGHT/LEFT - ESTOP - DRIVING)

      CASE 4
        PREVIOUS DECISIONS  : RIGHT / LEFT
        DECISIONS           : STRAIGHT / FOLLOW / ESTOP
        ACTIONS             : FINISH OT / ABORT OT / DRIVING

      ** PAIRING (prev - cur-decision - action)
        1. (RIGHT/LEFT - STRAIGHT - FINISH OT)
        2. (RIGHT/LEFT - FOLLOW - ABORT OT)
        3. (RIGHT/LEFT - ESTOP - DRIVING)
    */
      if (m_prev_decision == PLANNING_DECISION_TYPE::STRAIGHT) {
        ///////////////
        // CASE 1 CODE
        ///////////////
        // decision making (via progress comparison)
        if (progress_diff < 0.0) {
          m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
          m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
          m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
        } else if (progress_diff < m_config_follow_enable_dist) {
          // close enougth to enable the ACC
          m_cur_decision = PLANNING_DECISION_TYPE::FOLLOW;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
          // TODO : by passing maptrack, ACC should be handled in the control
          // side
          m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
          m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
        } else {
          // far from the opponent
          m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
          m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
          m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
        }
      } else if (m_prev_decision == PLANNING_DECISION_TYPE::FOLLOW) {
        ///////////////
        // CASE 2 CODE
        ///////////////

        bool is_left_overtaking_available = false;
        bool is_right_overtaking_available = false;
        // TODO : check whether overtaking is available or not, both sides
        // Check left side first

        if (is_left_overtaking_available) {
          m_cur_decision = PLANNING_DECISION_TYPE::LEFT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::START_OVERTAKING;
          // TODO : assign path
        } else if (is_right_overtaking_available) {
          m_cur_decision = PLANNING_DECISION_TYPE::RIGHT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::START_OVERTAKING;
          // TODO : assign path
        } else {
          // overtake unavailable
          m_cur_decision = PLANNING_DECISION_TYPE::FOLLOW;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
          // TODO : by passing maptrack, ACC should be handled in the control
          // side
          m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
          m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
        }
      } else if (m_prev_decision == PLANNING_DECISION_TYPE::RIGHT ||
                 m_prev_decision == PLANNING_DECISION_TYPE::LEFT) {
        //////////////////
        // CASE 3 & 4 CODE
        //////////////////

        if (progress_diff < 0.0 &&
            m_config_merging_longitudinal_margin < abs(progress_diff)) {
          // ego vehicle is in front of the opponent and the distance is safe
          // enough to merge to the racing line
          m_cur_decision = PLANNING_DECISION_TYPE::STRAIGHT;
          m_cur_overtaking_action = PLANNING_ACTION_TYPE::FINISH_OVERTAKING;
          // TODO : assign path, merging path is needed
        } else {
          bool is_side_by_side_available = false;
          // TODO : Need to decide SIDE-BY-SIDE or ABORT OT

          if (is_side_by_side_available) {
            m_cur_decision = m_prev_decision;
            m_cur_overtaking_action = PLANNING_ACTION_TYPE::SIDE_BY_SIDE;
            // TODO : assign path
          } else {
            m_cur_decision = PLANNING_DECISION_TYPE::FOLLOW;
            m_cur_overtaking_action = PLANNING_ACTION_TYPE::ABORT_OVERTAKING;
            // TODO : by passing maptrack, ACC should be handled in the control
            // side
            m_cur_ego_planned_result_body.trajectory_path = m_maptrack_body;
            m_cur_ego_planned_result_global.trajectory_path = m_maptrack_global;
          }
        }

      } else {
        /////////
        // ESTOP
        /////////

        m_cur_decision = PLANNING_DECISION_TYPE::ESTOP;
        m_cur_overtaking_action = PLANNING_ACTION_TYPE::DRIVING;
        // TODO : temporary just sending empty traj
        initOutputTrajectory(); // all path points are assigned with zero
        publishTrajectory();
      }
    }
  }
  publishTrajectory();
}

void DynamicPlannerNode::publishTrajectory() {
  m_cur_ego_planned_result_body.header.stamp = this->now();
  m_cur_ego_planned_result_global.header.stamp = this->now();

  m_cur_ego_planned_result_body.trajectory_type =
      nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;
  m_cur_ego_planned_result_global.trajectory_type =
      nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;

  if (m_cur_decision == PLANNING_DECISION_TYPE::STRAIGHT) {
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_STRAIGHT;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_STRAIGHT;
  } else if (m_cur_decision == PLANNING_DECISION_TYPE::FOLLOW) {
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_FOLLOW;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_FOLLOW;
  } else if (m_cur_decision == PLANNING_DECISION_TYPE::RIGHT) {
    m_cur_ego_planned_result_body.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_RIGHT;
    m_cur_ego_planned_result_global.planning_decision_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_RIGHT;
  } else if (m_cur_decision == PLANNING_DECISION_TYPE::LEFT) {
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

  if (m_cur_overtaking_action == PLANNING_ACTION_TYPE::DRIVING) {
    m_cur_ego_planned_result_body.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_DRIVING;
    m_cur_ego_planned_result_global.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_DRIVING;
  } else if (m_cur_overtaking_action ==
             PLANNING_ACTION_TYPE::START_OVERTAKING) {
    m_cur_ego_planned_result_body.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
    m_cur_ego_planned_result_global.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
  } else if (m_cur_overtaking_action == PLANNING_ACTION_TYPE::SIDE_BY_SIDE) {
    m_cur_ego_planned_result_body.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_SIDE_BY_SIDE;
    m_cur_ego_planned_result_global.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_SIDE_BY_SIDE;
  } else if (m_cur_overtaking_action ==
             PLANNING_ACTION_TYPE::FINISH_OVERTAKING) {
    m_cur_ego_planned_result_body.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
    m_cur_ego_planned_result_global.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
  } else {
    // defualt : ABORT_OVERTAKING
    m_cur_ego_planned_result_body.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_ABORT_OVERTAKING;
    m_cur_ego_planned_result_global.planning_action_type =
        nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_ABORT_OVERTAKING;
  }

  m_ego_traj_body_pub->publish(m_cur_ego_planned_result_body);
  m_ego_traj_global_pub->publish(m_cur_ego_planned_result_global);
}

void DynamicPlannerNode::initOutputTrajectory() {
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

double
DynamicPlannerNode::getProgress(const geometry_msgs::msg::Pose& pt_global_,
                                pcl::KdTreeFLANN<pcl::PointXY>& target_tree_) {
  double progress;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY* searchPoint = new pcl::PointXY();
  searchPoint->x = pt_global_.position.x;
  searchPoint->y = pt_global_.position.y;
  int index = 0;

  if (target_tree_.nearestKSearch(
          *searchPoint, 1, pointId_vector, pointRadius_vector) > 0) {
    index = pointId_vector[0];
  } else {
    // TODO : what happens?
  }

  return index * m_config_spline_interval;
}

double DynamicPlannerNode::calcCTE(const geometry_msgs::msg::Pose& pt_global_,
                                   pcl::KdTreeFLANN<pcl::PointXY>& target_tree_,
                                   pcl::PointCloud<pcl::PointXY>::Ptr& pc_) {
  double cte = 0;
  double progress;
  int sign;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY* searchPoint = new pcl::PointXY();
  searchPoint->x = pt_global_.position.x;
  searchPoint->y = pt_global_.position.y;

  if (target_tree_.nearestKSearch(
          *searchPoint, 1, pointId_vector, pointRadius_vector) > 0) {
    cte = pointRadius_vector[0];
    progress = pointId_vector[0] * m_config_spline_interval;

    int next_idx = pointId_vector[0] + 1;
    double next_x, next_y;
    if (pc_->points.size() < next_idx) {
      next_idx = next_idx - pc_->points.size();
    }
    next_x = pc_->points[next_idx].x;
    next_y = pc_->points[next_idx].y;

    auto cross_product = next_x * pc_->points[pointId_vector[0]].y -
        next_y * pc_->points[pointId_vector[0]].x;
    if (cross_product < 0) {
      sign = -1;
    } else {
      sign = 1;
    }

    cte = cte * sign;

  } else {
    // TODO : what happens?
  }

  return cte;
}

pcl::PointCloud<pcl::PointXY>::Ptr
DynamicPlannerNode::genPointCloudFromVec(vector<double>& x_,
                                         vector<double>& y_) {
  pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);

  // Generate pointcloud data
  cloud->width = x_.size();
  cloud->height = 1;
  cloud->points.resize(cloud->width * cloud->height);

  for (std::size_t i = 0; i < cloud->size(); ++i) {
    (*cloud)[i].x = x_[i];
    (*cloud)[i].y = y_[i];
  }
  return cloud;
}

double DynamicPlannerNode::calcProgressDiff(
    const geometry_msgs::msg::Pose& ego_pt_global_,
    const geometry_msgs::msg::Pose& target_pt_global_,
    pcl::KdTreeFLANN<pcl::PointXY>& target_tree_) {
  auto ego_progress = getProgress(ego_pt_global_, target_tree_);
  auto target_progress = getProgress(target_pt_global_, target_tree_);

  // TODO : progress wrapping is needed!!!!!!!!!!!!!!!!!!!
  return ego_progress - target_progress;
}

nav_msgs::msg::Path
DynamicPlannerNode::xyyawVec2Path(std::vector<double>& x_,
                                  std::vector<double>& y_,
                                  std::vector<double>& yaw_rad_) {
  nav_msgs::msg::Path output;

  for (int i = 0; i < x_.size(); i++) {
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.position.x = x_[i];
    pt.pose.position.y = y_[i];
    pt.pose.position.z = 0.0;
    pt.pose.orientation =
        nif::common::utils::coordination::ToQuaternion(yaw_rad_[i], 0.0, 0.0);
    output.poses.push_back(pt);
  }

  return output;
}

tuple<double, double> DynamicPlannerNode::calcProgressNCTE(
    const geometry_msgs::msg::Pose& pt_global_,
    pcl::KdTreeFLANN<pcl::PointXY>& target_tree_,
    pcl::PointCloud<pcl::PointXY>::Ptr& pc_) {
  double cte = 0;
  double progress;
  int sign;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY* searchPoint = new pcl::PointXY();
  searchPoint->x = pt_global_.position.x;
  searchPoint->y = pt_global_.position.y;

  if (target_tree_.nearestKSearch(
          *searchPoint, 1, pointId_vector, pointRadius_vector) > 0) {
    cte = pointRadius_vector[0];
    progress = pointId_vector[0] * m_config_spline_interval;

    int next_idx = pointId_vector[0] + 1;
    double next_x, next_y;
    if (pc_->points.size() < next_idx) {
      next_idx = next_idx - pc_->points.size();
    }
    next_x = pc_->points[next_idx].x;
    next_y = pc_->points[next_idx].y;

    auto cross_product = next_x * pc_->points[pointId_vector[0]].y -
        next_y * pc_->points[pointId_vector[0]].x;
    if (cross_product < 0) {
      sign = -1;
    } else {
      sign = 1;
    }

    cte = cte * sign;

  } else {
    // TODO : what happens?
  }

  return std::make_tuple(progress, cte);
}