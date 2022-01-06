//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_dynamic_planning_nodes/dynamic_planning_node_v2.h"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdlib.h>

using namespace nif::planning;
using namespace std;

DynamicPlannerNode::DynamicPlannerNode(const std::string &node_name_)
    : IBaseNode(node_name_, common::NodeType::PLANNING) {
  this->setNodeStatus(nif::common::NODE_NOT_INITIALIZED);

  m_config_load_success = false;

  std::string package_share_directory;
  try {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_dynamic_planning_nodes");
  } catch (std::exception e) {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }
  package_share_directory = package_share_directory.append("/");

  this->declare_parameter("planning_config_file_path", "");
  this->declare_parameter("velocity_profile_config_file_path", "");
  this->declare_parameter("maps_path_root", "");
  this->declare_parameter("vis_flg", true);

  this->get_parameter("maps_path_root", this->m_map_root_path);
  this->m_map_root_path.append("/");
  m_planning_config_file_path =
      this->get_parameter("planning_config_file_path").as_string();
  m_velocity_profile_config_file_path =
      this->get_parameter("velocity_profile_config_file_path").as_string();
  m_vis_flg = this->get_parameter("vis_flg").as_bool();

  if (m_planning_config_file_path.empty()) {
    throw std::runtime_error(
        "Parameter m_planning_config_file_path not declared, or empty.");
  }
  if (m_velocity_profile_config_file_path.empty()) {
    throw std::runtime_error("Parameter m_velocity_profile_config_file_path "
                             "not declared, or empty.");
  }

  // Load param
  loadConfig(m_planning_config_file_path);
  m_config_load_success = true;
  m_det_callback_first_run = true;
  m_oppo_pred_callback_first_run = true;
  m_timer_callback_first_run = true;

  // Init output trajectories
  // initOutputTrajectory();

  // //////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR RACING LINE
  // //////////////////////////////////////////////
  m_racingline_file_path = this->m_map_root_path + m_racingline_file_path;
  auto racingline_xy = loadCSVfile(m_racingline_file_path);
  auto racingline_x_vec = get<0>(racingline_xy);
  auto racingline_y_vec = get<1>(racingline_xy);
  m_racingline_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      racingline_x_vec, racingline_y_vec, m_config_spline_interval);
  m_racingline_x_vec = get<0>(m_racingline_spline_data);
  m_racingline_y_vec = get<1>(m_racingline_spline_data);
  m_racingline_path = xyyawVec2Path(get<0>(m_racingline_spline_data),
                                    get<1>(m_racingline_spline_data),
                                    get<2>(m_racingline_spline_data));
  // m_racingline_path_pc =
  // genPointCloudFromVec(get<0>(m_racingline_spline_data),
  //                                             get<1>(m_racingline_spline_data));
  // m_racineline_path_kdtree.setInputCloud(m_racingline_path_pc);
  m_racingline_dtraj.header = m_racingline_path.header;
  m_racingline_dtraj.trajectory_path = m_racingline_path;
  m_racingline_full_progress =
      get<4>(m_racingline_spline_data)->points_s().back();

  // minimal checking
  if (racingline_x_vec.size() != racingline_y_vec.size() ||
      racingline_x_vec.size() == 0) {
    throw std::runtime_error(
        "Racing wpt file has a problem. Stop node initialization.");
  }

  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Raceline is loaded...");
  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Loading defender line...");

  // ////////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR DEFENDER LINE
  // ////////////////////////////////////////////////
  m_defenderline_file_path = this->m_map_root_path + m_defenderline_file_path;
  auto defenderline_xy = loadCSVfile(m_defenderline_file_path);
  auto defenderline_x_vec = get<0>(defenderline_xy);
  auto defenderline_y_vec = get<1>(defenderline_xy);
  m_defenderline_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      defenderline_x_vec, defenderline_y_vec, m_config_spline_interval);
  m_defenderline_x_vec = get<0>(m_defenderline_spline_data);
  m_defenderline_y_vec = get<1>(m_defenderline_spline_data);
  m_defenderline_path = xyyawVec2Path(get<0>(m_defenderline_spline_data),
                                      get<1>(m_defenderline_spline_data),
                                      get<2>(m_defenderline_spline_data));
  m_defenderline_dtraj.header = m_defenderline_path.header;
  m_defenderline_dtraj.trajectory_path = m_defenderline_path;
  m_defenderline_full_progress =
      get<4>(m_defenderline_spline_data)->points_s().back();

  // minimal checking
  if (defenderline_x_vec.size() != defenderline_y_vec.size() ||
      defenderline_x_vec.size() == 0) {
    throw std::runtime_error(
        "Defender wpt file has a problem. Stop node initialization.");
  }

  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Defender line is loaded...");
  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Loading stay behind path...");

  // ////////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR STAYBEHIND LINE
  // ////////////////////////////////////////////////
  m_staybehind_file_path = this->m_map_root_path + m_staybehind_file_path;
  auto staybehind_xy = loadCSVfile(m_staybehind_file_path);
  auto staybehind_x_vec = get<0>(staybehind_xy);
  auto staybehind_y_vec = get<1>(staybehind_xy);
  m_staybehind_spline_data = m_frenet_generator_ptr->apply_cubic_spliner(
      staybehind_x_vec, staybehind_y_vec, m_config_spline_interval);
  m_staybehind_x_vec = get<0>(m_staybehind_spline_data);
  m_staybehind_y_vec = get<1>(m_staybehind_spline_data);
  m_staybehind_path = xyyawVec2Path(get<0>(m_staybehind_spline_data),
                                    get<1>(m_staybehind_spline_data),
                                    get<2>(m_staybehind_spline_data));
  m_staybehind_dtraj.header = m_staybehind_path.header;
  m_staybehind_dtraj.trajectory_path = m_staybehind_path;
  m_staybehind_full_progress =
      get<4>(m_staybehind_spline_data)->points_s().back();

  // minimal checking
  if (staybehind_x_vec.size() != staybehind_y_vec.size() ||
      staybehind_x_vec.size() == 0) {
    throw std::runtime_error(
        "STAY BEHIND wpt file has a problem. Stop node initialization.");
  }

  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Stay behind line is loaded...");
  RCLCPP_INFO(this->get_logger(),
              "[DYNAMICPLANNER] Loading overtaking path candidates...");

  // //////////////////////////////////////////////////////////////////
  // INIT SPLINER & SPLINE MODELING FOR EVERY OVERTAKING PATH CANDIDATES
  // //////////////////////////////////////////////////////////////////
  for (int candidate_idx = 0; candidate_idx < m_num_overtaking_candidates;
       candidate_idx++) {
    m_overtaking_candidates_file_path_vec[candidate_idx] =
        this->m_map_root_path +
        m_overtaking_candidates_file_path_vec[candidate_idx];
    auto wpt_xy =
        loadCSVfile(m_overtaking_candidates_file_path_vec[candidate_idx]);
    auto path_x_vec = get<0>(wpt_xy);
    auto path_y_vec = get<1>(wpt_xy);
    auto splined_result = m_frenet_generator_ptr->apply_cubic_spliner(
        path_x_vec, path_y_vec, m_config_spline_interval);
    m_overtaking_candidates_spline_data_vec.push_back(splined_result);
    m_overtaking_candidates_spline_model_vec.push_back(get<4>(splined_result));
    auto candidate_path = xyyawVec2Path(
        get<0>(splined_result), get<1>(splined_result), get<2>(splined_result));
    auto full_progress = get<4>(splined_result)->points_s().back();
    m_overtaking_candidates_full_progress_vec.push_back(full_progress);
    m_overtaking_candidates_path_vec.push_back(candidate_path);
    nif_msgs::msg::DynamicTrajectory tmp;
    tmp.header = candidate_path.header;
    tmp.trajectory_path = candidate_path;
    m_overkaing_candidates_dtraj_vec.push_back(tmp);

    // minimal checking
    if (path_x_vec.size() != path_y_vec.size() || path_x_vec.size() == 0) {
      throw std::runtime_error("One of the overtaking path file has a problem. "
                               "Stop node initialization.");
    }
  }

  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] All paths are loaded...");

  // INITIALIZE SUBSCRIBERS & PUBLISHER
  m_det_sub =
      this->create_subscription<nif::common::msgs::PerceptionResultList>(
          "tracking_output_topic_name", common::constants::QOS_PLANNING,
          std::bind(&DynamicPlannerNode::detectionResultCallback, this,
                    std::placeholders::_1));
  m_oppo_pred_sub = this->create_subscription<nif_msgs::msg::DynamicTrajectory>(
      "prediction_output_topic_name", common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::predictionResultCallback, this,
                std::placeholders::_1));
  m_maptrack_global_sub = this->create_subscription<nav_msgs::msg::Path>(
      "wptmanager_output_topic_name_global", common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::mapTrackGlobalCallback, this,
                std::placeholders::_1));
  m_maptrack_body_sub = this->create_subscription<nav_msgs::msg::Path>(
      "wptmanager_output_topic_name_body", common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNode::mapTrackBodyCallback, this,
                std::placeholders::_1));

  m_ego_traj_body_pub =
      this->create_publisher<nif_msgs::msg::DynamicTrajectory>(
          "out_trajectory_body", common::constants::QOS_PLANNING);
  m_ego_traj_global_pub =
      this->create_publisher<nif_msgs::msg::DynamicTrajectory>(
          "out_trajectory_global", common::constants::QOS_PLANNING);
  m_ego_traj_body_vis_pub = this->create_publisher<nav_msgs::msg::Path>(
      "out_trajectory_vis_body", common::constants::QOS_PLANNING);
  m_ego_traj_global_vis_pub = this->create_publisher<nav_msgs::msg::Path>(
      "out_trajectory_vis_global", common::constants::QOS_PLANNING);
  m_debug_vis_pub = this->create_publisher<nav_msgs::msg::Path>(
      "planning/debug", common::constants::QOS_PLANNING);

  m_ego_traj_global_vis_debug_pub1 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug1", common::constants::QOS_PLANNING);
  m_ego_traj_global_debug_pub1 =
      this->create_publisher<nif_msgs::msg::DynamicTrajectory>(
          "planning/traj/debug1", common::constants::QOS_PLANNING);
  m_ego_traj_global_vis_debug_pub2 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug2", common::constants::QOS_PLANNING);
  m_ego_traj_global_vis_debug_pub3 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug3", common::constants::QOS_PLANNING);

  // m_planner_timer = this->create_wall_timer(
  //     20ms, std::bind(&DynamicPlannerNode::timer_callback, this)); // 50 hz
  m_planner_timer = this->create_wall_timer(
      20ms, std::bind(&DynamicPlannerNode::timer_callback_rule, this)); // 50 hz
  // m_planner_timer = this->create_wall_timer(
  //     20ms,
  //     std::bind(&DynamicPlannerNode::timer_callback_debug, this)); // 50
  //     hz

  RCLCPP_INFO(this->get_logger(), "[DYNAMICPLANNER] Initialization done.");

  this->setNodeStatus(nif::common::NODE_INITIALIZED);
}

void DynamicPlannerNode::loadConfig(const std::string &planning_config_file_) {
  RCLCPP_INFO(get_logger(), "Loading planning params: %s",
              planning_config_file_.c_str());

  YAML::Node config = YAML::LoadFile(planning_config_file_);

  if (!config["path_candidates_param"]) {
    throw std::runtime_error(
        "path_candidates_param field not defined in config file.");
  }
  if (!config["planning_params"]) {
    throw std::runtime_error(
        "planning_params field not defined in config file.");
  }
  if (!config["collision_checking_params"]) {
    throw std::runtime_error(
        "collision_checking_params field not defined in config file.");
  }
  if (!config["waypoint_manager_param"]) {
    throw std::runtime_error(
        "waypoint_manager_param field not defined in config file.");
  }

  // path_candidates_param
  YAML::Node path_candidates_params = config["path_candidates_param"];

  m_racingline_file_path =
      path_candidates_params["racingline_path"].as<std::string>();
  m_staybehind_file_path =
      path_candidates_params["staybehindline_path"].as<std::string>();
  m_defenderline_file_path =
      path_candidates_params["defenderline_path"].as<std::string>();
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
        "path_candidates_param is not properly set. Check config file.");
  }

  m_num_overtaking_candidates = m_overtaking_candidates_file_path_vec.size();

  // planning params
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
  m_config_merge_allow_dist =
      planning_params["merging_allow_dist_to_racingline"].as<double>();

  // collision_checking_params
  YAML::Node collision_checking_params = config["collision_checking_params"];

  m_config_overlap_checking_dist_bound =
      collision_checking_params["overlap_checking_dist_bound"].as<double>();
  m_config_overlap_checking_time_bound =
      collision_checking_params["overlap_checking_time_bound"].as<double>();

  // collision_checking_params
  YAML::Node waypoint_manager_params = config["waypoint_manager_param"];

  m_maptrack_size = waypoint_manager_params["maptrack_size"].as<int>();

  // minimal checking
  if (m_maptrack_size < 0) {
    throw std::runtime_error(
        "m_maptrack_size can not be less than zero. Check config file.");
  }

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

  // if (m_config_overlap_checking_dist_bound <=
  //   nif::common::vehicle_param::VEH_WHEEL_BASE)
  // {
  //   throw std::runtime_error(
  //           "m_config_overlap_checking_dist_bound can not be "
  //           "less than Vehicle wheel base(4.921m). Check config file.");
  // }

  if (m_config_merge_allow_dist <= 0.0) {
    throw std::runtime_error("m_config_merge_allow_dist can not be "
                             "less than zero. Check config file.");
  }

  if (m_config_overlap_checking_dist_bound <= 0.0) {
    throw std::runtime_error("m_config_overlap_checking_dist_bound can not be "
                             "less than zero. Check config file.");
  }

  if (m_config_overlap_checking_time_bound <= 0.0) {
    throw std::runtime_error("m_config_overlap_checking_time_bound can not be "
                             "less than zero. Check config file.");
  }
}

tuple<vector<double>, vector<double>>
DynamicPlannerNode::loadCSVfile(const std::string &wpt_file_path_) {
  ifstream inputFile(wpt_file_path_);
  vector<double> vec_x, vec_y;

  while (inputFile) {
    string s;
    if (!getline(inputFile, s)) {
      break;
    }
    if (s[0] != '#') {
      istringstream ss(s);
      int cnt = 0;
      bool nan_flg = false;
      while (ss) {
        string line;
        if (!getline(ss, line, ',')) {
          break;
        }
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
    const nif::common::msgs::PerceptionResultList::SharedPtr msg) {
  // TODO: Detection result health check

  // TRACKING RESULT CALLBACK (GLOBAL COORDINATE)
  if (m_det_callback_first_run) {
    m_det_callback_first_run = false;
  } else {
    m_prev_det_global = m_cur_det_global;
  }

  if (!msg->perception_list.empty()) {
    m_cur_det_global = msg->perception_list[0];
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
  // TODO: Prediction result health check

  if (m_oppo_pred_callback_first_run) {
    m_oppo_pred_callback_first_run = false;
  } else {
    m_prev_oppo_pred_result = m_cur_oppo_pred_result;
  }
  m_prev_oppo_pred_last_update = this->now(); // TODO msg->header.stamp;
  m_cur_oppo_pred_result = *msg;
}

void DynamicPlannerNode::publishEmptyTrajectory() {
  nif_msgs::msg::DynamicTrajectory empty_traj;
  nav_msgs::msg::Path empty_path;

  empty_traj.header.stamp = this->now();
  empty_path.header.stamp = this->now();

  empty_traj.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  m_ego_traj_body_pub->publish(empty_traj);

  empty_traj.header.frame_id = nif::common::frame_id::localization::ODOM;
  m_ego_traj_global_pub->publish(empty_traj);

  empty_path.header.frame_id = nif::common::frame_id::localization::BASE_LINK;
  m_ego_traj_body_vis_pub->publish(empty_path);

  empty_path.header.frame_id = nif::common::frame_id::localization::ODOM;
  m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
}

void DynamicPlannerNode::publishPlannedTrajectory(bool vis_flg_) {
  m_cur_ego_planned_result_body.trajectory_path.poses.clear();
  m_cur_ego_planned_result_global.trajectory_path.poses.clear();

  m_cur_ego_planned_result_body.header.stamp = this->now();
  m_cur_ego_planned_result_body.header.frame_id =
      nif::common::frame_id::localization::BASE_LINK;
  m_cur_ego_planned_result_global.header.stamp = this->now();
  m_cur_ego_planned_result_global.header.frame_id =
      nif::common::frame_id::localization::ODOM;

  m_cur_ego_planned_result_body.trajectory_type =
      nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;
  m_cur_ego_planned_result_global.trajectory_type =
      nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;

  // Current idx
  m_ego_cur_idx_in_planned_traj = calcCurIdxFromDynamicTraj(m_cur_planned_traj);

  int tmp_wpt_len = 100;

  for (int wpt_idx = 0; wpt_idx < tmp_wpt_len; wpt_idx++) {
    geometry_msgs::msg::PoseStamped ps_body;
    geometry_msgs::msg::PoseStamped ps_global;

    int target_idx_in_full_path = m_ego_cur_idx_in_planned_traj + wpt_idx;
    // index wrapping
    if (target_idx_in_full_path >=
        m_cur_planned_traj.trajectory_path.poses.size()) {
      target_idx_in_full_path -=
          m_cur_planned_traj.trajectory_path.poses.size();
    }

    ps_global =
        m_cur_planned_traj.trajectory_path.poses[target_idx_in_full_path];
    ps_global.header.frame_id = nif::common::frame_id::localization::ODOM;

    ps_body =
        common::utils::coordination::getPtGlobaltoBody(m_ego_odom, ps_global);
    ps_body.header.frame_id = nif::common::frame_id::localization::BASE_LINK;

    m_cur_ego_planned_result_body.trajectory_path.poses.push_back(ps_body);
    m_cur_ego_planned_result_global.trajectory_path.poses.push_back(ps_global);

    // TODO : no idea to assign the velocity, time, progress.... FIX THIS NEAR
    // SOON!!!
  }
  m_ego_traj_body_pub->publish(m_cur_ego_planned_result_body);
  m_ego_traj_global_pub->publish(m_cur_ego_planned_result_global);

  if (vis_flg_) {
    m_ego_planned_vis_path_body = m_cur_ego_planned_result_body.trajectory_path;
    m_ego_planned_vis_path_global =
        m_cur_ego_planned_result_global.trajectory_path;

    m_ego_planned_vis_path_body.header.frame_id =
        nif::common::frame_id::localization::BASE_LINK;
    m_ego_planned_vis_path_global.header.frame_id =
        nif::common::frame_id::localization::ODOM;

    m_ego_planned_vis_path_body.header.stamp = this->now();
    m_ego_planned_vis_path_global.header.stamp = this->now();

    m_ego_traj_body_vis_pub->publish(m_ego_planned_vis_path_body);
    m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
  }
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

    init_path_body.poses.clear();
    init_path_global.poses.clear();

    init_path_body.poses.push_back(zero_pt_body);
    init_path_global.poses.push_back(zero_pt_global);

    m_cur_ego_planned_result_body.trajectory_timestamp_array.clear();
    m_cur_ego_planned_result_global.trajectory_timestamp_array.clear();

    m_cur_ego_planned_result_body.trajectory_timestamp_array.push_back(
        i * m_config_planning_dt);
    m_cur_ego_planned_result_global.trajectory_timestamp_array.push_back(
        i * m_config_planning_dt);
  }
  m_cur_ego_planned_result_body.trajectory_path = init_path_body;
  m_cur_ego_planned_result_global.trajectory_path = init_path_global;
}

double
DynamicPlannerNode::getProgress(const geometry_msgs::msg::Pose &pt_global_,
                                pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
  double progress;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY *searchPoint = new pcl::PointXY();
  searchPoint->x = pt_global_.position.x;
  searchPoint->y = pt_global_.position.y;
  int index = 0;

  if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
                                  pointRadius_vector) > 0) {
    index = pointId_vector[0];
  } else {
    // TODO : what happens?
  }

  return index * m_config_spline_interval;
}

double
DynamicPlannerNode::getProgress(const double &pt_x_, const double &pt_y_,
                                pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
  double progress;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY *searchPoint = new pcl::PointXY();
  searchPoint->x = pt_x_;
  searchPoint->y = pt_y_;
  int index = 0;

  if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
                                  pointRadius_vector) > 0) {
    index = pointId_vector[0];
  } else {
    // TODO : what happens?
  }

  return index * m_config_spline_interval;
}

double
DynamicPlannerNode::getCurIdx(const double &pt_x_, const double &pt_y_,
                              pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
  double progress;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY *searchPoint = new pcl::PointXY();
  searchPoint->x = pt_x_;
  searchPoint->y = pt_y_;
  int index = 0;

  if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
                                  pointRadius_vector) > 0) {
    index = pointId_vector[0];
  } else {
    // TODO : what happens?
  }

  return index;
}

double DynamicPlannerNode::getCurIdx(const double &pt_x_, const double &pt_y_,
                                     const nav_msgs::msg::Path &target_path_) {
  int closest_idx = 0;
  double min_dist = nif::common::constants::numeric::INF;
  for (int i = 0; i < target_path_.poses.size(); i++) {
    double dist = sqrt(pow(pt_x_ - target_path_.poses[i].pose.position.x, 2) +
                       pow(pt_y_ - target_path_.poses[i].pose.position.y, 2));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  return closest_idx;
}

double DynamicPlannerNode::calcCTE(const geometry_msgs::msg::Pose &pt_global_,
                                   pcl::KdTreeFLANN<pcl::PointXY> &target_tree_,
                                   pcl::PointCloud<pcl::PointXY>::Ptr &pc_) {
  double cte = 0;
  double progress;
  int sign;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY *searchPoint = new pcl::PointXY();
  searchPoint->x = pt_global_.position.x;
  searchPoint->y = pt_global_.position.y;

  if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
                                  pointRadius_vector) > 0) {
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
DynamicPlannerNode::genPointCloudFromVec(vector<double> &x_,
                                         vector<double> &y_) {
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
    const geometry_msgs::msg::Pose &ego_pt_global_,
    const geometry_msgs::msg::Pose &target_pt_global_,
    pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
  auto ego_progress = getProgress(ego_pt_global_, target_tree_);
  auto target_progress = getProgress(target_pt_global_, target_tree_);

  // TODO : progress wrapping is needed!!!!!!!!!!!!!!!!!!!
  return ego_progress - target_progress;
}

nav_msgs::msg::Path
DynamicPlannerNode::xyyawVec2Path(std::vector<double> &x_,
                                  std::vector<double> &y_,
                                  std::vector<double> &yaw_rad_) {
  nav_msgs::msg::Path output;
  output.header.frame_id = nif::common::frame_id::localization::ODOM;

  for (int i = 0; i < x_.size(); i++) {
    geometry_msgs::msg::PoseStamped pt;
    pt.pose.position.x = x_[i];
    pt.pose.position.y = y_[i];
    pt.pose.position.z = 0.0;
    pt.pose.orientation =
        nif::common::utils::coordination::euler2quat(yaw_rad_[i], 0.0, 0.0);
    output.poses.push_back(pt);
  }

  return output;
}

tuple<double, double> DynamicPlannerNode::calcProgressNCTE(
    const geometry_msgs::msg::Pose &pt_global_,
    pcl::KdTreeFLANN<pcl::PointXY> &target_tree_,
    pcl::PointCloud<pcl::PointXY>::Ptr &pc_) {
  double cte = 0;
  double progress;
  int sign;

  std::vector<int> pointId_vector;
  std::vector<float> pointRadius_vector;
  pcl::PointXY *searchPoint = new pcl::PointXY();
  searchPoint->x = pt_global_.position.x;
  searchPoint->y = pt_global_.position.y;

  if (target_tree_.nearestKSearch(*searchPoint, 1, pointId_vector,
                                  pointRadius_vector) > 0) {
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

tuple<double, double>
DynamicPlannerNode::calcProgressNCTE(const geometry_msgs::msg::Pose &pt_global_,
                                     nav_msgs::msg::Path &target_path_) {
  double cte = 0;
  double progress;
  int sign;
  int closest_idx = 0;

  double min_dist = 1000000000;

  for (int i = 0; i < target_path_.poses.size(); i++) {
    double dist = sqrt(
        pow(pt_global_.position.x - target_path_.poses[i].pose.position.x, 2) +
        pow(pt_global_.position.y - target_path_.poses[i].pose.position.y, 2));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  int next_idx = closest_idx + 1;
  double next_x, next_y;
  double cur_x, cur_y;
  if (target_path_.poses.size() <= next_idx) {
    next_idx = next_idx - target_path_.poses.size();
  }

  cur_x = target_path_.poses[closest_idx].pose.position.x;
  cur_y = target_path_.poses[closest_idx].pose.position.y;

  next_x = target_path_.poses[next_idx].pose.position.x;
  next_y = target_path_.poses[next_idx].pose.position.y;

  // vector 1 : (next_x - pt_global_.position.x , next_y - pt_global_.position.y
  // , )
  // vector 2 : (cur_x - pt_global_.position.x , cur_y - pt_global_.position.y
  // , )

  auto cross_product =
      (next_x - pt_global_.position.x) * (cur_y - pt_global_.position.y) -
      (cur_x - pt_global_.position.x) * (next_y - pt_global_.position.y);
  if (cross_product < 0) {
    sign = 1;
  } else {
    sign = -1;
  }

  cte = min_dist * sign;
  progress = closest_idx * m_config_spline_interval; // global progress

  return std::make_tuple(progress, cte);
}

std::shared_ptr<FrenetPath> DynamicPlannerNode::getFrenetToRacingLine() {
  // Generate trajectory segment from current odom to racing line.
  auto progressNcte = calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);

  std::tuple<std::shared_ptr<FrenetPath>,
             std::vector<std::shared_ptr<FrenetPath>>>
      frenet_path_generation_result = m_frenet_generator_ptr->calc_frenet_paths(
          get<1>(progressNcte),             // current_position_d
          get<0>(progressNcte),             // current_position_s
          0.0,                              // current_velocity_d
          m_ego_odom.twist.twist.linear.x,  // current_velocity_s
          0.0,                              // current_acceleration_d
          get<4>(m_racingline_spline_data), // cubic_spliner_2D
          SEC_4, SEC_4 + 0.01, SAMPLING_TIME, 0.0, 0.0001, 0.1);

  //   std::shared_ptr<FrenetPath>& predicted_frenet_path =
  //       std::get<0>(frenet_path_generation_result);
  return std::get<0>(frenet_path_generation_result);
}

int DynamicPlannerNode::calcCurIdxFromDynamicTraj(
    const nif_msgs::msg::DynamicTrajectory &msg) {
  int cur_idx = 0;
  double min_dist = 1000000000;

  for (int i = 0; i < msg.trajectory_path.poses.size(); i++) {
    double dist = sqrt(pow(m_ego_odom.pose.pose.position.x -
                               msg.trajectory_path.poses[i].pose.position.x,
                           2) +
                       pow(m_ego_odom.pose.pose.position.y -
                               msg.trajectory_path.poses[i].pose.position.y,
                           2));
    if (dist < min_dist) {
      min_dist = dist;
      cur_idx = i;
    }
  }
  return cur_idx;
}

bool DynamicPlannerNode::collisionCheckBTWtrajs(
    const nif_msgs::msg::DynamicTrajectory &ego_traj_,
    const nif_msgs::msg::DynamicTrajectory &oppo_traj_,
    const double collision_dist_boundary,
    const double collision_time_boundary) {
  // if there is collision, return true
  if (oppo_traj_.trajectory_path.poses.empty()) {
    return false;
  }

  bool is_collision = false;
  for (int ego_traj_idx = 0;
       ego_traj_idx < ego_traj_.trajectory_path.poses.size(); ego_traj_idx++) {
    for (int oppo_traj_idx = 0;
         oppo_traj_idx < oppo_traj_.trajectory_path.poses.size();
         oppo_traj_idx++) {
      double dist = sqrt(
          pow((ego_traj_.trajectory_path.poses[ego_traj_idx].pose.position.x -
               oppo_traj_.trajectory_path.poses[oppo_traj_idx].pose.position.x),
              2) +
          pow((ego_traj_.trajectory_path.poses[ego_traj_idx].pose.position.y -
               oppo_traj_.trajectory_path.poses[oppo_traj_idx].pose.position.y),
              2));

      double time_diff =
          abs(ego_traj_.trajectory_timestamp_array[ego_traj_idx] -
              oppo_traj_.trajectory_timestamp_array[oppo_traj_idx]);

      if (dist < collision_dist_boundary &&
          time_diff < collision_time_boundary) {
        is_collision = true;
        return is_collision;
      }
    }
  }
  return is_collision;
}

bool DynamicPlannerNode::collisionCheckBTWtrajsNFrenet(
    std::shared_ptr<FrenetPath> ego_frenet_traj_,
    const nif_msgs::msg::DynamicTrajectory &oppo_traj_,
    const double collision_dist_boundary,
    const double collision_time_boundary) {
  // if there is collision, return true.
  bool is_collision = false;

  vector<double> ego_frenet_x = ego_frenet_traj_->points_x();
  vector<double> ego_frenet_y = ego_frenet_traj_->points_y();
  vector<double> ego_frenet_time = ego_frenet_traj_->time();

  for (int ego_traj_idx = 0; ego_traj_idx < ego_frenet_x.size();
       ego_traj_idx++) {
    for (int oppo_traj_idx = 0;
         oppo_traj_idx < oppo_traj_.trajectory_path.poses.size();
         oppo_traj_idx++) {
      double dist = sqrt(
          pow((ego_frenet_x[ego_traj_idx] -
               oppo_traj_.trajectory_path.poses[oppo_traj_idx].pose.position.x),
              2) +
          pow((ego_frenet_y[ego_traj_idx] -
               oppo_traj_.trajectory_path.poses[oppo_traj_idx].pose.position.y),
              2));

      double time_diff =
          abs(ego_frenet_time[ego_traj_idx] -
              oppo_traj_.trajectory_timestamp_array[oppo_traj_idx]);

      if (dist < collision_dist_boundary &&
          time_diff < collision_time_boundary) {
        is_collision = true;
        return is_collision;
      }
    }
  }
  return is_collision;
}

nif_msgs::msg::DynamicTrajectory DynamicPlannerNode::stitchFrenetToPath(
    std::shared_ptr<FrenetPath> &frenet_segment_,
    pcl::KdTreeFLANN<pcl::PointXY> &target_tree_,
    nav_msgs::msg::Path &target_path_) {
  nif_msgs::msg::DynamicTrajectory out;

  // find closest index of target_path with respect to the start point of the
  // frenet segment
  auto vec_x = frenet_segment_->points_x();
  auto vec_y = frenet_segment_->points_y();
  auto vec_yaw = frenet_segment_->yaw();

  auto cloest_pt_idx_wrt_segment_start_pt =
      getCurIdx(vec_x[0], vec_y[0], target_tree_);
  auto cloest_pt_idx_wrt_segment_end_pt =
      getCurIdx(vec_x.back(), vec_y.back(), target_tree_);

  for (int i = 0; i < vec_x.size(); i++) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = vec_x[i];
    ps.pose.position.y = vec_y[i];
    ps.pose.orientation =
        nif::common::utils::coordination::euler2quat(vec_yaw[i], 0.0, 0.0);

    out.trajectory_path.poses.push_back(ps);
  }

  if (cloest_pt_idx_wrt_segment_start_pt > cloest_pt_idx_wrt_segment_end_pt) {
    // index wrapping
    out.trajectory_path.poses.insert(
        out.trajectory_path.poses.end(),
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_end_pt,
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
  } else {
    out.trajectory_path.poses.insert(out.trajectory_path.poses.end(),
                                     target_path_.poses.begin() +
                                         cloest_pt_idx_wrt_segment_end_pt,
                                     target_path_.poses.end());
    out.trajectory_path.poses.insert(
        out.trajectory_path.poses.end(), target_path_.poses.begin(),
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
  }
  return out;
}

nif_msgs::msg::DynamicTrajectory DynamicPlannerNode::stitchFrenetToPath(
    std::shared_ptr<FrenetPath> &frenet_segment_,
    nav_msgs::msg::Path &target_path_) {
  nif_msgs::msg::DynamicTrajectory out;

  auto vec_x = frenet_segment_->points_x();
  auto vec_y = frenet_segment_->points_y();
  auto vec_yaw = frenet_segment_->yaw();

  // find closest index of target_path with respect to the start point of the
  // frenet segment
  auto cloest_pt_idx_wrt_segment_start_pt =
      getCurIdx(vec_x[0], vec_y[0], target_path_);
  // find closest index of target_path with respect to the end point of the
  // frenet segment + 2 (to cope with the case that the nearest point is behind
  // )
  auto cloest_pt_idx_wrt_segment_end_pt =
      getCurIdx(vec_x.back(), vec_y.back(), target_path_) + 2;

  m_reset_wpt_idx = cloest_pt_idx_wrt_segment_end_pt;

  // Index wrapping for end point of the frenet
  if (cloest_pt_idx_wrt_segment_end_pt >= target_path_.poses.size()) {
    cloest_pt_idx_wrt_segment_end_pt -= target_path_.poses.size();
  }

  // Add frenet part first
  for (int i = 0; i < vec_x.size(); i++) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = vec_x[i];
    ps.pose.position.y = vec_y[i];
    ps.pose.orientation =
        nif::common::utils::coordination::euler2quat(vec_yaw[i], 0.0, 0.0);
    ps.header.frame_id = "odom";
    out.trajectory_path.poses.push_back(ps);
  }

  if (cloest_pt_idx_wrt_segment_start_pt > cloest_pt_idx_wrt_segment_end_pt) {
    // index wrapping (frenet start from end part of the target path.)
    out.trajectory_path.poses.insert(
        out.trajectory_path.poses.end(),
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_end_pt,
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
  } else {
    out.trajectory_path.poses.insert(out.trajectory_path.poses.end(),
                                     target_path_.poses.begin() +
                                         cloest_pt_idx_wrt_segment_end_pt,
                                     target_path_.poses.end());
    out.trajectory_path.poses.insert(
        out.trajectory_path.poses.end(), target_path_.poses.begin(),
        target_path_.poses.begin() + cloest_pt_idx_wrt_segment_start_pt);
  }

  return out;
}

double DynamicPlannerNode::getProgress(
    const geometry_msgs::msg::Pose &pt_global_,
    const nif_msgs::msg::DynamicTrajectory &target_traj) {
  return getProgress(pt_global_.position.x, pt_global_.position.y, target_traj);
}

double DynamicPlannerNode::getProgress(
    const double &pt_x_, const double &pt_y_,
    const nif_msgs::msg::DynamicTrajectory &target_traj) {
  double out;

  int closest_idx = 0;
  double min_dist = 1000000000;

  for (int i = 0; i < target_traj.trajectory_path.poses.size(); i++) {
    double dist = sqrt(
        pow(pt_x_ - target_traj.trajectory_path.poses[i].pose.position.x, 2) +
        pow(pt_y_ - target_traj.trajectory_path.poses[i].pose.position.y, 2));
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }

  return closest_idx * m_config_spline_interval;
}

nav_msgs::msg::Path DynamicPlannerNode::getIntervalPath(
    const geometry_msgs::msg::Pose &start_global_,
    const geometry_msgs::msg::Pose &end_global_,
    const nif_msgs::msg::DynamicTrajectory &target_traj) {
  // Inside here, progress wrapping is done.

  return getIntervalPath(start_global_.position.x, start_global_.position.y,
                         end_global_.position.x, end_global_.position.y,
                         target_traj);
}

nav_msgs::msg::Path DynamicPlannerNode::getIntervalPath(
    const double &start_x_, const double &start_y_, const double &end_x_,
    const double &end_y_, const nif_msgs::msg::DynamicTrajectory &target_traj) {
  // Inside here, progress wrapping is done.

  nav_msgs::msg::Path interval_path_out;

  auto start_pt_progress = getProgress(start_x_, start_y_, target_traj);
  auto end_pt_progress = getProgress(end_x_, end_y_, target_traj);
  auto start_pt_idx = int(start_pt_progress / m_config_spline_interval);
  auto end_pt_idx = int(end_pt_progress / m_config_spline_interval);

  ////////////////////
  // PROGRESS WRAPPING
  ////////////////////

  if (end_pt_idx - start_pt_progress > 0) {
    interval_path_out.poses = std::vector<geometry_msgs::msg::PoseStamped>(
        target_traj.trajectory_path.poses.begin() + start_pt_idx,
        target_traj.trajectory_path.poses.begin() + end_pt_idx);
  } else {
    interval_path_out.poses = std::vector<geometry_msgs::msg::PoseStamped>(
        target_traj.trajectory_path.poses.begin() + start_pt_idx,
        target_traj.trajectory_path.poses.end());

    interval_path_out.poses.insert(interval_path_out.poses.end(),
                                   target_traj.trajectory_path.poses.begin(),
                                   target_traj.trajectory_path.poses.begin() +
                                       end_pt_idx);
  }

  return interval_path_out;
}

nav_msgs::msg::Path DynamicPlannerNode::getCertainLenOfPathSeg(
    const double &start_x_, const double &start_y_,
    const nav_msgs::msg::Path &target_path_, const int &idx_length) {
  if (target_path_.poses.empty()) {
    RCLCPP_ERROR_ONCE(this->get_logger(),
                      "In side of getCertainLenOfPathSeg : target path is "
                      "empty. Return empty path.");
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = nif::common::frame_id::localization::ODOM;
    return empty_path;
  }

  nav_msgs::msg::Path out;
  out.header.frame_id = nif::common::frame_id::localization::ODOM;
  out.poses.resize(
      std::min(static_cast<unsigned int>(idx_length),
               static_cast<unsigned int>(target_path_.poses.size())));

  auto cur_idx_on_target_path = getCurIdx(start_x_, start_y_, target_path_);

  for (int idx = 0; idx < out.poses.size(); idx++) {
    // index wrapping
    int idx_on_target_path = cur_idx_on_target_path + idx;
    if (idx_on_target_path >= target_path_.poses.size()) {
      idx_on_target_path -= target_path_.poses.size();
    }
    out.poses[idx] = target_path_.poses[idx_on_target_path];
  }
  return out;
}

void DynamicPlannerNode::publishPlannedTrajectory(
    nif_msgs::msg::DynamicTrajectory &traj_, bool is_acc_, bool vis_) {
  traj_.header.frame_id = nif::common::frame_id::localization::ODOM;
  traj_.trajectory_type = traj_.TRAJECTORY_TYPE_PLANNING;

  if (is_acc_) {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_FOLLOW;
  } else {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_STRAIGHT;
  }

  traj_.trajectory_path.header.frame_id = traj_.header.frame_id;
  m_ego_traj_global_pub->publish(traj_);

  if (vis_) {
    m_ego_planned_vis_path_global = traj_.trajectory_path;
    m_ego_planned_vis_path_global.header.frame_id =
        nif::common::frame_id::localization::ODOM;
    m_ego_planned_vis_path_global.header.stamp = this->now();
    m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
  }
}

void DynamicPlannerNode::publishPlannedTrajectory(
    nif_msgs::msg::DynamicTrajectory &traj_, int32_t longi_type_,
    int32_t lat_type_, bool vis_) {
  traj_.header.frame_id = nif::common::frame_id::localization::ODOM;
  traj_.trajectory_type = traj_.TRAJECTORY_TYPE_PLANNING;

  if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::STRAIGHT) {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_STRAIGHT;
  } else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::FOLLOW) {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_FOLLOW;
  } else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::ESTOP) {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_ESTOP;
  } else {
    // std::cout << "Unknown type of longitudinal control type." << std::endl;
  }

  if (lat_type_ == LATERAL_PLANNING_TYPE::KEEP) {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_KEEP;
  } else if (lat_type_ == LATERAL_PLANNING_TYPE::MERGE) {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_MERGE;
  } else if (lat_type_ == LATERAL_PLANNING_TYPE::CHANGE_PATH) {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_CHANGE_PATH;
  } else {
    // std::cout << "Unknown type of lateral control type." << std::endl;
  }

  if (m_last_update_target_path_alias == "raceline") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_RACELINE;
  } else if (m_last_update_target_path_alias == "center") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_CENTER;
  } else if (m_last_update_target_path_alias == "right" ||
             m_last_update_target_path_alias == "stay_behind") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_RIGHT;
  } else if (m_last_update_target_path_alias == "left") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_LEFT;
  } else if (m_last_update_target_path_alias == "center_right") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_CENTER_RIGHT;
  } else if (m_last_update_target_path_alias == "center_left") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_CENTER_LEFT;
  } else if (m_last_update_target_path_alias == "race_ready") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_RACE_READY;
  } else if (m_last_update_target_path_alias == "defender") {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_DEFENDER;
  } else {
    // std::cout << "Unknown type of target path." << std::endl;
  }

  traj_.trajectory_path.header.frame_id = traj_.header.frame_id;
  m_ego_traj_global_pub->publish(traj_);

  if (vis_) {
    m_ego_planned_vis_path_global = traj_.trajectory_path;
    m_ego_planned_vis_path_global.header.frame_id =
        nif::common::frame_id::localization::ODOM;
    m_ego_planned_vis_path_global.header.stamp = this->now();
    m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
  }
}

void DynamicPlannerNode::checkSwitchToStaticWPT(int cur_wpt_idx_) {

  if (cur_wpt_idx_ >= m_reset_wpt_idx &&
      m_reset_wpt_idx != RESET_PATH_TYPE::NONE &&
      m_reset_target_path_idx != RESET_PATH_TYPE::NONE) {
    if (m_reset_target_path_idx == RESET_PATH_TYPE::RACE_LINE) {
      // target path is the racing line
      m_cur_planned_traj.trajectory_path = m_racingline_path;
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;

    } else if (m_reset_target_path_idx == RESET_PATH_TYPE::DEFENDER_LINE) {
      // target path is the racing line
      m_cur_planned_traj.trajectory_path = m_defenderline_path;
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;

    } else if (m_reset_target_path_idx == RESET_PATH_TYPE::STAY_BEHIND) {
      // target path is the racing line
      m_cur_planned_traj.trajectory_path = m_staybehind_path;
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;

    } else {
      // target path is one of the overtaking path candidates
      m_cur_planned_traj.trajectory_path =
          m_overtaking_candidates_path_vec[m_reset_target_path_idx];
      m_reset_wpt_idx = RESET_PATH_TYPE::NONE;
      m_reset_target_path_idx = RESET_PATH_TYPE::NONE;
    }
  }
}

void DynamicPlannerNode::publishPlannedTrajectory(
    nif_msgs::msg::DynamicTrajectory &traj_, int longi_type_, int lat_type_,
    int target_path_, bool vis_) {
  traj_.header.frame_id = nif::common::frame_id::localization::ODOM;
  traj_.trajectory_type = traj_.TRAJECTORY_TYPE_PLANNING;

  if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::STRAIGHT) {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_STRAIGHT;
  } else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::FOLLOW) {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_FOLLOW;
  } else if (longi_type_ == LONGITUDINAL_PLANNING_TYPE::ESTOP) {
    traj_.longi_planning_type = traj_.LONGITUDINAL_PLANNING_TYPE_ESTOP;
  } else {
    // std::cout << "Unknown type of longitudinal control type." << std::endl;
  }

  if (lat_type_ == LATERAL_PLANNING_TYPE::KEEP) {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_KEEP;
  } else if (lat_type_ == LATERAL_PLANNING_TYPE::MERGE) {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_MERGE;
  } else if (lat_type_ == LATERAL_PLANNING_TYPE::CHANGE_PATH) {
    traj_.lat_planning_type = traj_.LATERAL_PLANNING_TYPE_CHANGE_PATH;
  } else {
    // std::cout << "Unknown type of lateral control type." << std::endl;
  }

  if (target_path_ == TARGET_PATH_TYPE::PATH_CENTER) {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_CENTER;
  } else if (target_path_ == TARGET_PATH_TYPE::PATH_RIGHT) {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_RIGHT;
  } else if (target_path_ == TARGET_PATH_TYPE::PATH_LEFT) {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_LEFT;
  } else if (target_path_ == TARGET_PATH_TYPE::PATH_CENTER_RIGHT) {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_CENTER_RIGHT;
  } else if (target_path_ == TARGET_PATH_TYPE::PATH_CENTER_LEFT) {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_CENTER_LEFT;
  } else if (target_path_ == TARGET_PATH_TYPE::PATH_RACE_READY) {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_RACE_READY;
  } else if (target_path_ == TARGET_PATH_TYPE::PATH_DEFENDER) {
    traj_.planning_target_path_type = traj_.PLANNING_TARGET_PATH_DEFENDER;
  } else {
    // std::cout << "Unknown type of target path." << std::endl;
  }

  traj_.trajectory_path.header.frame_id = traj_.header.frame_id;
  m_ego_traj_global_pub->publish(traj_);

  if (vis_) {
    m_ego_planned_vis_path_global = traj_.trajectory_path;
    m_ego_planned_vis_path_global.header.frame_id =
        nif::common::frame_id::localization::ODOM;
    m_ego_planned_vis_path_global.header.stamp = this->now();
    m_ego_traj_global_vis_pub->publish(m_ego_planned_vis_path_global);
  }
}
void DynamicPlannerNode::timer_callback_debug() {
  debug_ego_speed = debug_ego_speed + 0.1;

  if (debug_ego_speed > 67) {
    debug_ego_speed = 67;
  }

  m_ego_odom.twist.twist.linear.x = debug_ego_speed;

  auto allowable_maximum_vy = abs(tan(m_acceptable_slip_angle_rad)) *
                              std::max(debug_ego_speed, MIN_SPEED_MPS);
  allowable_maximum_vy = std::max(1.5, allowable_maximum_vy); // mps

  m_ego_odom.pose.pose.position.x = 69.016509;
  m_ego_odom.pose.pose.position.y = -409.4941123;

  // 329.8444616,-88.6056703
  // 206.0902806, -602.6045846
  // 90.2037954,-452.4725963
  // 69.016509,-409.4941123

  if (m_cur_planned_traj.trajectory_path.poses.empty() || true) {
    m_race_mode_first_callback = false;

    m_cur_planned_traj.trajectory_global_progress.clear();
    m_cur_planned_traj.trajectory_velocity.clear();
    m_cur_planned_traj.trajectory_timestamp_array.clear();
    m_cur_planned_traj.trajectory_path.poses.clear();

    auto progreeNCTE_racingline =
        calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);

    std::cout << "planning time : "
              << abs(get<1>(progreeNCTE_racingline) / allowable_maximum_vy)
              << std::endl;

    // Merging frenet segment generation
    // Generate single frenet path segment
    std::vector<std::shared_ptr<FrenetPath>> frenet_path_generation_result =
        m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
            get<1>(progreeNCTE_racingline), // current_position_d
            get<0>(progreeNCTE_racingline), // current_position_s
            0.0,                            // current_velocity_d
            std::max(debug_ego_speed,
                     MIN_SPEED_MPS),          // current_velocity_s
            0.0,                              // current_acceleration_d
            get<4>(m_racingline_spline_data), // cubicSplineModel
            std::max(abs(get<1>(progreeNCTE_racingline) / allowable_maximum_vy),
                     5.0),
            std::max(abs(get<1>(progreeNCTE_racingline) / allowable_maximum_vy),
                     5.0) +
                0.01,
            SAMPLING_TIME, 0.0, 0.0001, 0.1);

    if (frenet_path_generation_result.empty() ||
        frenet_path_generation_result[0]->points_x().empty()) {
      // Abnormal situation
      // publish empty & Estop trajectory
      RCLCPP_ERROR_ONCE(this->get_logger(),
                        "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                        "generation result or point vector is empty");
      m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
      m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
      nif_msgs::msg::DynamicTrajectory empty_traj;
      empty_traj.header.frame_id = nif::common::frame_id::localization::ODOM;
      publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);
      return;
    }

    m_cur_planned_traj =
        stitchFrenetToPath(frenet_path_generation_result[0], m_racingline_path);

    m_reset_wpt_idx =
        getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                  frenet_path_generation_result[0]->points_y().back(),
                  m_cur_planned_traj.trajectory_path);

    // Set the target path index to -1 which means the racing line
    m_last_update_target_path_alias = "raceline";
    m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;
  }

  auto raceline_path_seg = getCertainLenOfPathSeg(
      m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
      m_cur_planned_traj.trajectory_path, 100);

  auto race_traj = m_velocity_profiler_obj.velProfileWCollisionChecking(
      m_ego_odom, raceline_path_seg, m_cur_oppo_pred_result,
      m_config_overlap_checking_dist_bound,
      m_config_overlap_checking_time_bound, false, 1.0); // @DEBUG

  raceline_path_seg.header.frame_id = "odom";
  race_traj.trajectory_path.header.frame_id = "odom";
  m_debug_vis_pub->publish(race_traj.trajectory_path);
  m_racingline_path.header.frame_id = "odom";
  m_ego_traj_global_vis_debug_pub1->publish(m_racingline_path);
  m_ego_traj_global_debug_pub1->publish(race_traj);
}

void DynamicPlannerNode::timer_callback_rule() {

  ///////////////////////////////////////
  // race line : left side center
  // path candidate : right side center
  ///////////////////////////////////////

  // ----------------------------------------------------------------
  // --------------------- SYSTEM health check ---------------------
  // ----------------------------------------------------------------
  auto &mission_status = this->getSystemStatus().mission_status;
  double mission_max_vel = mission_status.max_velocity_mps;

  // Set the maximum accel and decel following the mission manager
  m_mission_accel_max = abs(mission_status.zone_status.long_acceleration_max);
  m_mission_decel_max = abs(
      mission_status.zone_status.long_acceleration_min); // should be negative

  // Set to velocity profiler
  auto flg = m_velocity_profiler_obj.setConstraintMaxVel(mission_max_vel);
  auto success_flg =
      m_velocity_profiler_obj.setConstraintMaxAccel(m_mission_accel_max);
  success_flg =
      m_velocity_profiler_obj.setConstraintMaxDeccel(m_mission_decel_max);

  double acc_min_dist_straight = 0.0;
  double acc_time_headway_straight = 0.0;

  auto default_planning_time_max = 0.0;
  auto default_planning_time_min = 0.0;

  if (mission_status.zone_status.zone_type ==
      mission_status.zone_status.ZONE_TYPE_STRAIGHT) {
    // acc_min_dist_straight = ;
    // acc_time_headway_straight = ;
    // m_velocity_profiler_obj.setACCMindist(acc_min_dist_straight);
    // m_velocity_profiler_obj.setACCTimeHeadway(acc_time_headway_straight);

    // set planning horizon based on zone type and id
    default_planning_time_max = SEC_3;
    default_planning_time_min = SEC_2;
  } else {
    // acc_min_dist_straight = ;
    // acc_time_headway_straight = ;
    // m_velocity_profiler_obj.setACCMindist(acc_min_dist_straight);
    // m_velocity_profiler_obj.setACCTimeHeadway(acc_time_headway_straight);

    // set planning horizon based on zone type and id
    default_planning_time_max = SEC_4;
    default_planning_time_min = SEC_3;
  }
  // ---------------------------------------------------------------

  auto allowable_maximum_vy =
      abs(tan(m_acceptable_slip_angle_rad)) *
      std::max(m_ego_odom.twist.twist.linear.x, MIN_SPEED_MPS);
  allowable_maximum_vy = std::max(1.5, allowable_maximum_vy); // mps

  // ---------------------------------------------------------------
  // --------------- Oppo prediction health check ---------------
  // ---------------------------------------------------------------
  if (!m_oppo_pred_callback_first_run) {
    if (this->now() - m_prev_oppo_pred_last_update > rclcpp::Duration(2, 0)) {
      m_cur_oppo_pred_result.trajectory_path.poses.clear();
      m_cur_oppo_pred_result.trajectory_velocity.clear();
      m_cur_oppo_pred_result.trajectory_timestamp_array.clear();
      m_cur_oppo_pred_result.trajectory_global_progress.clear();
    }
  }

  if (this->hasEgoOdometry() && !m_maptrack_global.poses.empty()) {
    // System ok
    nif::common::NodeStatusCode node_status = nif::common::NODE_OK;
    this->setNodeStatus(node_status);

    // Update ego odometry
    m_ego_odom = this->getEgoOdometry();

    // ---------------------------------------------------------------

    if (this->missionIs(nif::common::MissionStatus::MISSION_PIT_IN) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_STANDBY) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_OUT) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_TO_TRACK) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_INIT) ||
        this->missionIs(nif::common::MissionStatus::MISSION_DEFAULT)) {

      m_defender_mode_first_callback = true;
      m_race_mode_first_callback = true;
      m_keep_position_mode_first_callback = true;
      m_non_overtaking_mode_first_callback = true;

      m_cur_planned_traj.trajectory_path.poses.clear();

      // Convert maptrack to trajectory and publish (only global / without
      // ACC)
      auto cur_traj = m_velocity_profiler_obj.velProfile(
          m_ego_odom, m_maptrack_global, 1.0);

      m_last_update_target_path_alias = "race_ready";
      m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
      m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
      publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);
      return;
    } else {


      // if (mission_status.mission_status_code ==
      //     nif::common::MissionStatus::MISSION_RACE) {
        if (false) {

        m_defender_mode_first_callback = true;
        m_keep_position_mode_first_callback = true;
        m_non_overtaking_mode_first_callback = true;

        if (m_cur_planned_traj.trajectory_path.poses.empty()) {
          m_race_mode_first_callback = false;

          auto progreeNCTE_racingline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);

          // Merging frenet segment generation
          // Generate single frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_racingline), // current_position_d
                      get<0>(progreeNCTE_racingline), // current_position_s
                      0.0,                            // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_racingline_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               default_planning_time_max),
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               default_planning_time_max) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {
            // Abnormal situation
            // publish empty & Estop trajectory
            RCLCPP_ERROR_ONCE(
                this->get_logger(),
                "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          m_cur_planned_traj = stitchFrenetToPath(
              frenet_path_generation_result[0], m_racingline_path);

          m_reset_wpt_idx =
              getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                        frenet_path_generation_result[0]->points_y().back(),
                        m_cur_planned_traj.trajectory_path);

          // Set the target path index to -1 which means the racing line
          m_last_update_target_path_alias = "raceline";
          m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;
        }

        ///////////////////////////////////////////
        // Change the target path to the static wpt
        ///////////////////////////////////////////
        auto cur_idx_on_previous_path = getCurIdx(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path);

        checkSwitchToStaticWPT(cur_idx_on_previous_path);
        // ----------------------------------------------------------------

        //////////////////////////////////////////////////////////////////
        // Merging back to the racing line if the ego vehicle is far from the
        // opponent (only in the straight section --from mission mananger)
        //////////////////////////////////////////////////////////////////

        auto naive_gap = nif::common::constants::numeric::INF;
        if (!m_cur_oppo_pred_result.trajectory_path.poses.empty()) {
          naive_gap = nif::common::utils::geometry::calEuclideanDistance(
              m_ego_odom.pose.pose,
              m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
        }

        // Check merging behavior only on the straight section
        // std::cout << "----------------" << std::endl;
        // std::cout <<( mission_status.zone_status.zone_type ==
        //         mission_status.zone_status.ZONE_TYPE_STRAIGHT )<< std::endl;
        // std::cout << (naive_gap > m_merging_back_gap_thres) << std::endl;
        // std::cout << m_last_update_target_path_alias << std::endl;
        // std::cout << m_reset_target_path_idx << std::endl;

        if (mission_status.zone_status.zone_type ==
                mission_status.zone_status.ZONE_TYPE_STRAIGHT &&
            naive_gap > m_merging_back_gap_thres && // longitudinal wise
            m_last_update_target_path_alias != "raceline" &&
            m_last_update_target_path_alias != "left_center" &&
            // m_last_lat_planning_type == LATERAL_PLANNING_TYPE::KEEP &&
            m_reset_target_path_idx == RESET_PATH_TYPE::NONE) {
          auto progreeNCTE_racingline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);

          // Merging frenet segment generation
          // Generate single frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_racingline), // current_position_d
                      get<0>(progreeNCTE_racingline), // current_position_s
                      0.0,                            // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_racingline_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               default_planning_time_max),
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               default_planning_time_max) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {
            // Abnormal situation
            // publish empty & Estop trajectory
            RCLCPP_ERROR_ONCE(
                this->get_logger(),
                "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          auto stitched_path = stitchFrenetToPath(
              frenet_path_generation_result[0], m_racingline_path);

          auto raceline_path_seg = getCertainLenOfPathSeg(
              m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
              stitched_path.trajectory_path, 100);

          auto race_traj = m_velocity_profiler_obj.velProfileWCollisionChecking(
              m_ego_odom, raceline_path_seg, m_cur_oppo_pred_result,
              m_config_overlap_checking_dist_bound,
              m_config_overlap_checking_time_bound, false, 1.0); // @DEBUG

          if (!race_traj.has_collision) {
            // Change the defualt path to the racing line (full path)
            // Not considering the ACC in this case
            m_cur_planned_traj = stitched_path;
            m_reset_wpt_idx =
                getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                          frenet_path_generation_result[0]->points_y().back(),
                          m_cur_planned_traj.trajectory_path);

            m_last_update_target_path_alias = "raceline";
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::MERGE;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
            m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;

            publishPlannedTrajectory(race_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }
        }
        // ----------------------------------------------------------------

        ///////////////////////////////////////////////////
        // Velocity profiling with the previous planned path
        ///////////////////////////////////////////////////
        auto cur_path_seg = getCertainLenOfPathSeg(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path, 100);

        auto cur_traj = m_velocity_profiler_obj.velProfileWCollisionChecking(
            m_ego_odom, cur_path_seg, m_cur_oppo_pred_result,
            m_config_overlap_checking_dist_bound,
            m_config_overlap_checking_time_bound, false, 1.0); // @DEBUG

        if (!cur_traj.has_collision) {
          ///////////////////////////////////////
          // Keep current planned traj
          // not considering the ACC in this case
          ///////////////////////////////////////
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
          publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                   m_last_lat_planning_type, true);
          return;
          // ----------------------------------------------------------------
        } else {

          if (mission_status.zone_status.zone_type ==
              mission_status.zone_status.ZONE_TYPE_CORNER_MID) {
            auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
                m_ego_odom, m_cur_oppo_pred_result,
                m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
                1.0);
            // Publish cur_traj
            // Keep previous plan and do ACC
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
            publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          ////////////////////////////////////////////////////////
          // ----------- Previous path has a collision -----------
          // -------- Search for the collision-free path ---------
          ////////////////////////////////////////////////////////

          vector<std::shared_ptr<FrenetPath>> collision_free_frenet_vec;
          vector<double> collision_free_frenet_progress_vec;
          vector<int> collision_free_frenet_index_vec;

          // Generate the frenet candidates to all overtaking path candidates
          // Right side first
          for (int path_candidate_idx = 0;
               path_candidate_idx < m_overtaking_candidates_path_vec.size();
               path_candidate_idx++) {

            if (m_overtaking_candidates_path_vec[path_candidate_idx]
                    .poses.empty()) {
              continue;
            }

            auto progressNcte = calcProgressNCTE(
                m_ego_odom.pose.pose,
                m_overtaking_candidates_path_vec[path_candidate_idx]);

            std::vector<std::shared_ptr<FrenetPath>>
                frenet_path_generation_result =
                    m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                        get<1>(progressNcte), // current_position_d
                        get<0>(progressNcte), // current_position_s
                        0.0,                  // current_velocity_d
                        std::max(m_ego_odom.twist.twist.linear.x,
                                 MIN_SPEED_MPS), // current_velocity_s
                        0.0,                     // current_acceleration_d
                        m_overtaking_candidates_spline_model_vec
                            [path_candidate_idx], // cubicSplineModel
                        std::max(
                            abs(get<1>(progressNcte) / allowable_maximum_vy),
                            default_planning_time_min),
                        std::max(
                            abs(get<1>(progressNcte) / allowable_maximum_vy),
                            default_planning_time_min) +
                            2.0 + 0.01,
                        SAMPLING_TIME, 0.0, 0.0001, 0.1);

            if (frenet_path_generation_result.empty()) {
              continue;
            }

            for (int frenet_idx = frenet_path_generation_result.size() - 1;
                 frenet_idx >= 0; frenet_idx--) {
              //  Check collision in order of length of path (which means less
              //  jerky)
              auto frenet_candidate = frenet_path_generation_result[frenet_idx];

              auto stitched_path = stitchFrenetToPath(
                  frenet_candidate,
                  m_overtaking_candidates_path_vec[path_candidate_idx]);

              auto cur_path_seg =
                  getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                         m_ego_odom.pose.pose.position.y,
                                         stitched_path.trajectory_path, 100);

              auto cur_traj =
                  m_velocity_profiler_obj.velProfileWCollisionChecking(
                      m_ego_odom, cur_path_seg, m_cur_oppo_pred_result,
                      m_config_overlap_checking_dist_bound,
                      m_config_overlap_checking_time_bound, false,
                      1.0); // @DEBUG

              if (!cur_traj.has_collision) {

                m_cur_planned_traj = stitched_path;
                m_reset_wpt_idx = getCurIdx(frenet_candidate->points_x().back(),
                                            frenet_candidate->points_y().back(),
                                            m_cur_planned_traj.trajectory_path);

                m_reset_target_path_idx = path_candidate_idx;
                m_last_update_target_path_alias =
                    m_overtaking_candidates_alias_vec[path_candidate_idx];
                m_last_lat_planning_type = LATERAL_PLANNING_TYPE::CHANGE_PATH;
                m_last_long_planning_type =
                    LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
                publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                         m_last_lat_planning_type, true);
                // FIXME: Currently, if there is a collision free path, we
                // change the path immediately.
                return;

                // FIXME: For later, here stores collision-free frenet and
                // target path idx
                // (with return above, it doesn't affect anything)
                collision_free_frenet_vec.push_back(frenet_candidate);
                collision_free_frenet_index_vec.push_back(path_candidate_idx);
              }
            }
          }

          if (collision_free_frenet_vec.empty()) {
            /////////////////////////////////////////
            // All path are cancled due to collisions
            /////////////////////////////////////////
            auto cur_path_seg =
                getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                       m_ego_odom.pose.pose.position.y,
                                       m_cur_planned_traj.trajectory_path, 100);

            auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
                m_ego_odom, m_cur_oppo_pred_result,
                m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
                1.0);

            // Publish cur_traj
            // Keep previous plan and do ACC
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
            publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          } else {
            // ////////////////////////////////////////////////////////
            // There are more than one collision-free frenet candidates
            // ////////////////////////////////////////////////////////

            // Calculate the progress for each trajectory
            std::vector<nif_msgs::msg::DynamicTrajectory> stitched_traj_vec;
            std::vector<nif_msgs::msg::DynamicTrajectory> planned_traj_vec;
            std::vector<double> estimated_arrive_time_vec;

            stitched_traj_vec.resize(collision_free_frenet_vec.size());
            planned_traj_vec.resize(collision_free_frenet_vec.size());
            estimated_arrive_time_vec.resize(collision_free_frenet_vec.size());

            for (int collision_free_frenet_idx = 0;
                 collision_free_frenet_idx < collision_free_frenet_vec.size();
                 collision_free_frenet_idx++) {
              auto stitch_target_path_candidate_idx =
                  collision_free_frenet_index_vec[collision_free_frenet_idx];

              auto stitched_path = stitchFrenetToPath(
                  collision_free_frenet_vec[collision_free_frenet_idx],
                  m_overtaking_candidates_path_vec
                      [stitch_target_path_candidate_idx]);

              auto path_seg =
                  getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                         m_ego_odom.pose.pose.position.y,
                                         stitched_path.trajectory_path, 100);

              auto planned_traj =
                  m_velocity_profiler_obj.velProfile(m_ego_odom, path_seg, 1.0);

              stitched_traj_vec[collision_free_frenet_idx] = stitched_path;
              planned_traj_vec[collision_free_frenet_idx] = planned_traj;
              estimated_arrive_time_vec[collision_free_frenet_idx] =
                  planned_traj.trajectory_timestamp_array.back();
            }

            auto naive_max_progree_path_idx =
                std::min_element(estimated_arrive_time_vec.begin(),
                                 estimated_arrive_time_vec.end()) -
                estimated_arrive_time_vec.begin();

            m_cur_planned_traj = stitched_traj_vec[naive_max_progree_path_idx];

            m_reset_wpt_idx =
                getCurIdx(collision_free_frenet_vec[naive_max_progree_path_idx]
                              ->points_x()
                              .back(),
                          collision_free_frenet_vec[naive_max_progree_path_idx]
                              ->points_y()
                              .back(),
                          m_cur_planned_traj.trajectory_path);

            m_reset_target_path_idx =
                collision_free_frenet_index_vec[naive_max_progree_path_idx];
            m_last_update_target_path_alias = m_overtaking_candidates_alias_vec
                [collision_free_frenet_index_vec[naive_max_progree_path_idx]];
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::CHANGE_PATH;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
            publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }
        }

      } 
      // else if (mission_status.mission_status_code ==
      //            nif::common::MissionStatus::MISSION_CONSTANT_SPEED) {
      else if (true) {
        // ----------------------------------------------
        // ---------------- Defender mode ---------------
        // --------------- Drive innerline --------------
        // ----------------- ACC activate ---------------
        // ----------------------------------------------

        m_race_mode_first_callback = true;
        m_keep_position_mode_first_callback = true;
        m_non_overtaking_mode_first_callback = true;

        ///////////////////////////////
        // defender mode first callback
        ///////////////////////////////
        if (m_defender_mode_first_callback == true ||
            m_cur_planned_traj.trajectory_path.poses.empty()) {
          m_defender_mode_first_callback = false;

          // Switch to the defender line
          auto progreeNCTE_defenderline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_defenderline_path);

          // Merging frenet segment generation
          // Generate SINGLE frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_defenderline), // current_position_d
                      get<0>(progreeNCTE_defenderline), // current_position_s
                      0.0,                              // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_defenderline_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_defenderline) /
                                   allowable_maximum_vy),
                               default_planning_time_max),
                      std::max(abs(get<1>(progreeNCTE_defenderline) /
                                   allowable_maximum_vy),
                               default_planning_time_max) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {
            // Abnormal situation
            // publish empty & Estop trajectory
            // @DEBUG:
            RCLCPP_ERROR_ONCE(this->get_logger(),
                              "CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                              "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          m_cur_planned_traj = stitchFrenetToPath(
              frenet_path_generation_result[0], m_defenderline_path);

          m_last_update_target_path_alias = "defender";

          m_reset_wpt_idx =
              getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                        frenet_path_generation_result[0]->points_y().back(),
                        m_cur_planned_traj.trajectory_path);

          m_reset_target_path_idx = RESET_PATH_TYPE::DEFENDER_LINE;
        }
        // Keep previous plan

        ///////////////////////////////////////////
        // Change the target path to the static wpt
        ///////////////////////////////////////////
        auto cur_idx_on_previous_path = getCurIdx(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path);

        checkSwitchToStaticWPT(cur_idx_on_previous_path);
        // -----------------------------------------

        auto cur_path_seg = getCertainLenOfPathSeg(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path, 100);

        auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
            m_ego_odom, m_cur_oppo_pred_result,
            m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
            1.0);

        // Publish cur_traj
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                 m_last_lat_planning_type, true);
        return;
      } else {

        ////////////////////////////
        // Overtaking is not allowed
        // Stay behind on the right side of the track
        ////////////////////////////

        m_defender_mode_first_callback = true;
        m_race_mode_first_callback = true;
        m_keep_position_mode_first_callback = true;

        if (m_cur_planned_traj.trajectory_path.poses.empty()) {
          m_non_overtaking_mode_first_callback = false;

          auto progreeNCTE_racingline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);

          // Merging frenet segment generation
          // Generate single frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_racingline), // current_position_d
                      get<0>(progreeNCTE_racingline), // current_position_s
                      0.0,                            // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_racingline_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               default_planning_time_max),
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               default_planning_time_max) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {

            // Abnormal situation
            // publish empty & Estop trajectory
            RCLCPP_ERROR_ONCE(this->get_logger(),
                              "[MISSION_KEEP_POSITION] CRITICAL BUG HAS BEEN "
                              "HIT.\n Frenet path "
                              "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          auto stitched_path = stitchFrenetToPath(
              frenet_path_generation_result[0], m_racingline_path);

          m_cur_planned_traj = stitched_path;

          m_reset_wpt_idx =
              getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                        frenet_path_generation_result[0]->points_y().back(),
                        m_cur_planned_traj.trajectory_path);

          m_last_update_target_path_alias = "raceline";
          m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;
        }

        // Keep previous plan
        ///////////////////////////////////////////
        // Change the target path to the static wpt
        ///////////////////////////////////////////
        auto cur_idx_on_previous_path = getCurIdx(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path);

        checkSwitchToStaticWPT(cur_idx_on_previous_path);
        // -----------------------------------------

        //////////////////////////////////////////////////////////////////
        // Merging to the right side if the ego vehicle is close from the
        // opponent (only in the straight section --from mission mananger)
        //////////////////////////////////////////////////////////////////

        auto naive_gap = nif::common::constants::numeric::INF;
        if (!m_cur_oppo_pred_result.trajectory_path.poses.empty()) {
          naive_gap = nif::common::utils::geometry::calEuclideanDistance(
              m_ego_odom.pose.pose,
              m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
        }

        // Check merging behavior only on the straight section
        if (mission_status.zone_status.zone_type ==
                mission_status.zone_status.ZONE_TYPE_STRAIGHT &&
            naive_gap < 150 &&
            m_last_update_target_path_alias != "right_center" &&
            m_last_update_target_path_alias != "stay_behind" &&
            m_reset_target_path_idx == RESET_PATH_TYPE::NONE) {

          auto progreeNCTE_staybehind =
              calcProgressNCTE(m_ego_odom.pose.pose, m_staybehind_path);

          // Merging frenet segment generation
          // Generate single frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_staybehind), // current_position_d
                      get<0>(progreeNCTE_staybehind), // current_position_s
                      0.0,                            // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_staybehind_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_staybehind) /
                                   allowable_maximum_vy),
                               default_planning_time_max),
                      std::max(abs(get<1>(progreeNCTE_staybehind) /
                                   allowable_maximum_vy),
                               default_planning_time_max) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {
            // Abnormal situation
            // publish empty & Estop trajectory
            RCLCPP_ERROR_ONCE(
                this->get_logger(),
                "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          auto stitched_path = stitchFrenetToPath(
              frenet_path_generation_result[0], m_staybehind_path);

          auto staybehind_path_seg = getCertainLenOfPathSeg(
              m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
              stitched_path.trajectory_path, 100);

          auto staybehind_traj =
              m_velocity_profiler_obj.velProfileWCollisionChecking(
                  m_ego_odom, staybehind_path_seg, m_cur_oppo_pred_result,
                  m_config_overlap_checking_dist_bound,
                  m_config_overlap_checking_time_bound, false, 1.0); // @DEBUG

          if (!staybehind_traj.has_collision) {
            // Change the defualt path to the racing line (full path)
            // Not considering the ACC in this case
            m_cur_planned_traj = stitched_path;
            m_reset_wpt_idx =
                getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                          frenet_path_generation_result[0]->points_y().back(),
                          m_cur_planned_traj.trajectory_path);

            m_last_update_target_path_alias = "stay_behind";
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::MERGE;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
            m_reset_target_path_idx = RESET_PATH_TYPE::STAY_BEHIND;

            publishPlannedTrajectory(staybehind_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }
        }

        // Keep previous plan and ACC trajectory generation
        auto cur_path_seg = getCertainLenOfPathSeg(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path, 100);

        auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
            m_ego_odom, m_cur_oppo_pred_result,
            m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
            1.0);

        // Publish cur_traj
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                 m_last_lat_planning_type, true);
        return;
      }
    }
  } else {
    // ESTOP
    nif::common::NodeStatusCode node_status = nif::common::NODE_INITIALIZED;
    this->setNodeStatus(node_status);
  }
}

void DynamicPlannerNode::timer_callback() {
  // ----------------------------------------------------------------
  // SYSTEM health check
  auto mission_status = this->getSystemStatus().mission_status;
  double mission_max_vel =
      this->getSystemStatus().mission_status.max_velocity_mps;

  // Mission maximum vel set to the profiler
  auto flg = m_velocity_profiler_obj.setConstraintMaxVel(mission_max_vel);
  // ---------------------------------------------------------------

  auto allowable_maximum_vy =
      abs(tan(m_acceptable_slip_angle_rad)) *
      std::max(m_ego_odom.twist.twist.linear.x, MIN_SPEED_MPS);
  allowable_maximum_vy = std::max(1.5, allowable_maximum_vy); // mps

  // ---------------------------------------------------------------

  // Oppo prediction health check
  // TODO improve this
  if (!m_oppo_pred_callback_first_run) {
    if (this->now() - m_prev_oppo_pred_last_update > rclcpp::Duration(2, 0)) {
      m_cur_oppo_pred_result.trajectory_path.poses.clear();
      m_cur_oppo_pred_result.trajectory_timestamp_array.clear();
    }
  }

  if (this->hasEgoOdometry() && !m_maptrack_global.poses.empty()) {
    // System ok
    nif::common::NodeStatusCode node_status = nif::common::NODE_OK;
    this->setNodeStatus(node_status);

    // Update ego odometry
    m_ego_odom = this->getEgoOdometry();

    // ---------------------------------------------------------------

    // if (mission_status.mission_status_code ==
    //         nif::common::MissionStatus::MISSION_PIT_IN ||
    //     mission_status.mission_status_code ==
    //         nif::common::MissionStatus::MISSION_PIT_STANDBY) {
    if (this->missionIs(nif::common::MissionStatus::MISSION_PIT_IN) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_STANDBY) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_OUT) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_TO_TRACK) ||
        this->missionIs(nif::common::MissionStatus::MISSION_PIT_INIT) ||
        this->missionIs(nif::common::MissionStatus::MISSION_DEFAULT)) {
      m_defender_mode_first_callback = true;
      m_race_mode_first_callback = true;
      m_keep_position_mode_first_callback = true;
      m_non_overtaking_mode_first_callback = true;

      // Convert maptrack to trajectory and publish (only global / without
      // ACC)
      auto cur_traj = m_velocity_profiler_obj.velProfile(
          m_ego_odom, m_maptrack_global, 1.0);

      m_last_update_target_path_alias = "race_ready";
      m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
      m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
      publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                               m_last_lat_planning_type, true);
      return;
    } else {
      if (mission_status.mission_status_code ==
          nif::common::MissionStatus::MISSION_RACE) {
        m_defender_mode_first_callback = true;
        m_keep_position_mode_first_callback = true;
        m_non_overtaking_mode_first_callback = true;

        if (m_race_mode_first_callback == true ||
            m_cur_planned_traj.trajectory_path.poses.empty()) {
          m_race_mode_first_callback = false;

          auto progreeNCTE_racingline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);

          // Merging frenet segment generation
          // Generate single frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_racingline), // current_position_d
                      get<0>(progreeNCTE_racingline), // current_position_s
                      0.0,                            // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_racingline_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               2.0),
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               2.0) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {
            // Abnormal situation
            // publish empty & Estop trajectory
            RCLCPP_ERROR_ONCE(
                this->get_logger(),
                "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          m_cur_planned_traj = stitchFrenetToPath(
              frenet_path_generation_result[0], m_racingline_path);

          m_reset_wpt_idx =
              getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                        frenet_path_generation_result[0]->points_y().back(),
                        m_cur_planned_traj.trajectory_path);

          // Set the target path index to -1 which means the racing line
          m_last_update_target_path_alias = "raceline";
          m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;
        }

        ///////////////////////////////////////////
        // Change the target path to the static wpt
        ///////////////////////////////////////////
        auto cur_idx_on_previous_path = getCurIdx(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path);

        checkSwitchToStaticWPT(cur_idx_on_previous_path);
        // ----------------------------------------------------------------

        //////////////////////////////////////////////////////////////////
        // Merging back to the racing line if the ego vehicle is close and
        // collision-free
        //////////////////////////////////////////////////////////////////

        auto naive_gap = nif::common::constants::numeric::INF;
        if (!m_cur_oppo_pred_result.trajectory_path.poses.empty()) {
          naive_gap = nif::common::utils::geometry::calEuclideanDistance(
              m_ego_odom.pose.pose,
              m_cur_oppo_pred_result.trajectory_path.poses.front().pose);
        }

        if (naive_gap > m_merging_back_gap_thres && // longitudinal wise
            m_last_update_target_path_alias != "raceline" &&
            m_last_lat_planning_type == LATERAL_PLANNING_TYPE::KEEP &&
            m_reset_target_path_idx == RESET_PATH_TYPE::NONE) {
          auto progreeNCTE_racingline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);
          bool is_close_racingline =
              (get<1>(progreeNCTE_racingline) <
               m_config_merge_allow_dist); // lateral wise

          if (is_close_racingline) {
            // Merging frenet segment generation
            // Generate single frenet path segment
            std::vector<std::shared_ptr<FrenetPath>>
                frenet_path_generation_result =
                    m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                        get<1>(progreeNCTE_racingline), // current_position_d
                        get<0>(progreeNCTE_racingline), // current_position_s
                        0.0,                            // current_velocity_d
                        std::max(m_ego_odom.twist.twist.linear.x,
                                 MIN_SPEED_MPS), // current_velocity_s
                        0.0,                     // current_acceleration_d
                        get<4>(m_racingline_spline_data), // cubicSplineModel
                        std::max(abs(get<1>(progreeNCTE_racingline) /
                                     allowable_maximum_vy),
                                 2.0),
                        std::max(abs(get<1>(progreeNCTE_racingline) /
                                     allowable_maximum_vy),
                                 2.0) +
                            0.01,
                        SAMPLING_TIME, 0.0, 0.0001, 0.1);

            if (frenet_path_generation_result.empty() ||
                frenet_path_generation_result[0]->points_x().empty()) {
              // Abnormal situation
              // publish empty & Estop trajectory
              RCLCPP_ERROR_ONCE(
                  this->get_logger(),
                  "[RACE MODE] CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                  "generation result or point vector is empty");
              m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
              m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
              nif_msgs::msg::DynamicTrajectory empty_traj;
              empty_traj.header.frame_id =
                  nif::common::frame_id::localization::ODOM;
              publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                       m_last_lat_planning_type, true);
              return;
            }

            auto stitched_path = stitchFrenetToPath(
                frenet_path_generation_result[0], m_racingline_path);

            auto raceline_path_seg =
                getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                       m_ego_odom.pose.pose.position.y,
                                       stitched_path.trajectory_path, 100);

            // Convert to the trajectory with the velocity profiling
            // (without considering ACC)

            // auto race_traj = m_velocity_profiler_obj.velProfile(
            //     m_ego_odom, raceline_path_seg, 1.0);
            // auto has_collision = collisionCheckBTWtrajs(
            //     race_traj, m_cur_oppo_pred_result,
            //     m_config_overlap_checking_dist_bound,
            //     m_config_overlap_checking_time_bound, m_use_sat);

            auto race_traj =
                m_velocity_profiler_obj.velProfileWCollisionChecking(
                    m_ego_odom, raceline_path_seg, m_cur_oppo_pred_result,
                    m_config_overlap_checking_dist_bound,
                    m_config_overlap_checking_time_bound, false, 1.0); // @DEBUG

            if (!race_traj.has_collision) {
              // Change the defualt path to the racing line (full path)
              // Not considering the ACC in this case
              m_cur_planned_traj = stitched_path;
              m_reset_wpt_idx =
                  getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                            frenet_path_generation_result[0]->points_y().back(),
                            m_cur_planned_traj.trajectory_path);

              m_last_update_target_path_alias = "raceline";
              m_last_lat_planning_type = LATERAL_PLANNING_TYPE::MERGE;
              m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
              m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;

              publishPlannedTrajectory(race_traj, m_last_long_planning_type,
                                       m_last_lat_planning_type, true);
              return;
            }
          }
        }
        // ----------------------------------------------------------------

        ///////////////////////////////////////////////////
        // Velocity profiling with the previous planned path
        // (Not close enough to the racing line or colliding)
        ///////////////////////////////////////////////////
        auto cur_path_seg = getCertainLenOfPathSeg(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path, 100);

        // auto cur_traj =
        //     m_velocity_profiler_obj.velProfile(m_ego_odom,
        //     cur_path_seg, 1.0);
        // // Collision check btw two trajectories
        // auto has_collision = collisionCheckBTWtrajs(
        //     cur_traj, m_cur_oppo_pred_result,
        //     m_config_overlap_checking_dist_bound,
        //     m_config_overlap_checking_time_bound, m_use_sat);

        auto cur_traj = m_velocity_profiler_obj.velProfileWCollisionChecking(
            m_ego_odom, cur_path_seg, m_cur_oppo_pred_result,
            m_config_overlap_checking_dist_bound,
            m_config_overlap_checking_time_bound, false, 1.0); // @DEBUG

        if (!cur_traj.has_collision) {
          ///////////////////////////////////////
          // Keep current planned traj
          // not considering the ACC in this case
          ///////////////////////////////////////
          m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
          m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
          publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                   m_last_lat_planning_type, true);
          return;
          // ----------------------------------------------------------------

        } else {
          ////////////////////////////////////////////////////////
          // ----------- Previous path has a collision -----------
          // -------- Search for the collision-free path ---------
          ////////////////////////////////////////////////////////

          vector<std::shared_ptr<FrenetPath>> collision_free_frenet_vec;
          vector<double> collision_free_frenet_progress_vec;
          vector<int> collision_free_frenet_index_vec;

          // Generate the frenet candidates to all overtaking path candidates
          // Right side first
          for (int path_candidate_idx = 0;
               path_candidate_idx < m_overtaking_candidates_path_vec.size();
               path_candidate_idx++) {
            if (m_overtaking_candidates_path_vec[path_candidate_idx]
                    .poses.empty()) {
              continue;
            }

            auto progressNcte = calcProgressNCTE(
                m_ego_odom.pose.pose,
                m_overtaking_candidates_path_vec[path_candidate_idx]);

            std::vector<std::shared_ptr<FrenetPath>>
                frenet_path_generation_result =
                    m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                        get<1>(progressNcte), // current_position_d
                        get<0>(progressNcte), // current_position_s
                        0.0,                  // current_velocity_d
                        std::max(m_ego_odom.twist.twist.linear.x,
                                 MIN_SPEED_MPS), // current_velocity_s
                        0.0,                     // current_acceleration_d
                        m_overtaking_candidates_spline_model_vec
                            [path_candidate_idx], // cubicSplineModel
                        std::max(
                            abs(get<1>(progressNcte) / allowable_maximum_vy),
                            2.0),
                        std::max(
                            abs(get<1>(progressNcte) / allowable_maximum_vy),
                            2.0) +
                            2.0 + 0.01,
                        SAMPLING_TIME, 0.0, 0.0001, 0.1);

            if (frenet_path_generation_result.empty()) {
              continue;
            }

            for (int frenet_idx = frenet_path_generation_result.size() - 1;
                 frenet_idx >= 0; frenet_idx--) {
              //  Check collision in order of length of path (which means less
              //  jerky)
              auto frenet_candidate = frenet_path_generation_result[frenet_idx];

              auto stitched_path = stitchFrenetToPath(
                  frenet_candidate,
                  m_overtaking_candidates_path_vec[path_candidate_idx]);

              auto cur_path_seg =
                  getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                         m_ego_odom.pose.pose.position.y,
                                         stitched_path.trajectory_path, 100);

              // auto cur_traj = m_velocity_profiler_obj.velProfile(
              //     m_ego_odom, cur_path_seg, 1.0);

              // auto has_collision = collisionCheckBTWtrajs(
              //     cur_traj, m_cur_oppo_pred_result,
              //     m_config_overlap_checking_dist_bound,
              //     m_config_overlap_checking_time_bound, m_use_sat);

              auto cur_traj =
                  m_velocity_profiler_obj.velProfileWCollisionChecking(
                      m_ego_odom, cur_path_seg, m_cur_oppo_pred_result,
                      m_config_overlap_checking_dist_bound,
                      m_config_overlap_checking_time_bound, false,
                      1.0); // @DEBUG

              if (!cur_traj.has_collision) {
                m_cur_planned_traj = stitched_path;
                m_reset_wpt_idx = getCurIdx(frenet_candidate->points_x().back(),
                                            frenet_candidate->points_y().back(),
                                            m_cur_planned_traj.trajectory_path);
                m_reset_target_path_idx = path_candidate_idx;
                m_last_update_target_path_alias =
                    m_overtaking_candidates_alias_vec[path_candidate_idx];
                m_last_lat_planning_type = LATERAL_PLANNING_TYPE::CHANGE_PATH;
                m_last_long_planning_type =
                    LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
                publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                         m_last_lat_planning_type, true);
                // FIXME: Currently, if there is a collision free path, we
                // change the path immediately.
                return;

                // FIXME: For later, here stores collision-free frenet and
                // target path idx
                // (with return above, it doesn't affect anything)
                collision_free_frenet_vec.push_back(frenet_candidate);
                collision_free_frenet_index_vec.push_back(path_candidate_idx);
              }
            }
          }

          /////////////////////////////////////////
          // All path are cancled due to collisions
          /////////////////////////////////////////
          if (collision_free_frenet_vec.empty()) {
            auto cur_path_seg =
                getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                       m_ego_odom.pose.pose.position.y,
                                       m_cur_planned_traj.trajectory_path, 100);

            auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
                m_ego_odom, m_cur_oppo_pred_result,
                m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
                1.0);

            // Publish cur_traj
            // Keep previous plan and do ACC
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
            publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          } else {
            // ////////////////////////////////////////////////////////
            // There are more than one collision-free frenet candidates
            // ////////////////////////////////////////////////////////

            // Calculate the progress for each trajectory
            std::vector<nif_msgs::msg::DynamicTrajectory> stitched_traj_vec;
            std::vector<nif_msgs::msg::DynamicTrajectory> planned_traj_vec;
            std::vector<double> estimated_arrive_time_vec;

            stitched_traj_vec.resize(collision_free_frenet_vec.size());
            planned_traj_vec.resize(collision_free_frenet_vec.size());
            estimated_arrive_time_vec.resize(collision_free_frenet_vec.size());

            for (int collision_free_frenet_idx = 0;
                 collision_free_frenet_idx < collision_free_frenet_vec.size();
                 collision_free_frenet_idx++) {
              auto stitch_target_path_candidate_idx =
                  collision_free_frenet_index_vec[collision_free_frenet_idx];

              auto stitched_path = stitchFrenetToPath(
                  collision_free_frenet_vec[collision_free_frenet_idx],
                  m_overtaking_candidates_path_vec
                      [stitch_target_path_candidate_idx]);

              auto path_seg =
                  getCertainLenOfPathSeg(m_ego_odom.pose.pose.position.x,
                                         m_ego_odom.pose.pose.position.y,
                                         stitched_path.trajectory_path, 100);

              auto planned_traj =
                  m_velocity_profiler_obj.velProfile(m_ego_odom, path_seg, 1.0);

              stitched_traj_vec[collision_free_frenet_idx] = stitched_path;
              planned_traj_vec[collision_free_frenet_idx] = planned_traj;
              estimated_arrive_time_vec[collision_free_frenet_idx] =
                  planned_traj.trajectory_timestamp_array.back();
            }

            auto naive_max_progree_path_idx =
                std::min_element(estimated_arrive_time_vec.begin(),
                                 estimated_arrive_time_vec.end()) -
                estimated_arrive_time_vec.begin();

            m_cur_planned_traj = stitched_traj_vec[naive_max_progree_path_idx];

            m_reset_wpt_idx =
                getCurIdx(collision_free_frenet_vec[naive_max_progree_path_idx]
                              ->points_x()
                              .back(),
                          collision_free_frenet_vec[naive_max_progree_path_idx]
                              ->points_y()
                              .back(),
                          m_cur_planned_traj.trajectory_path);

            m_reset_target_path_idx =
                collision_free_frenet_index_vec[naive_max_progree_path_idx];
            m_last_update_target_path_alias = m_overtaking_candidates_alias_vec
                [collision_free_frenet_index_vec[naive_max_progree_path_idx]];
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::CHANGE_PATH;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::STRAIGHT;
            publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }
        }
      } else if (mission_status.mission_status_code ==
                 nif::common::MissionStatus::MISSION_CONSTANT_SPEED) {
        // ----------------------------------------------
        // ---------------- Defender mode ---------------
        // --------------- Drive innerline --------------
        // ----------------- ACC activate ---------------
        // ----------------------------------------------

        m_race_mode_first_callback = true;
        m_keep_position_mode_first_callback = true;
        m_non_overtaking_mode_first_callback = true;

        ///////////////////////////////
        // defender mode first callback
        ///////////////////////////////
        if (m_defender_mode_first_callback == true ||
            m_cur_planned_traj.trajectory_path.poses.empty()) {
          m_defender_mode_first_callback = false;

          // Switch to the defender line
          auto progreeNCTE_defenderline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_defenderline_path);

          // Merging frenet segment generation
          // Generate SINGLE frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_defenderline), // current_position_d
                      get<0>(progreeNCTE_defenderline), // current_position_s
                      0.0,                              // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_defenderline_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_defenderline) /
                                   allowable_maximum_vy),
                               2.0),
                      std::max(abs(get<1>(progreeNCTE_defenderline) /
                                   allowable_maximum_vy),
                               2.0) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {
            // Abnormal situation
            // publish empty & Estop trajectory
            // @DEBUG:
            RCLCPP_ERROR_ONCE(this->get_logger(),
                              "CRITICAL BUG HAS BEEN HIT.\n Frenet path "
                              "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          m_cur_planned_traj = stitchFrenetToPath(
              frenet_path_generation_result[0], m_defenderline_path);
          m_last_update_target_path_alias = "defender";
          m_reset_wpt_idx =
              getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                        frenet_path_generation_result[0]->points_y().back(),
                        m_cur_planned_traj.trajectory_path);
          m_reset_target_path_idx = RESET_PATH_TYPE::DEFENDER_LINE;
        } else {
          // Keep previous plan

          ///////////////////////////////////////////
          // Change the target path to the static wpt
          ///////////////////////////////////////////
          auto cur_idx_on_previous_path = getCurIdx(
              m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
              m_cur_planned_traj.trajectory_path);

          checkSwitchToStaticWPT(cur_idx_on_previous_path);
          // -----------------------------------------
        }

        auto cur_path_seg = getCertainLenOfPathSeg(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path, 100);

        auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
            m_ego_odom, m_cur_oppo_pred_result,
            m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
            1.0);

        // Publish cur_traj
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                 m_last_lat_planning_type, true);
        return;
      } else {
        ////////////////////////////
        // Overtaking is not allowed
        ////////////////////////////

        m_defender_mode_first_callback = true;
        m_race_mode_first_callback = true;
        m_keep_position_mode_first_callback = true;

        // if (m_non_overtaking_mode_first_callback == true ||
        //     m_cur_planned_traj.trajectory_path.poses.empty()) {
        if (m_cur_planned_traj.trajectory_path.poses.empty()) {
          m_non_overtaking_mode_first_callback = false;

          auto progreeNCTE_racingline =
              calcProgressNCTE(m_ego_odom.pose.pose, m_racingline_path);

          // Merging frenet segment generation
          // Generate single frenet path segment
          std::vector<std::shared_ptr<FrenetPath>>
              frenet_path_generation_result =
                  m_frenet_generator_ptr->calc_frenet_paths_multi_longi(
                      get<1>(progreeNCTE_racingline), // current_position_d
                      get<0>(progreeNCTE_racingline), // current_position_s
                      0.0,                            // current_velocity_d
                      std::max(m_ego_odom.twist.twist.linear.x,
                               MIN_SPEED_MPS), // current_velocity_s
                      0.0,                     // current_acceleration_d
                      get<4>(m_racingline_spline_data), // cubicSplineModel
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               2.0),
                      std::max(abs(get<1>(progreeNCTE_racingline) /
                                   allowable_maximum_vy),
                               2.0) +
                          0.01,
                      SAMPLING_TIME, 0.0, 0.0001, 0.1);

          if (frenet_path_generation_result.empty() ||
              frenet_path_generation_result[0]->points_x().empty()) {

            // Abnormal situation
            // publish empty & Estop trajectory
            RCLCPP_ERROR_ONCE(this->get_logger(),
                              "[MISSION_KEEP_POSITION] CRITICAL BUG HAS BEEN "
                              "HIT.\n Frenet path "
                              "generation result or point vector is empty");
            m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
            m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::ESTOP;
            nif_msgs::msg::DynamicTrajectory empty_traj;
            empty_traj.header.frame_id =
                nif::common::frame_id::localization::ODOM;
            publishPlannedTrajectory(empty_traj, m_last_long_planning_type,
                                     m_last_lat_planning_type, true);
            return;
          }

          auto stitched_path = stitchFrenetToPath(
              frenet_path_generation_result[0], m_racingline_path);

          m_cur_planned_traj = stitched_path;

          m_reset_wpt_idx =
              getCurIdx(frenet_path_generation_result[0]->points_x().back(),
                        frenet_path_generation_result[0]->points_y().back(),
                        m_cur_planned_traj.trajectory_path);

          m_last_update_target_path_alias = "raceline";
          m_reset_target_path_idx = RESET_PATH_TYPE::RACE_LINE;
        } else {
          // Keep previous plan

          ///////////////////////////////////////////
          // Change the target path to the static wpt
          ///////////////////////////////////////////
          auto cur_idx_on_previous_path = getCurIdx(
              m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
              m_cur_planned_traj.trajectory_path);

          checkSwitchToStaticWPT(cur_idx_on_previous_path);
          // -----------------------------------------
        }

        // -----------------------------------------

        // Keep previous plan and ACC trajectory generation
        auto cur_path_seg = getCertainLenOfPathSeg(
            m_ego_odom.pose.pose.position.x, m_ego_odom.pose.pose.position.y,
            m_cur_planned_traj.trajectory_path, 100);

        auto cur_traj = m_velocity_profiler_obj.velProfileForAcc(
            m_ego_odom, m_cur_oppo_pred_result,
            m_cur_det_global.obj_velocity_in_global.linear.x, cur_path_seg,
            1.0);

        // Publish cur_traj
        m_last_lat_planning_type = LATERAL_PLANNING_TYPE::KEEP;
        m_last_long_planning_type = LONGITUDINAL_PLANNING_TYPE::FOLLOW;
        publishPlannedTrajectory(cur_traj, m_last_long_planning_type,
                                 m_last_lat_planning_type, true);
        return;
      }
    }
  } else {
    // ESTOP
    nif::common::NodeStatusCode node_status = nif::common::NODE_INITIALIZED;
    this->setNodeStatus(node_status);
  }
}

bool DynamicPlannerNode::collisionCheckBTWtrajs(
    const nif_msgs::msg::DynamicTrajectory &ego_traj_,
    const nif_msgs::msg::DynamicTrajectory &oppo_traj_,
    const double collision_dist_boundary, const double collision_time_boundary,
    bool use_sat_) {
  // TODO: prediction and detection result health check

  if (oppo_traj_.trajectory_path.poses.empty()) {
    return false;
  }

  double collision_time_filetered = collision_time_boundary;
  if (collision_time_filetered < 1.0) {
    // std::cout << "Too risky. Set to 1 sec as default" << std::endl;
    collision_time_filetered = 1.0;
  }

  if (!use_sat_) { // if there is collision, return true

    bool is_collision = false;
    for (int ego_traj_idx = 0;
         ego_traj_idx < ego_traj_.trajectory_path.poses.size();
         ego_traj_idx++) {
      for (int oppo_traj_idx = 0;
           oppo_traj_idx < oppo_traj_.trajectory_path.poses.size();
           oppo_traj_idx++) {
        double dist = sqrt(
            pow((ego_traj_.trajectory_path.poses[ego_traj_idx].pose.position.x -
                 oppo_traj_.trajectory_path.poses[oppo_traj_idx]
                     .pose.position.x),
                2) +
            pow((ego_traj_.trajectory_path.poses[ego_traj_idx].pose.position.y -
                 oppo_traj_.trajectory_path.poses[oppo_traj_idx]
                     .pose.position.y),
                2));

        double time_diff =
            abs(ego_traj_.trajectory_timestamp_array[ego_traj_idx] -
                oppo_traj_.trajectory_timestamp_array[oppo_traj_idx]);

        if (dist < collision_dist_boundary &&
            time_diff < collision_time_filetered) {
          RCLCPP_INFO_ONCE(this->get_logger(),
                           "Collision True <Dist and Time>: <%lf, %lf> ", dist,
                           time_diff);
          is_collision = true;
          return is_collision;
        }
      }
    }
    return is_collision;
  } else {
    // USE SAT
    // SAT makes the program too slow.
    // TODO: need to check this
    return nif::planning::sat::separating_axis_intersect_traj_v2(
        ego_traj_, oppo_traj_, collision_time_filetered);
  }
}
