//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_dynamic_planning_nodes/dynamic_planning_node_v3.h"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <stdlib.h>

using namespace nif::planning;
using namespace std;

DynamicPlannerNodeV3::DynamicPlannerNodeV3(const std::string &node_name_)
    : IBaseNode(node_name_, common::NodeType::PLANNING) {
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
  this->declare_parameter("maps_path_root", "");
  this->declare_parameter("vis_flg", true);

  this->get_parameter("maps_path_root", this->m_map_root_path);
  this->m_map_root_path.append("/");
  m_planning_config_file_path =
      this->get_parameter("planning_config_file_path").as_string();
  m_vis_flg = this->get_parameter("vis_flg").as_bool();

  if (m_planning_config_file_path.empty())
    throw std::runtime_error(
        "Parameter m_planning_config_file_path not declared, or empty.");

  // Load param
  loadConfig(m_planning_config_file_path);
  m_config_load_success = true;
  m_det_callback_first_run = true;
  m_oppo_pred_callback_first_run = true;
  m_timer_callback_first_run = true;

  // Init output trajectories
  initOutputTrajectory();

  // m_racingline_file_path =
  //   this->m_map_root_path.append(m_racingline_file_path);

  m_racingline_file_path = this->m_map_root_path + m_racingline_file_path;

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

  m_racingline_dtraj.header = m_racingline_path.header;
  m_racingline_dtraj.trajectory_path = m_racingline_path;

  std::cout << "[DYNAMICPLANNER] raceline loaded..." << std::endl;
  std::cout << "[DYNAMICPLANNER] Loading path candidates..." << std::endl;

  // Init spliner & spline modeling for every overtaking path candidates
  for (int candidate_idx = 0; candidate_idx < m_num_overtaking_candidates;
       candidate_idx++) {
    // m_overtaking_candidates_file_path_vec[candidate_idx] =
    //   this->m_map_root_path.append(m_overtaking_candidates_file_path_vec[candidate_idx]);

    m_overtaking_candidates_file_path_vec[candidate_idx] =
        this->m_map_root_path +
        m_overtaking_candidates_file_path_vec[candidate_idx];

    std::cout << "[DYNAMICPLANNER] Loading "
              << m_overtaking_candidates_alias_vec[candidate_idx] << std::endl;

    auto wpt_xy =
        loadCSVfile(m_overtaking_candidates_file_path_vec[candidate_idx]);

    auto path_x_vec = get<0>(wpt_xy);
    auto path_y_vec = get<1>(wpt_xy);
    auto splined_result = m_frenet_generator_ptr->apply_cubic_spliner(
        path_x_vec, path_y_vec, m_config_spline_interval);

    m_overtaking_candidates_spline_data_vec.push_back(splined_result);
    m_overtaking_candidates_spline_model_vec.push_back(get<4>(splined_result));

    auto pc =
        genPointCloudFromVec(get<0>(splined_result), get<1>(splined_result));
    m_overtaking_candidates_path_pc_vec.push_back(pc);

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(pc);
    m_overtaking_candidates_path_kdtree_vec.push_back(kdtree);

    auto candidate_path = xyyawVec2Path(
        get<0>(splined_result), get<1>(splined_result), get<2>(splined_result));

    m_overtaking_candidates_path_vec.push_back(candidate_path);

    nif_msgs::msg::DynamicTrajectory tmp;
    tmp.header = candidate_path.header;
    tmp.trajectory_path = candidate_path;
    m_overkaing_candidates_dtraj_vec.push_back(tmp);
  }

  std::cout << "[DYNAMICPLANNER] Loaded all the pathes" << std::endl;

  // INITIALIZE SUBSCRIBERS & PUBLISHER
  m_det_sub = this->create_subscription<nif_msgs::msg::Perception3D>(
      "tracking_output_topic_name", common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNodeV3::detectionResultCallback, this,
                std::placeholders::_1));

  m_oppo_pred_sub = this->create_subscription<nif_msgs::msg::DynamicTrajectory>(
      "prediction_output_topic_name", common::constants::QOS_PLANNING,
      std::bind(&DynamicPlannerNodeV3::predictionResultCallback, this,
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
  m_ego_traj_global_vis_debug_pub2 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug2", common::constants::QOS_PLANNING);
  m_ego_traj_global_vis_debug_pub3 =
      this->create_publisher<nav_msgs::msg::Path>(
          "planning/debug3", common::constants::QOS_PLANNING);

  m_planner_timer = this->create_wall_timer(
      20ms, std::bind(&DynamicPlannerNodeV3::timer_callback, this)); // 50 hz

  std::cout << "[DYNAMICPLANNER] Initialization done." << std::endl;
}

void DynamicPlannerNodeV3::loadConfig(
    const std::string &planning_config_file_) {
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

  // path_candidates_param
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

  if (m_config_overlap_checking_dist_bound <=
      nif::common::vehicle_param::VEH_WHEEL_BASE) {
    throw std::runtime_error(
        "m_config_overlap_checking_dist_bound can not be "
        "less than Vehicle wheel base(4.921m). Check config file.");
  }

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
DynamicPlannerNodeV3::loadCSVfile(const std::string &wpt_file_path_) {
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

void DynamicPlannerNodeV3::detectionResultCallback(
    const nif_msgs::msg::Perception3D::SharedPtr msg) {
  if (m_det_callback_first_run) {
    m_det_callback_first_run = false;
  } else {
    m_prev_det = m_cur_det;
  }

  m_cur_det = *msg;
}

void DynamicPlannerNodeV3::mapTrackBodyCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  m_maptrack_body = *msg;
}

void DynamicPlannerNodeV3::mapTrackGlobalCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  m_maptrack_global = *msg;
}

void DynamicPlannerNodeV3::predictionResultCallback(
    const nif_msgs::msg::DynamicTrajectory::SharedPtr msg) {
  if (m_oppo_pred_callback_first_run) {
    m_cur_oppo_pred_result = *msg;
    m_oppo_pred_callback_first_run = false;
  } else {
    m_prev_oppo_pred_result = m_cur_oppo_pred_result;
    m_cur_oppo_pred_result = *msg;
  }
}

void DynamicPlannerNodeV3 ::timer_callback() {
  /*
  Main flow of this approach is as follows:

        0. Check whether current trajectory has a collision within the certain
        time window.
            1. If there is no collision, follow the current planned trajectory
            as fast as we can.
            2. Else, generate the ACC trajectory and store this.

        1. Based on the opponent's predicted trajectory, find the side-by-side
        position at certain time.

        2. At that time, ego vehicle's desired velocity is assigned based on the
        remaind distance upto end of the passing zone. (+ margin) --> Also
        desired acceleration can be assigned but let's start from the zero
        desired acceleration as the moment.

        3. Given desired position and velocity(w acceleration), do planning
        using quintic_polynomials model.

        4. Check whether the planned trajectory collides with the opponent's
        predicted trajectory using distance and time terms.
            1. If no collision, check whether the planned trajectory obeys the
            pre-defined constraints with respective to acceleration and
            jerkness.
                1. If yes, execute the overtaking trajectory. DONE
                2. If no, execute the acc trajectory. DONE
            2. It there is collision, execute the acc trajectory. DONE
  */

  // update ego odometry
  m_ego_odom = this->getEgoOdometry();

  //  Check wheter we are close enough to the racing line
  bool is_close_racingline =
      (calcCTE(m_ego_odom.pose.pose, m_racineline_path_kdtree,
               m_racingline_path_pc) < m_config_merge_allow_dist)
          ? true
          : false;

  if (is_close_racingline) {
    // collision check

    auto fp_race = getFrenetToRacingLine();
    auto collision_raceline = collisionCheckBTWtrajsNFrenet(
        fp_race, m_cur_oppo_pred_result, m_config_overlap_checking_dist_bound,
        m_config_overlap_checking_time_bound);

    if (!collision_raceline) {
      // change the defualt path to the racing line
      m_cur_planned_traj = m_racingline_dtraj;
      // TODO: velocity profiling with limited length
      // HERE
      // ALSO AT SOME POINT, FORCE UPDATE IS NEEDED FOR SAFETY
      publishPlannedTrajectory(m_vis_flg);
      return;
    }
  }

  // step -1
  m_ego_cur_idx_in_planned_traj = calcCurIdxFromDynamicTraj(m_cur_planned_traj);

  // step 0
  // Velocity profiling (do with the curvature based as a start point)
  auto ego_traj = m_frenet_generator_ptr->convert_path_to_traj_curv(
      m_cur_planned_traj.trajectory_path, m_ego_cur_idx_in_planned_traj,
      m_config_max_accel, m_config_spline_interval);
}

void DynamicPlannerNodeV3 ::timer_callback_debug() {
  vector<std::shared_ptr<FrenetPath>> frenet_vec;

  // m_ego_odom = this->getEgoOdometry();
  m_ego_odom.pose.pose.position.x = -52.619985445315336;
  m_ego_odom.pose.pose.position.y = 56.95423448172532;

  // -48.04168226740312, 56.954560989081656
  // -52.619985445315336, 56.95423448172532
  // -73.21128471223794,38.746032695691326
  // -51.1039050935139,58.257967783360655

  for (int path_candidate_idx = 0;
       path_candidate_idx < m_overtaking_candidates_path_vec.size();
       path_candidate_idx++) {
    // step 1.1 : Generate the frenet candidates to all wpt
    // auto progressNcte = calcProgressNCTE(
    //     m_ego_odom.pose.pose,
    //     m_overtaking_candidates_path_kdtree_vec[path_candidate_idx],
    //     m_overtaking_candidates_path_pc_vec[path_candidate_idx]);

    auto progressNcte =
        calcProgressNCTE(m_ego_odom.pose.pose,
                         m_overtaking_candidates_path_vec[path_candidate_idx]);

    std::cout << " I  : " << path_candidate_idx << std::endl;
    std::cout << "Crosstrack error : " << get<1>(progressNcte) << std::endl;
    std::cout << "Progress : " << get<0>(progressNcte) << std::endl;

    std::tuple<std::shared_ptr<FrenetPath>,
               std::vector<std::shared_ptr<FrenetPath>>>
        frenet_path_generation_result =
            m_frenet_generator_ptr->calc_frenet_paths(
                get<1>(progressNcte), // current_position_d
                get<0>(progressNcte), // current_position_s
                0.0,                  // current_velocity_d
                std::max(m_ego_odom.twist.twist.linear.x,
                         5.0), // current_velocity_s
                0.0,           // current_acceleration_d
                m_overtaking_candidates_spline_model_vec
                    [path_candidate_idx], // cubic_spliner_2D
                m_config_planning_horizon, m_config_planning_horizon + 0.01,
                m_config_planning_dt, 0.0, 0.0001, 0.1);

    std::shared_ptr<FrenetPath> frenet_candidate =
        std::get<0>(frenet_path_generation_result);

    if (path_candidate_idx == 0) {
      nav_msgs::msg::Path debug_frenet_seg_path;
      debug_frenet_seg_path.header.frame_id = "odom";

      for (int i = 0; i < frenet_candidate->points_x().size(); i++) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "odom";
        ps.pose.position.x = frenet_candidate->points_x()[i];
        ps.pose.position.y = frenet_candidate->points_y()[i];
        debug_frenet_seg_path.poses.push_back(ps);
      }

      m_ego_traj_global_vis_debug_pub1->publish(debug_frenet_seg_path);
    }
    if (path_candidate_idx == 1) {
      nav_msgs::msg::Path debug_frenet_seg_path;
      debug_frenet_seg_path.header.frame_id = "odom";

      for (int i = 0; i < frenet_candidate->points_x().size(); i++) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "odom";
        ps.pose.position.x = frenet_candidate->points_x()[i];
        ps.pose.position.y = frenet_candidate->points_y()[i];
        debug_frenet_seg_path.poses.push_back(ps);
      }

      m_ego_traj_global_vis_debug_pub2->publish(debug_frenet_seg_path);
    }
    if (path_candidate_idx == 2) {
      nav_msgs::msg::Path debug_frenet_seg_path;
      debug_frenet_seg_path.header.frame_id = "odom";

      for (int i = 0; i < frenet_candidate->points_x().size(); i++) {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = "odom";
        ps.pose.position.x = frenet_candidate->points_x()[i];
        ps.pose.position.y = frenet_candidate->points_y()[i];
        debug_frenet_seg_path.poses.push_back(ps);
      }

      m_ego_traj_global_vis_debug_pub3->publish(debug_frenet_seg_path);
    }
  }
}

void DynamicPlannerNodeV3::publishEmptyTrajectory() {
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

void DynamicPlannerNodeV3::publishPlannedTrajectory(bool vis_flg_) {
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

void DynamicPlannerNodeV3::publishTrajectory() {
  // m_cur_ego_planned_result_body.header.stamp = this->now();
  // m_cur_ego_planned_result_global.header.stamp = this->now();

  // m_cur_ego_planned_result_body.trajectory_type =
  //     nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;
  // m_cur_ego_planned_result_global.trajectory_type =
  //     nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PLANNING;

  // if (m_cur_decision == PLANNING_DECISION_TYPE::STRAIGHT) {
  //   m_cur_ego_planned_result_body.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_STRAIGHT;
  //   m_cur_ego_planned_result_global.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_STRAIGHT;
  // } else if (m_cur_decision == PLANNING_DECISION_TYPE::FOLLOW) {
  //   m_cur_ego_planned_result_body.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_FOLLOW;
  //   m_cur_ego_planned_result_global.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_FOLLOW;
  // } else if (m_cur_decision == PLANNING_DECISION_TYPE::RIGHT) {
  //   m_cur_ego_planned_result_body.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_RIGHT;
  //   m_cur_ego_planned_result_global.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_RIGHT;
  // } else if (m_cur_decision == PLANNING_DECISION_TYPE::LEFT) {
  //   m_cur_ego_planned_result_body.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_LEFT;
  //   m_cur_ego_planned_result_global.planning_decision_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_DECISION_TYPE_LEFT;
  // } else {
  //   bool collisionCheckBTWtrajs(
  //       const nif_msgs::msg::DynamicTrajectory& ego_traj_,
  //       const nif_msgs::msg::DynamicTrajectory& oppo_traj_,
  //       const double collision_dist_boundary,
  //       const double collision_time_boundary); // if there is collision,
  //       return
  //                                              // true.PE::DRIVING) {
  //   m_cur_ego_planned_result_body.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_DRIVING;
  //   m_cur_ego_planned_result_global.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_DRIVING;
  // }
  // else if (m_cur_overtaking_action == PLANNING_ACTION_TYPE::START_OVERTAKING)
  // {
  //   m_cur_ego_planned_result_body.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
  //   m_cur_ego_planned_result_global.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
  // }
  // else if (m_cur_overtaking_action == PLANNING_ACTION_TYPE::SIDE_BY_SIDE) {
  //   m_cur_ego_planned_result_body.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_SIDE_BY_SIDE;
  //   m_cur_ego_planned_result_global.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_SIDE_BY_SIDE;
  // }
  // else if (m_cur_overtaking_action ==
  // PLANNING_ACTION_TYPE::FINISH_OVERTAKING) {
  //   m_cur_ego_planned_result_body.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
  //   m_cur_ego_planned_result_global.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_START_OVERTAKING;
  // }
  // else {
  //   // defualt : ABORT_OVERTAKING
  //   m_cur_ego_planned_result_body.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_ABORT_OVERTAKING;
  //   m_cur_ego_planned_result_global.planning_action_type =
  //       nif_msgs::msg::DynamicTrajectory::PLANNING_ACTION_ABORT_OVERTAKING;
  // }

  // m_ego_traj_body_pub->publish(m_cur_ego_planned_result_body);
  // m_ego_traj_global_pub->publish(m_cur_ego_planned_result_global);
}

void DynamicPlannerNodeV3::initOutputTrajectory() {
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

double DynamicPlannerNodeV3::getProgress(
    const geometry_msgs::msg::Pose &pt_global_,
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

double DynamicPlannerNodeV3::getProgress(
    const double &pt_x_, const double &pt_y_,
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
DynamicPlannerNodeV3::getCurIdx(const double &pt_x_, const double &pt_y_,
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

double
DynamicPlannerNodeV3::calcCTE(const geometry_msgs::msg::Pose &pt_global_,
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
DynamicPlannerNodeV3::genPointCloudFromVec(vector<double> &x_,
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

double DynamicPlannerNodeV3::calcProgressDiff(
    const geometry_msgs::msg::Pose &ego_pt_global_,
    const geometry_msgs::msg::Pose &target_pt_global_,
    pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
  auto ego_progress = getProgress(ego_pt_global_, target_tree_);
  auto target_progress = getProgress(target_pt_global_, target_tree_);

  // TODO : progress wrapping is needed!!!!!!!!!!!!!!!!!!!
  return ego_progress - target_progress;
}

nav_msgs::msg::Path
DynamicPlannerNodeV3::xyyawVec2Path(std::vector<double> &x_,
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

tuple<double, double> DynamicPlannerNodeV3::calcProgressNCTE(
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

tuple<double, double> DynamicPlannerNodeV3::calcProgressNCTE(
    const geometry_msgs::msg::Pose &pt_global_,
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

std::shared_ptr<FrenetPath> DynamicPlannerNodeV3::getFrenetToRacingLine() {
  // Generate trajectory segment from current odom to racing line.
  auto progressNcte = calcProgressNCTE(
      m_ego_odom.pose.pose, m_racineline_path_kdtree, m_racingline_path_pc);

  std::tuple<std::shared_ptr<FrenetPath>,
             std::vector<std::shared_ptr<FrenetPath>>>
      frenet_path_generation_result = m_frenet_generator_ptr->calc_frenet_paths(
          get<1>(progressNcte),             // current_position_d
          get<0>(progressNcte),             // current_position_s
          0.0,                              // current_velocity_d
          m_ego_odom.twist.twist.linear.x,  // current_velocity_s
          0.0,                              // current_acceleration_d
          get<4>(m_racingline_spline_data), // cubic_spliner_2D
          m_config_planning_horizon, m_config_planning_horizon + 0.01,
          m_config_planning_dt, 0.0, 0.0001, 0.1);

  //   std::shared_ptr<FrenetPath>& predicted_frenet_path =
  //       std::get<0>(frenet_path_generation_result);
  return std::get<0>(frenet_path_generation_result);
}

double DynamicPlannerNodeV3::calcProgressDiff(
    const nif_msgs::msg::DynamicTrajectory &ego_traj_,
    const nif_msgs::msg::DynamicTrajectory &oppo_traj_,
    pcl::KdTreeFLANN<pcl::PointXY> &target_tree_) {
  // Checking items
  // 1. Collision
  // 2. At the specific
}

int DynamicPlannerNodeV3::calcCurIdxFromDynamicTraj(
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

bool DynamicPlannerNodeV3::collisionCheckBTWtrajs(
    const nif_msgs::msg::DynamicTrajectory &ego_traj_,
    const nif_msgs::msg::DynamicTrajectory &oppo_traj_,
    const double collision_dist_boundary,
    const double collision_time_boundary) {
  // if there is collision, return true

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

bool DynamicPlannerNodeV3::collisionCheckBTWtrajsNFrenet(
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

nif_msgs::msg::DynamicTrajectory DynamicPlannerNodeV3::stitchFrenetToPath(
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