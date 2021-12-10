#include "nif_opponent_prediction_nodes/frenet_based_opponent_predictor.h"

using namespace nif::perception;

FrenetBasedOpponentPredictor::FrenetBasedOpponentPredictor(
    const string& ref_file_path_, const string& prediction_config_file_path_)
  : Node("opponent_predictor_node") {
  m_prediction_valid_flg = false;
  m_opponent_target_path_valid_flg = false;
  m_config_valid_flg = false;
  m_initialize_done_flg = false;

  double defualt_oppo_vel = 80;                  // mph
  m_defender_vel_mps = defualt_oppo_vel / 2.237; // mph2mps

  // TODO
  m_opponent_status_topic_name = "SetPropoerTopicName";
  m_ego_status_topic_name = "SetPropoerTopicName";

  m_opponent_global_progress = 0.0;
  m_opponent_cte = 0.0;

  // NOTE : load center line as a reference
  m_centerline_ref_file_path = ref_file_path_;
  // prediction configuration
  m_prediction_config_file_path = prediction_config_file_path_;

  m_predicted_output_in_global.header.frame_id =
      common::frame_id::localization::ODOM;
  m_predicted_output_in_global_vis.header.frame_id =
      common::frame_id::localization::ODOM;
  m_predicted_output_in_local.header.frame_id =
      common::frame_id::localization::BASE_LINK;

  if (m_centerline_ref_file_path == "" || m_prediction_config_file_path == "") {
    // initialization failed
    throw std::runtime_error("Frenet based Opponent predictor : Initialization "
                             "failed (Empty path).");
  }

  // TODO : load config and assign
  // check items
  // spline interval and m_config_prediction_horizon can not be 0.0

  // Load opponent's target path based on predefined static file <csv> (THIS
  // ASSUMPTION IS DANGEROUS, THIS SHOULD BE IMPROVED)
  auto wpt_xy = loadCSVFile(m_centerline_ref_file_path);
  m_centerline_path_x = get<0>(wpt_xy);
  m_centerline_path_y = get<1>(wpt_xy);

  // Assign full target path in global coordinate (in nav_msg path)
  // deprecated
  // m_opponent_target_path.header.frame_id =
  //     common::frame_id::localization::ODOM;
  // for (int i = 0; i < m_centerline_path_x.size(); i++) {
  //   geometry_msgs::msg::PoseStamped pt;
  //   pt.header.frame_id = common::frame_id::localization::ODOM;
  //   pt.pose.position.x = m_centerline_path_x[i];
  //   pt.pose.position.y = m_centerline_path_y[i];
  //   m_opponent_target_path.poses.push_back(pt);
  // }

  // Assign splined full target path in global coordinate (in nav_msg path)
  auto splined_result = m_frenet_generator_ptr->apply_cubic_spliner(
      m_centerline_path_x, m_centerline_path_y, m_config_path_spline_interval);

  m_splined_center_path_x = get<0>(splined_result);
  m_splined_center_path_y = get<1>(splined_result);
  m_splined_center_path_yaw = get<2>(splined_result);
  m_splined_center_path_curvature = get<3>(splined_result);
  m_centerline_splined_model = get<4>(splined_result);

  // deprecated
  // m_opponent_splined_target_path.header.frame_id =
  //     common::frame_id::localization::ODOM;
  // for (int i = 0; i < m_splined_center_path_x.size(); i++) {
  //   geometry_msgs::msg::PoseStamped pt;
  //   pt.header.frame_id = common::frame_id::localization::ODOM;
  //   pt.pose.position.x = m_splined_center_path_x[i];
  //   pt.pose.position.y = m_splined_center_path_y[i];
  //   pt.pose.orientation =
  //       amathutils::getQuaternionFromYaw(m_splined_center_path_yaw[i]);
  //   m_opponent_splined_target_path.poses.push_back(pt);
  // }

  // Initialize subscribers & publisher
  m_sub_opponent_status =
      this->create_subscription<common::msgs::PerceptionResult>(
          m_opponent_status_topic_name,
          common::constants::QOS_PLANNING,
          std::bind(&FrenetBasedOpponentPredictor::opponentStatusCallback,
                    this,
                    std::placeholders::_1));
  m_sub_ego_status = this->create_subscription<nav_msgs::msg::Odometry>(
      m_ego_status_topic_name,
      common::constants::QOS_EGO_ODOMETRY,
      std::bind(&FrenetBasedOpponentPredictor::egoStatusCallback,
                this,
                std::placeholders::_1));
  m_sub_defender_vel = this->create_subscription<std_msgs::msg::Float32>(
      m_defender_vel_topic_name,
      common::constants::QOS_PLANNING,
      std::bind(&FrenetBasedOpponentPredictor::defenderVelCallback,
                this,
                std::placeholders::_1));
  m_pub_predicted_trajectory =
      this->create_publisher<common::msgs::NIF_Trajectory>(
          m_predicted_trajectory_topic_name, common::constants::QOS_PLANNING);

  m_opponent_target_path_valid_flg = true;
  m_config_valid_flg = true;
  m_initialize_done_flg = true;
}

void FrenetBasedOpponentPredictor::defenderVelCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_defender_vel_mps = msg->data +
      m_config_oppo_vel_bias_mps; // NOTE : should be mps / absolute vel
}

void FrenetBasedOpponentPredictor::opponentStatusCallback(
    const common::msgs::PerceptionResult::SharedPtr msg) {
  m_opponent_status = *msg;

  // Do prediction when the oppponent's status is callbacked
  this->predict();

  // TODO: publish predicted trajectory
}

void FrenetBasedOpponentPredictor::calcOpponentProgress() {
  if (m_centerline_path_x.size() == 0) {
    // empty target path
    std::cout << "center path is empty.." << std::endl;
  }

  // calc closest index
  double min_dist = common::constants::numeric::INF;
  int opponent_index = 0;
  for (int i = 0; i < m_centerline_path_x.size(); i++) {
    double path_x = m_centerline_path_x[i];
    double path_y = m_centerline_path_y[i];

    double dist = sqrt(
        pow(m_opponent_status.detection_result_3d.center.position.x - path_x,
            2) +
        pow(m_opponent_status.detection_result_3d.center.position.y - path_y,
            2));
    if (min_dist > dist) {
      min_dist = dist;
      opponent_index = i;
    }
  }
  m_opponent_global_progress =
      (opponent_index * m_config_path_spline_interval * 1.0);

  // positive : opponent is in the left side of the centerline
  // negative : opponent is in the right side of the centerline
  // Sign calculation
  int sign = 1;
  // global to local transform
  auto local_pt = nif::common::utils::coordination::getPtGlobaltoBody(
      m_ego_status,
      m_centerline_path_x[opponent_index],
      m_centerline_path_y[opponent_index]);

  auto cross_product = local_pt.pose.position.x *
          m_opponent_status.detection_result_3d.center.position.y -
      local_pt.pose.position.y *
          m_opponent_status.detection_result_3d.center.position.x;
  if (cross_product < 0) {
    sign = -1;
  } else {
    sign = 1;
  }
  m_opponent_cte = min_dist * sign;
}

double FrenetBasedOpponentPredictor::calcProgress(
    geometry_msgs::msg::PoseStamped& pt_) {
  if (m_centerline_path_x.size() == 0) {
    // empty target path
    std::cout << "center path is empty.." << std::endl;
  }

  // calc closest index
  double min_dist = common::constants::numeric::INF;
  int pt_index = 0;
  for (int i = 0; i < m_centerline_path_x.size(); i++) {
    double path_x = m_centerline_path_x[i];
    double path_y = m_centerline_path_y[i];

    double dist = sqrt(pow(pt_.pose.position.x - path_x, 2) +
                       pow(pt_.pose.position.y - path_y, 2));
    if (min_dist > dist) {
      min_dist = dist;
      pt_index = i;
    }
  }
  auto progress = (pt_index * m_config_path_spline_interval * 1.0);
  return progress;
}

void FrenetBasedOpponentPredictor::predict() {
  // TODO : prediction algorithm
  // 1. Calculate the opponent's progress
  // NOTE : Based on current detection result, it calculate the opponent's
  // current position progress.
  this->calcOpponentProgress();

  // 2. Generate minimum jerk frenet path to the reference path
  double left_margin_tmp = m_opponent_cte;
  double right_margin_tmp = left_margin_tmp + 0.0001;
  double width_tmp = 0.1;
  std::tuple<std::shared_ptr<FrenetPath>,
             std::vector<std::shared_ptr<FrenetPath>>>
      frenet_path_generation_result = m_frenet_generator_ptr->calc_frenet_paths(
          m_opponent_cte,             // current_position_d
          m_opponent_global_progress, // current_position_s
          0.0,                        // current_velocity_d
          m_defender_vel_mps,         // current_velocity_s
          0.0,                        // current_acceleration_d
          m_centerline_splined_model, // cubic_spliner_2D
          m_config_prediction_horizon,
          m_config_prediction_horizon + 0.01,
          m_config_prediction_sampling_time,
          left_margin_tmp,
          right_margin_tmp,
          width_tmp);

  std::shared_ptr<FrenetPath>& predicted_frenet_path =
      std::get<0>(frenet_path_generation_result);

  m_predicted_output_in_global.trajectory_pts.clear();
  m_predicted_output_in_global_vis.poses.clear();

  if (!predicted_frenet_path->points_x().empty()) {
    for (int i = 0; i < predicted_frenet_path->points_x().size(); i++) {
      nif_msgs::msg::TrajectoryPoint tp;
      geometry_msgs::msg::Pose ps;
      std_msgs::msg::Header header;
      header.frame_id = common::frame_id::localization::ODOM;
      ps.position.x = predicted_frenet_path->points_x()[i];
      ps.position.y = predicted_frenet_path->points_y()[i];

      tp.pose = ps;
      tp.timestep = predicted_frenet_path->time()[i];

      if (ps.position.x >= predicted_frenet_path->points_x()[0]) {
        m_predicted_output_in_global.trajectory_pts.push_back(tp);
        geometry_msgs::msg::PoseStamped p_st;
        p_st.header = header;
        p_st.pose = ps;
        m_predicted_output_in_global_vis.poses.push_back(p_st);
      }
    }
  } else {
    std::cout << "predicted frenet path length is zero" << std::endl;
  }
}

void FrenetBasedOpponentPredictor::egoStatusCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  m_ego_status = *msg;
}

tuple<vector<double>, vector<double>>
FrenetBasedOpponentPredictor::loadCSVFile(const string wpt_file_path_) {
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