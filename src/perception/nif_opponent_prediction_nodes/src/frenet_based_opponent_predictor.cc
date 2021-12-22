#include "nif_opponent_prediction_nodes/frenet_based_opponent_predictor.h"

using namespace nif::perception;

FrenetBasedOpponentPredictor::FrenetBasedOpponentPredictor(
    const string& node_name)
  : Node(node_name),
    m_prediction_valid_flg(false),
    m_opponent_target_path_valid_flg(false),
    m_config_valid_flg(false),
    m_initialize_done_flg(false)

{
  this->declare_parameter("ref_line_file_path", "");
  this->declare_parameter("path_spline_interval", 1.0);
  this->declare_parameter("prediction_horizon_s", 4.0);
  this->declare_parameter("prediction_sampling_time", 0.2);
  this->declare_parameter("oppo_vel_bias_mps", 0.5);
  this->declare_parameter("m_defender_vel_mps", 15.0);

  // TODO
  m_opponent_status_topic_name = "/ghost/perception";
  m_ego_status_topic_name = "aw_localization/ekf/odom";
  m_defender_vel_topic_name = "/defender_vel";
  m_predicted_trajectory_topic_name = "/oppo/prediction";
  m_predicted_trajectory_vis_topic_name = "/oppo/vis/prediction";

  m_opponent_global_progress = 0.0;
  m_opponent_cte = 0.0;

  m_predicted_output_in_global.header.frame_id =
      common::frame_id::localization::ODOM;
  m_predicted_output_in_global_vis.header.frame_id =
      common::frame_id::localization::ODOM;
  m_predicted_output_in_local.header.frame_id =
      common::frame_id::localization::BASE_LINK;

  // TODO : Testing
  // NOTE : load center line as a reference
  m_centerline_ref_file_path =
      this->get_parameter("ref_line_file_path").as_string();
  m_config_path_spline_interval_m =
      this->get_parameter("path_spline_interval").as_double();
  m_config_prediction_horizon_s =
      this->get_parameter("prediction_horizon_s").as_double();
  m_config_prediction_sampling_time_s =
      this->get_parameter("prediction_sampling_time").as_double();
  m_config_oppo_vel_bias_mps =
      this->get_parameter("oppo_vel_bias_mps").as_double();
  m_defender_vel_default_mps = this->get_parameter("m_defender_vel_mps").as_double();

  if (m_centerline_ref_file_path == "") {
    // initialization failed
    throw std::runtime_error("Frenet based Opponent predictor : Initialization "
                             "failed (Empty path).");
  }

  // TODO : load config and assign
  // check items
  // spline interval and m_config_prediction_horizon_s can not be 0.0

  // Load opponent's target path based on predefined static file <csv> (THIS
  // ASSUMPTION IS DANGEROUS, THIS SHOULD BE IMPROVED)
  auto wpt_xy = loadCSVFile(m_centerline_ref_file_path);
  m_centerline_path_x = get<0>(wpt_xy);
  m_centerline_path_y = get<1>(wpt_xy);

  // Assign splined full target path in global coordinate (in nav_msg path)
  auto splined_result = m_frenet_generator_ptr->apply_cubic_spliner(
      m_centerline_path_x,
      m_centerline_path_y,
      m_config_path_spline_interval_m);

  m_splined_center_path_x = get<0>(splined_result);
  m_splined_center_path_y = get<1>(splined_result);
  m_splined_center_path_yaw = get<2>(splined_result);
  m_splined_center_path_curvature = get<3>(splined_result);
  m_centerline_splined_model = get<4>(splined_result);

  m_progress_vec = m_centerline_splined_model->points_s();

  // Initialize subscribers & publisher
  m_sub_opponent_status =
      this->create_subscription<common::msgs::PerceptionResultList>(
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
  m_pub_predicted_trajectory_vis = this->create_publisher<nav_msgs::msg::Path>(
      m_predicted_trajectory_vis_topic_name, common::constants::QOS_PLANNING);

  m_opponent_target_path_valid_flg = true;
  m_config_valid_flg = true;
  m_initialize_done_flg = true;

  this->parameters_callback_handle = this->add_on_set_parameters_callback(
      std::bind(&FrenetBasedOpponentPredictor::parametersCallback,
                this,
                std::placeholders::_1));
}

void FrenetBasedOpponentPredictor::defenderVelCallback(
    const std_msgs::msg::Float32::SharedPtr msg) {
  m_defender_vel_mps = msg->data +
      m_config_oppo_vel_bias_mps; // NOTE : should be mps / absolute vel
}

void FrenetBasedOpponentPredictor::opponentStatusCallback(
    const common::msgs::PerceptionResultList::SharedPtr msg) {
  if (msg->perception_list.size() > 0) 
  {

    auto& perception_el = msg->perception_list[0];
    m_opponent_status = perception_el;

    m_defender_vel_mps = perception_el.obj_velocity_in_local.linear.x +
        // m_ego_status.twist.twist.linear.x + // Only if tracking result is relative to ego.
        m_config_oppo_vel_bias_mps; // NOTE : should be mps / absolute vel

    m_defender_vel_mps = std::max(m_defender_vel_mps, 0.5);

    // Do prediction when the oppponent's status is callbacked
    this->predict();

    // TODO: publish predicted trajectory

    m_pub_predicted_trajectory->publish(m_predicted_output_in_global);
    m_pub_predicted_trajectory_vis->publish(m_predicted_output_in_global_vis);
  }
}

void FrenetBasedOpponentPredictor::calcOpponentProgress() {
  if (m_centerline_path_x.size() == 0) {
    // empty target path
    std::cout << "center path is empty.." << std::endl;
  }

  // TODO : opponent position (body to global)

  geometry_msgs::msg::PoseStamped ps_local;
  ps_local.pose = m_opponent_status.detection_result_3d.center;

  auto global_ps = nif::common::utils::coordination::getPtBodytoGlobal(
      m_ego_status, ps_local);

  nif_msgs::msg::Perception3D opponent_status_global;
  opponent_status_global = m_opponent_status;
  opponent_status_global.detection_result_3d.center = global_ps.pose;

  // calc closest index
  double min_dist = common::constants::numeric::INF;
  int opponent_index = 0;
  for (int i = 0; i < m_centerline_path_x.size(); i++) {
    double path_x = m_centerline_path_x[i];
    double path_y = m_centerline_path_y[i];

    double dist =
        sqrt(pow(opponent_status_global.detection_result_3d.center.position.x -
                     path_x,
                 2) +
             pow(opponent_status_global.detection_result_3d.center.position.y -
                     path_y,
                 2));
    if (min_dist > dist) {
      min_dist = dist;
      opponent_index = i;
    }
  }

  // m_opponent_global_progress =
  //     (opponent_index * m_config_path_spline_interval_m * 1.0);

  m_opponent_global_progress = m_progress_vec[opponent_index];

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
  auto progress = (pt_index * m_config_path_spline_interval_m * 1.0);
  return progress;
}

void FrenetBasedOpponentPredictor::predict() {
  // 1. Calculate the opponent's progress
  // NOTE : Based on current detection result, it calculate the opponent's
  // current position progress and cross-track error
  this->calcOpponentProgress();

  // 2. Generate minimum jerk frenet path which is parallel to the
  // centerline

  std::tuple<std::shared_ptr<FrenetPath>,
             std::vector<std::shared_ptr<FrenetPath>>>
      frenet_path_generation_result = m_frenet_generator_ptr->calc_frenet_paths(
          m_opponent_cte,                // current_position_d
          m_opponent_global_progress,    // current_position_s
          0.0,                           // current_velocity_d
          m_defender_vel_mps,            // current_velocity_s
          0.0,                           // current_acceleration_d
          m_centerline_splined_model,    // cubic_spliner_2D
          m_config_prediction_horizon_s, // Prediction horizon
          m_config_prediction_horizon_s +
              0.01, // Max max horizon (we want only one here)
          m_config_prediction_sampling_time_s, //
          m_opponent_cte,
          m_opponent_cte + 0.01,
          0.1);

  std::shared_ptr<FrenetPath>& predicted_frenet_path =
      std::get<0>(frenet_path_generation_result);

  //DynamicTrajectory initialize
  m_predicted_output_in_global.trajectory_path.poses.clear();
  m_predicted_output_in_global.trajectory_velocity.clear();
  m_predicted_output_in_global.trajectory_timestamp_array.clear();
  m_predicted_output_in_global.trajectory_global_progress.clear();

  m_predicted_output_in_global_vis.poses.clear();

  nav_msgs::msg::Path traj_global;

  if (!predicted_frenet_path->points_x().empty()) {
    traj_global.header.frame_id = common::frame_id::localization::ODOM;

    for (int i = 0; i < predicted_frenet_path->points_x().size(); i++) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = common::frame_id::localization::ODOM;
      std_msgs::msg::Header header;
      header.frame_id = common::frame_id::localization::ODOM;

      ps.pose.position.x = predicted_frenet_path->points_x()[i];
      ps.pose.position.y = predicted_frenet_path->points_y()[i];
      ps.pose.orientation = nif::common::utils::coordination::euler2quat(predicted_frenet_path->yaw()[i],
                                                                        0.0, 0.0);
      m_predicted_output_in_global.trajectory_timestamp_array.push_back(
          predicted_frenet_path->time()[i]);
      m_predicted_output_in_global.trajectory_global_progress.push_back(
          predicted_frenet_path->points_s()[i]);
      traj_global.poses.push_back(ps);
    }

    m_predicted_output_in_global.trajectory_path = traj_global;

    m_predicted_output_in_global.trajectory_type =
        nif_msgs::msg::DynamicTrajectory::TRAJECTORY_TYPE_PREDICTION;
    m_predicted_output_in_global_vis = traj_global;

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

rcl_interfaces::msg::SetParametersResult
FrenetBasedOpponentPredictor::parametersCallback(
    const std::vector<rclcpp::Parameter>& vector) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  for (const auto& param : vector) {
    if (param.get_name() == "path_spline_interval") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 10.0) {
          this->m_config_path_spline_interval_m = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "prediction_horizon_s") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 10.0) {
          this->m_config_prediction_horizon_s = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "prediction_sampling_time") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 10.0) {
          this->m_config_prediction_sampling_time_s = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "oppo_vel_bias_mps") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 10.0) {
          this->m_config_oppo_vel_bias_mps = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "m_defender_vel_mps") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 100.0) {
          this->m_defender_vel_default_mps = param.as_double();
          result.successful = true;
        }
      }
    }
  }

  return result;
}
