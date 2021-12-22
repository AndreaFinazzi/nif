#include <nif_dynamic_planning_nodes/velocity_profile/velocity_profiler.hpp>

velocity_profiler::velocity_profiler(std::string config_file_path_) {
  m_config_path = config_file_path_;

  auto success = parseConfig(m_config_path);
  checkConfig();
}

bool velocity_profiler::parseConfig(const std::string &config_file_path_) {
  YAML::Node config = YAML::LoadFile(config_file_path_);

  if (!config["velocity_profiling_param"]) {
    throw std::runtime_error(
        "velocity_profiling_param field not defined in config file.");
  }

  if (!config["adaptive_cruise_control_param"]) {
    throw std::runtime_error(
        "adaptive_cruise_control_param field not defined in config file.");
  }

  YAML::Node velocity_profiling_param = config["velocity_profiling_param"];
  YAML::Node adaptive_cruise_control_param =
      config["adaptive_cruise_control_param"];

  m_constraint_max_t =
      velocity_profiling_param["constraint_max_t"].as<double>();
  m_constraint_min_t =
      velocity_profiling_param["constraint_min_t"].as<double>();
  m_constraint_max_accel =
      velocity_profiling_param["constraint_max_accel"].as<double>();
  m_constraint_max_lat_accel =
      velocity_profiling_param["m_constraint_max_lat_accel"].as<double>();
  m_constraint_max_deccel =
      velocity_profiling_param["constraint_max_deccel"].as<double>();
  m_constraint_max_vel =
      velocity_profiling_param["constraint_max_vel"].as<double>();

  m_config_use_veh_model =
      velocity_profiling_param["config_use_veh_model"].as<bool>();
  m_config_use_acc_model =
      velocity_profiling_param["config_use_acc_model"].as<bool>();
  m_config_use_curvature_model =
      velocity_profiling_param["config_use_curvature_model"].as<bool>();

  m_config_dt = velocity_profiling_param["config_dt"].as<double>();

  m_acc_config_s0 = velocity_profiling_param["acc_config_s0"].as<double>();
  m_acc_config_s1 = velocity_profiling_param["acc_config_s1"].as<double>();
  m_acc_config_v_desired =
      velocity_profiling_param["acc_config_v_desired"].as<double>();
  m_acc_config_time_headway =
      velocity_profiling_param["acc_config_time_headway"].as<double>();
  m_acc_config_accel_max =
      velocity_profiling_param["acc_config_accel_max"].as<double>();
  m_acc_config_decel_desired =
      velocity_profiling_param["acc_config_decel_desired"].as<double>();
  m_acc_config_delta =
      velocity_profiling_param["acc_config_delta"].as<double>();
  m_acc_config_veh_l =
      velocity_profiling_param["acc_config_veh_l"].as<double>();

  if (m_constraint_max_t < m_constraint_min_t) {
    throw std::runtime_error("m_constraint_max_t can not be less than "
                             "m_constraint_min_t. Check config file.");
  }
  if (m_constraint_max_accel < 0.0) {
    throw std::runtime_error("m_constraint_max_accel can not be less than "
                             "zero. Check config file.");
  }
  if (m_constraint_max_lat_accel < 0.0) {
    throw std::runtime_error("m_constraint_max_lat_accel can not be less than "
                             "zero. Check config file.");
  }
  if (m_constraint_max_deccel > 0.0) {
    throw std::runtime_error("m_constraint_max_deccel can not be larger than "
                             "zero. Check config file.");
  }
  if (m_constraint_max_vel < 0.0) {
    throw std::runtime_error("m_constraint_max_vel can not be less than "
                             "zero. Check config file.");
  }

  if (m_config_use_veh_model == false && m_config_use_acc_model == false &&
      m_config_use_curvature_model == false) {
    throw std::runtime_error(
        "At least, one of three model should be set to true. "
        "Check config file.");
  }
}

void velocity_profiler::setConfigUseVehModel(bool &flg) {
  m_config_use_veh_model = flg;
}
void velocity_profiler::setConfigUseACCModel(bool &flg) {
  m_config_use_acc_model = flg;
}
void velocity_profiler::setConfigUseCurvatureMode(bool &flg) {
  m_config_use_curvature_model = flg;

  if (m_config_use_veh_model || m_config_use_acc_model ||
      m_config_use_curvature_model == false) {
    std::cout << "At least one of the velocity profiling method should be set "
                 "to true. Force set to True"
              << std::endl;
    m_config_use_curvature_model = true;
  }
}

bool velocity_profiler::checkConfig() {
  if (m_constraint_max_t < m_constraint_min_t) {
    return false;
  }
  if (m_constraint_max_accel < 0.0) {
    return false;
  }
  if (m_constraint_max_deccel > 0.0) {
    return false;
  }
  if (m_constraint_max_vel < 0.0) {
    return false;
  }
  return true;
}

bool velocity_profiler::setConstraintMaxT(double &value) {
  auto constraint_max_t_prev = m_constraint_max_t;
  m_constraint_max_t = value;
  auto check = checkConfig();
  if (check == false) {
    std::cout << "Envalid constraint. Keep the previous config." << std::endl;
    m_constraint_max_t = constraint_max_t_prev;
  }
} // [sec]
bool velocity_profiler::setConstraintMinT(double &value) {
  auto constraint_min_t_prev = m_constraint_min_t;
  m_constraint_min_t = value;
  auto check = checkConfig();
  if (check == false) {
    std::cout << "Envalid constraint. Keep the previous config." << std::endl;
    m_constraint_min_t = constraint_min_t_prev;
  }
} // [sec]
bool velocity_profiler::setConstraintMaxAccel(double &value) {
  auto constraint_max_accel_prev = m_constraint_max_accel;
  m_constraint_max_accel = value;
  auto check = checkConfig();
  if (check == false) {
    std::cout << "Envalid constraint. Keep the previous config." << std::endl;
    m_constraint_max_accel = constraint_max_accel_prev;
  }
} // [mpss]
bool velocity_profiler::setConstraintMaxDeccel(double &value) {
  auto constraint_max_deccel_prev = m_constraint_max_deccel;
  m_constraint_max_deccel = value;
  auto check = checkConfig();
  if (check == false) {
    std::cout << "Envalid constraint. Keep the previous config." << std::endl;
    m_constraint_max_deccel = constraint_max_deccel_prev;
  }
} // [mpss]
bool velocity_profiler::setConstraintMaxVel(double &value) {
  auto constraint_max_vel_prev = m_constraint_max_vel;
  m_constraint_max_vel = value;
  auto check = checkConfig();
  if (check == false) {
    std::cout << "Envalid constraint. Keep the previous config." << std::endl;
    m_constraint_max_vel = constraint_max_vel_prev;
  }
} // [mps]

nif_msgs::msg::DynamicTrajectory
velocity_profiler::velProfile(const nav_msgs::msg::Odometry &odom_,
                              const nav_msgs::msg::Path &target_path_,
                              const double &spline_interval_) {
  std::vector<double> target_path_x(target_path_.poses.size(), 0.0);
  std::vector<double> target_path_y(target_path_.poses.size(), 0.0);

  m_profiled_traj.trajectory_path.poses.clear();

  for (int i = 0; i < target_path_.poses.size(); i++) {
    target_path_x[i] = target_path_.poses[i].pose.position.x;
    target_path_y[i] = target_path_.poses[i].pose.position.y;
  }

  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
      new CubicSpliner2D(target_path_x, target_path_y));

  std::vector<double> cubic_spliner_x;
  std::vector<double> cubic_spliner_y;
  std::vector<double> cubic_spliner_yaw;
  std::vector<double> cubic_spliner_curvature;

  double point_s = 0.0;
  double point_s_end = cubic_spliner_2D->points_s().back();

  while (point_s < point_s_end) {
    std::tuple<double, double> position =
        cubic_spliner_2D->calculate_position(point_s);

    double point_x = std::get<0>(position);
    double point_y = std::get<1>(position);

    double yaw = cubic_spliner_2D->calculate_yaw(point_s);

    double curvature = cubic_spliner_2D->calculate_curvature(point_s);

    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = point_x;
    ps.pose.position.y = point_y;
    ps.pose.orientation =
        nif::common::utils::coordination::euler2quat(yaw, 0.0, 0.0);
    m_profiled_traj.trajectory_path.poses.push_back(ps);

    if (point_s == 0.0) {
      m_profiled_traj.trajectory_velocity.push_back(odom_.twist.twist.linear.x);
    } else {
      double curve_vel =
          std::min(m_constraint_max_vel / 3.6,
                   sqrt(abs(m_constraint_max_lat_accel) / abs(curvature)));
      double step_limited_vel =
          std::min(curve_vel, m_profiled_traj.trajectory_velocity.back() +
                                  (spline_interval_ /
                                   m_profiled_traj.trajectory_velocity.back()) *
                                      m_constraint_max_accel);
      m_profiled_traj.trajectory_velocity.push_back(step_limited_vel);
    }

    if (point_s == 0.0) {
      m_profiled_traj.trajectory_timestamp_array.push_back(0.0);
    } else {
      m_profiled_traj.trajectory_timestamp_array.push_back(
          m_profiled_traj.trajectory_timestamp_array.back() +
          (spline_interval_ /
           (m_profiled_traj.trajectory_velocity.back() + 1e-6)));
    }

    cubic_spliner_x.push_back(point_x);
    cubic_spliner_y.push_back(point_y);
    cubic_spliner_yaw.push_back(yaw);
    cubic_spliner_curvature.push_back(curvature);

    point_s += spline_interval_;
  }
  return m_profiled_traj;
}

nif_msgs::msg::DynamicTrajectory velocity_profiler::velProfileForAcc(
    const nav_msgs::msg::Odometry &odom_,
    const nif_msgs::msg::DynamicTrajectory &cipv_predicted_traj_,
    const double &cipv_vel_abs_, const nav_msgs::msg::Path &target_path_,
    const double &spline_interval_) {

  auto naive_gap =
      sqrt(pow(odom_.pose.pose.position.x -
                   cipv_predicted_traj_.trajectory_path.poses.front()
                       .pose.position.x,
               2) +
           pow(odom_.pose.pose.position.y -
                   cipv_predicted_traj_.trajectory_path.poses.front()
                       .pose.position.y,
               2));

  if (naive_gap > 120.0) {
    //   Dont care about the ACC
    velProfile(odom_, target_path_, spline_interval_);
  } else {
    // Care about the ACC in the trajectory planning
    std::vector<double> target_path_x(target_path_.poses.size(), 0.0);
    std::vector<double> target_path_y(target_path_.poses.size(), 0.0);

    m_profiled_traj.trajectory_path.poses.clear();

    for (int i = 0; i < target_path_.poses.size(); i++) {
      target_path_x[i] = target_path_.poses[i].pose.position.x;
      target_path_y[i] = target_path_.poses[i].pose.position.y;
    }

    std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
        new CubicSpliner2D(target_path_x, target_path_y));

    std::vector<double> cubic_spliner_x;
    std::vector<double> cubic_spliner_y;
    std::vector<double> cubic_spliner_yaw;
    std::vector<double> cubic_spliner_curvature;

    double point_s = 0.0;
    double point_s_end = cubic_spliner_2D->points_s().back();

    while (point_s < point_s_end) {
      std::tuple<double, double> position =
          cubic_spliner_2D->calculate_position(point_s);

      double point_x = std::get<0>(position);
      double point_y = std::get<1>(position);
      double yaw = cubic_spliner_2D->calculate_yaw(point_s);
      double curvature = cubic_spliner_2D->calculate_curvature(point_s);

      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = point_x;
      ps.pose.position.y = point_y;
      ps.pose.orientation =
          nif::common::utils::coordination::euler2quat(yaw, 0.0, 0.0);
      m_profiled_traj.trajectory_path.poses.push_back(ps);

      if (point_s == 0.0) {
        m_profiled_traj.trajectory_velocity.push_back(
            odom_.twist.twist.linear.x);
        m_profiled_traj.trajectory_timestamp_array.push_back(0.0);
      } else {
        double curve_vel =
            std::min(m_constraint_max_vel / 3.6,
                     sqrt(abs(m_constraint_max_lat_accel) / abs(curvature)));
        double step_limited_vel = std::min(
            curve_vel, m_profiled_traj.trajectory_velocity.back() +
                           (spline_interval_ /
                            m_profiled_traj.trajectory_velocity.back()) *
                               m_constraint_max_accel);

        auto naive_idm_desired_gap =
            m_acc_config_s0 +
            m_acc_config_s1 * sqrt(m_profiled_traj.trajectory_velocity.back() /
                                   m_acc_config_v_desired) +
            m_acc_config_time_headway *
                m_profiled_traj.trajectory_velocity.back() +
            m_profiled_traj.trajectory_velocity.back() *
                (m_profiled_traj.trajectory_velocity.back() - cipv_vel_abs_) /
                (2 * sqrt(m_constraint_max_accel *
                          (-1 * m_acc_config_decel_desired)));

        auto predictided_oppo_pose =
            cipv_predicted_traj_.trajectory_path.poses[closest(
                cipv_predicted_traj_.trajectory_timestamp_array,
                m_profiled_traj.trajectory_timestamp_array.back())];

        auto naive_cur_gap = std::max(
            sqrt(pow(predictided_oppo_pose.pose.position.x - point_x, 2) +
                 pow(predictided_oppo_pose.pose.position.y - point_y, 2)),
            1e-6);

        auto acc_desired_accel =
            m_constraint_max_accel *
            (1 -
             pow((m_profiled_traj.trajectory_velocity.back() /
                  m_acc_config_v_desired),
                 m_acc_config_delta) -
             pow((naive_idm_desired_gap / naive_cur_gap), 2));

        acc_desired_accel = std::clamp(
            acc_desired_accel, m_constraint_max_deccel, m_constraint_max_accel);

        auto acc_limited_vel = std::min(
            step_limited_vel, m_profiled_traj.trajectory_velocity.back() +
                                  (spline_interval_ /
                                   m_profiled_traj.trajectory_velocity.back()) *
                                      acc_desired_accel);

        m_profiled_traj.trajectory_velocity.push_back(acc_limited_vel);

        m_profiled_traj.trajectory_timestamp_array.push_back(
            m_profiled_traj.trajectory_timestamp_array.back() +
            (spline_interval_ /
             (m_profiled_traj.trajectory_velocity.back() + 1e-6)));
      }

      cubic_spliner_x.push_back(point_x);
      cubic_spliner_y.push_back(point_y);
      cubic_spliner_yaw.push_back(yaw);
      cubic_spliner_curvature.push_back(curvature);

      point_s += spline_interval_;
    }
    return m_profiled_traj;
  }
}