#include "nif_control_joint_lqr_nodes/control_lqr_node.h"
#include <nif_common/vehicle_model.h>

using nif::control::ControlLQRNode;

ControlLQRNode::ControlLQRNode(const std::string &node_name)
    : IControllerNode(node_name) {
  control_cmd = std::make_shared<nif::common::msgs::ControlCmd>();

  m_camber_manager_ptr = std::make_shared<CamberCompensator>(
      nif::control::CAMBERCOMPESATORMODE::FIRST_ORDER, true);

  // Debug Publishers
  lqr_command_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "control_joint_lqr/tracking_valid", nif::common::constants::QOS_DEFAULT);
  lqr_valid_conditions_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(
          "control_joint_lqr/valid_conditions",
          nif::common::constants::QOS_DEFAULT);
  lqr_steering_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/lqr_command", nif::common::constants::QOS_DEFAULT);
  lqr_accel_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/accel_command", nif::common::constants::QOS_DEFAULT);
  track_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/track_distance", nif::common::constants::QOS_DEFAULT);
  lqr_tracking_idx_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "control_joint_lqr/track_idx", nif::common::constants::QOS_DEFAULT);
  lqr_tracking_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "control_joint_lqr/track_point", nif::common::constants::QOS_DEFAULT);
  lqr_error_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "control_joint_lqr/lqr_error", nif::common::constants::QOS_DEFAULT);
  lqr_desired_velocity_mps_pub_ =
      this->create_publisher<std_msgs::msg::Float32>(
          "control_joint_lqr/desired_velocity_mps",
          nif::common::constants::QOS_DEFAULT);
  // Subscribers
  velocity_sub_ =
      this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
          "/raptor_dbw_interface/wheel_speed_report",
          nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&ControlLQRNode::velocityCallback, this,
                    std::placeholders::_1));

  direct_desired_velocity_sub =
      this->create_subscription<std_msgs::msg::Float32>(
          "velocity_planner/des_vel", nif::common::constants::QOS_CONTROL_CMD,
          std::bind(&ControlLQRNode::directDesiredVelocityCallback, this,
                    std::placeholders::_1));

  // Disable the ACC function
  //   acc_sub_ = this->create_subscription<std_msgs::msg::Float32>(
  //       "control/acc/accel_cmd", nif::common::constants::QOS_CONTROL_CMD,
  //       std::bind(&ControlLQRNode::accCMDCallback, this,
  //       std::placeholders::_1));

  this->declare_parameter("lqr_config_file", "");
  // Automatically boot with lat_autonomy_enabled
  //  this->declare_parameter("lat_autonomy_enabled", false);
  // Max Steering Angle in Degrees
  this->declare_parameter("max_steering_angle_deg", 20.0);
  // convert from degress to steering units (should be 1 - 1 ?)
  this->declare_parameter("steering_units_multiplier", 1.0);
  // Minimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_min_dist_m", 3.0);
  // Maximimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_max_dist_m", 8.);
  // pure_pursuit lookahead distance 1st velocity theshold (65 kph)
  this->declare_parameter("pure_pursuit_1st_vel_ms", 25.0);
  // Maximimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_max_max_dist_m", 13.0);
  // Factor to increase the pure pursuit tracking distance as a function of
  // speed (m/s)
  this->declare_parameter("pure_pursuit_k_vel_m_ms", 0.5);
  // Use tire speed instead of gps velocity estimate
  this->declare_parameter("use_tire_velocity", true);
  // Safety timeouts for odometry and the path (set negative to ignore)
  this->declare_parameter("odometry_timeout_sec", 0.1);
  this->declare_parameter("path_timeout_sec", 0.5);
  // Limit the max change in the steering signal over time
  this->declare_parameter("steering_max_ddeg_dt", 3.0);
  // Limit the max change in the des_accel signal over time (acceleration)
  this->declare_parameter("des_accel_max_da_dt", 9.0); // from 5 to 9
  // Limit the max change in the des_accel signal over time (decceleration)
  this->declare_parameter("des_deccel_max_da_dt", 5.0); // keep 5
  // Minimum length of the reference path
  this->declare_parameter("path_min_length_m", 30.0);

  //  Invert steering command for simulation
  this->declare_parameter("invert_steering", false);
  // Use mission status maximum desired velocity
  this->declare_parameter("use_mission_max_vel", true);
  // Use ACC cmd
  this->declare_parameter("use_acc", false);

  // Create Joint LQR Controller from yaml file
  std::string lqr_config_file =
      this->get_parameter("lqr_config_file").as_string();
  if (lqr_config_file.empty())
    throw std::runtime_error(
        "Parameter lqr_config_file not declared, or empty.");

  RCLCPP_INFO(get_logger(), "Loading control params: %s",
              lqr_config_file.c_str());
  joint_lqr_ = joint_lqr::lqr::JointLQR::newPtr(lqr_config_file);

  // Read in misc. parameters
  max_steering_angle_deg_ =
      this->get_parameter("max_steering_angle_deg").as_double();
  steering_units_multiplier_ =
      this->get_parameter("steering_units_multiplier").as_double();
  pure_pursuit_min_dist_m_ =
      this->get_parameter("pure_pursuit_min_dist_m").as_double();
  pure_pursuit_max_dist_m_ =
      this->get_parameter("pure_pursuit_max_dist_m").as_double();
  pure_pursuit_1st_vel_ms_ =
      this->get_parameter("pure_pursuit_1st_vel_ms").as_double();
  pure_pursuit_max_max_dist_m_ =
      this->get_parameter("pure_pursuit_max_max_dist_m").as_double();
  pure_pursuit_k_vel_m_ms_ =
      this->get_parameter("pure_pursuit_k_vel_m_ms").as_double();
  use_tire_velocity_ = this->get_parameter("use_tire_velocity").as_bool();
  odometry_timeout_sec_ =
      this->get_parameter("odometry_timeout_sec").as_double();
  path_timeout_sec_ = this->get_parameter("path_timeout_sec").as_double();
  steering_max_ddeg_dt_ =
      this->get_parameter("steering_max_ddeg_dt").as_double();
  des_accel_max_da_dt_ = this->get_parameter("des_accel_max_da_dt").as_double();
  des_deccel_max_da_dt_ =
      this->get_parameter("des_deccel_max_da_dt").as_double();
  invert_steering_ = this->get_parameter("invert_steering").as_bool();
  m_use_mission_max_vel_ = this->get_parameter("use_mission_max_vel").as_bool();
  m_path_min_length_m = this->get_parameter("path_min_length_m").as_double();
  // m_use_acc = this->get_parameter("use_acc").as_bool();
  m_use_acc = false;

  if (odometry_timeout_sec_ <= 0. || path_timeout_sec_ <= 0.) {
    RCLCPP_ERROR(this->get_logger(),
                 "path and ego_odometry timeouts must be greater than zero. "
                 "Got odometry_timeout_sec_: %f; path_timeout_sec_: %f",
                 odometry_timeout_sec_, path_timeout_sec_);
    throw std::range_error("Parameter out of range.");
  }

  this->parameters_callback_handle =
      this->add_on_set_parameters_callback(std::bind(
          &ControlLQRNode::parametersCallback, this, std::placeholders::_1));
}

void ControlLQRNode::publishSteerAccelDiagnostics(
    bool lqr_command_valid, bool valid_path, bool valid_odom,
    bool valid_wpt_distance, bool valid_target_position,
    double lqr_steering_command, double lqr_accel_command,
    double track_distance, unsigned int lqr_tracking_idx,
    geometry_msgs::msg::PoseStamped lqr_track_point,
    joint_lqr::lqr::JointLQR::ErrorMatrix lqr_err_cog,
    joint_lqr::lqr::JointLQR::ErrorMatrix lqr_err,
    double desired_velocity_mps) {
  std_msgs::msg::Bool command_valid_msg;
  command_valid_msg.data = lqr_command_valid;
  lqr_command_valid_pub_->publish(command_valid_msg);

  std_msgs::msg::Float32MultiArray valid_conditions_msg;
  valid_conditions_msg.data.push_back(lqr_command_valid);
  valid_conditions_msg.data.push_back(valid_path);
  valid_conditions_msg.data.push_back(valid_odom);
  valid_conditions_msg.data.push_back(valid_wpt_distance);
  valid_conditions_msg.data.push_back(valid_target_position);
  lqr_valid_conditions_pub_->publish(valid_conditions_msg);

  std_msgs::msg::Float32 steering_command_msg;
  steering_command_msg.data = lqr_steering_command;
  lqr_steering_command_pub_->publish(steering_command_msg);

  std_msgs::msg::Float32 desired_accel_command_msg;
  desired_accel_command_msg.data = lqr_accel_command;
  lqr_accel_command_pub_->publish(desired_accel_command_msg);

  std_msgs::msg::Float32 track_distance_msg;
  track_distance_msg.data = track_distance;
  track_distance_pub_->publish(track_distance_msg);

  std_msgs::msg::Int32 lqr_tracking_idx_msg;
  lqr_tracking_idx_msg.data = lqr_tracking_idx;
  lqr_tracking_idx_pub_->publish(lqr_tracking_idx_msg);

  lqr_track_point.header.frame_id = "odom";
  lqr_tracking_point_pub_->publish(lqr_track_point);
  auto error_cog_array_msg =
      joint_lqr::utils::ROSError(lqr_err_cog); // Float32MultiArray
  auto error_array_msg =
      joint_lqr::utils::ROSError(lqr_err); // Float32MultiArray
  error_cog_array_msg.data.push_back(error_array_msg.data[0]);
  error_cog_array_msg.data.push_back(error_array_msg.data[1]);
  error_cog_array_msg.data.push_back(error_array_msg.data[2]);
  error_cog_array_msg.data.push_back(error_array_msg.data[3]);
  error_cog_array_msg.data.push_back(error_array_msg.data[4]);
  lqr_error_pub_->publish(error_cog_array_msg);

  std_msgs::msg::Float32 des_vel_msg{};
  des_vel_msg.data = desired_velocity_mps;
  lqr_desired_velocity_mps_pub_->publish(des_vel_msg);
}

nif::common::msgs::ControlCmd::SharedPtr ControlLQRNode::solve() {
  auto now = this->now();
  nif::common::NodeStatusCode node_status = common::NODE_ERROR;

  //  Check whether we have updated data
  bool valid_path =
      this->hasReferenceTrajectory() &&
      !this->getReferenceTrajectory()->trajectory_path.poses.empty() &&
      this->getReferencePathLastPointDistance() > m_path_min_length_m &&
      nif::common::utils::time::secs(
          now - this->getReferenceTrajectoryUpdateTime()) < path_timeout_sec_;

  bool valid_odom =
      this->hasEgoOdometry() &&
      nif::common::utils::time::secs(now - this->getEgoOdometryUpdateTime()) <
          odometry_timeout_sec_;

  // Check valid waypoint starting distance & valid target waypoint
  // position(front)
  // - starting wpt should be around ego.
  bool valid_wpt_distance =
      valid_path && valid_odom &&
      pure_pursuit_max_max_dist_m_ >
          joint_lqr::utils::pursuit_dist(
              this->getReferenceTrajectory()->trajectory_path.poses[0],
              this->getEgoOdometry());

  // - initialize valid target position
  bool valid_target_position = false;

  bool valid_tracking_result = false;

  double steering_angle_deg = 0.0;
  double desired_accel = 0.0;
  joint_lqr::lqr::JointLQR::ErrorMatrix error;
  joint_lqr::lqr::JointLQR::ErrorMatrix error_COG; // error at center of gravity

  // Perform Tracking if path is good
  if (valid_path && valid_odom) {
    // Check whether path is global/local
    bool is_local_path = this->getReferenceTrajectory()->header.frame_id ==
                         this->getBodyFrameId();

    auto state = joint_lqr::utils::LQRState(this->getEgoOdometry());
    if (use_tire_velocity_)
      state(2, 0) = current_speed_ms_;
    if (is_local_path) {
      // x, y, yaw are zeros in local coordinate
      state(0, 0) = 0.0;
      state(1, 0) = 0.0;
      state(4, 0) = 0.0;
    }

    // Compute the tracking distance (and ensure it is within a valid range)
    double track_distance =
        pure_pursuit_min_dist_m_ + pure_pursuit_k_vel_m_ms_ * state(2, 0);
    if (track_distance > pure_pursuit_max_dist_m_) {
      if (state(2, 0) < pure_pursuit_1st_vel_ms_) {
        track_distance = pure_pursuit_max_dist_m_;
      } else {
        track_distance =
            pure_pursuit_max_dist_m_ +
            pure_pursuit_k_vel_m_ms_ * (state(2, 0) - pure_pursuit_1st_vel_ms_);
      }
    }
    if (track_distance < pure_pursuit_min_dist_m_)
      track_distance = pure_pursuit_min_dist_m_;
    if (track_distance > pure_pursuit_max_max_dist_m_)
      track_distance = pure_pursuit_max_max_dist_m_;

    // Track on the trajectory
    double target_distance = 0.0;
    bool target_reached_end = false;
    joint_lqr::utils::track(
        this->getReferenceTrajectory()->trajectory_path.poses,
        this->getEgoOdometry(),
        track_distance, // inputs
        lqr_tracking_idx_, target_distance,
        target_reached_end); // outputs

    // - target point should be ahead.
    double target_point_azimuth = joint_lqr::utils::pursuit_azimuth(
        this->getReferenceTrajectory()
            ->trajectory_path.poses[lqr_tracking_idx_],
        this->getEgoOdometry());
    valid_target_position = M_PI * 3 / 4. > std::abs(target_point_azimuth);

    double l_desired_velocity = 0.0;
    if (valid_wpt_distance && valid_target_position) {
      valid_tracking_result = true;
      // Run LQR :)
      // Desired velocity check

      // deprecated
      //   if (this->hasDesiredVelocity() &&
      //       (this->now() - this->getDesiredVelocityUpdateTime() <=
      //        rclcpp::Duration(1, 0))) {
      //     l_desired_velocity = this->getDesiredVelocity()->data;
      //   }

      if (this->hasReferenceTrajectory() &&
          (this->now() - this->getReferenceTrajectoryUpdateTime() <=
           rclcpp::Duration(1, 0))) {
        // TODO: Review this with Hyunki
        // FIXME:
        // l_desired_velocity = this->getReferenceTrajectory()
        //                          ->trajectory_velocity[lqr_tracking_idx_];

        ////////////////////////////////////
        // Look-ahead time implementation //
        ////////////////////////////////////
        double test_lookahead_time = 0.8;
        // step 1. Search the nearest time within the trajectory's timestamp
        // array
        auto closest_time_idx = nif::common::utils::closestIndex(
            this->getReferenceTrajectory()->trajectory_timestamp_array,
            test_lookahead_time);

        // step 2. Safety feature
        auto time_differ =
            abs(this->getReferenceTrajectory()
                    ->trajectory_timestamp_array[closest_time_idx] -
                test_lookahead_time);

        if (time_differ < 2) {
          l_desired_velocity = this->getReferenceTrajectory()
                                   ->trajectory_velocity[closest_time_idx];
          if (l_desired_velocity < 1.5)
            l_desired_velocity = 0.0;
        } else {
          l_desired_velocity = 0.0;
        }
      }

      if (!m_use_mission_max_vel_) {
        // if not using mission status maximum velocity,
        // directly use des_vel from velocity planner
        l_desired_velocity = direct_desired_velocity_;
      }

      auto goal = joint_lqr::utils::LQRGoal(
          this->getReferenceTrajectory()
              ->trajectory_path.poses[lqr_tracking_idx_],
          l_desired_velocity);
      auto goal_COG = joint_lqr::utils::LQRGoal(
          this->getReferenceTrajectory()->trajectory_path.poses[0],
          l_desired_velocity);
      error = joint_lqr_->computeError(state, goal);
      error_COG = joint_lqr_->computeError(state, goal_COG);
      auto cmd = joint_lqr_->process(state, goal);
      steering_angle_deg = cmd(0, 0) * nif::common::constants::RAD2DEG;
      desired_accel = cmd(1, 0);

      // Make sure steering angle is within range
      if (steering_angle_deg > max_steering_angle_deg_)
        steering_angle_deg = max_steering_angle_deg_;
      if (steering_angle_deg < -max_steering_angle_deg_)
        steering_angle_deg = -max_steering_angle_deg_;

      // ----------------------------------------------------------------------------
      /*
      // APPLY CAMBER COMPENSATION
      */
      double camber_compensatation_deg = 0.0;
      try {
        m_camber_manager_ptr->setVehSpeed(this->current_speed_ms_);
        double tmp_bank_ = 0.0;
        m_camber_manager_ptr->setBankAngle(tmp_bank_);
        camber_compensatation_deg =
            m_camber_manager_ptr->getCamberCompensation();

      } catch (std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "CAMBER COMPENSATION EXCEPTION. %s",
                     e.what());
        camber_compensatation_deg = 0.0;
      }
      steering_angle_deg = steering_angle_deg + camber_compensatation_deg;
      // ----------------------------------------------------------------------------

      // Smooth and publish diagnostics
      double period_double_s =
          nif::common::utils::time::secs(this->getGclockPeriodDuration());
      RCLCPP_DEBUG(this->get_logger(), "Smoothing with dt: [s] %f",
                   period_double_s);
      joint_lqr::utils::smoothSignal(steering_angle_deg, last_steering_command_,
                                     steering_max_ddeg_dt_, period_double_s);

      if (desired_accel > 0) {
        joint_lqr::utils::smoothSignal(desired_accel, last_accel_command_,
                                       des_accel_max_da_dt_, period_double_s);
      } else {
        joint_lqr::utils::smoothSignal(desired_accel, last_accel_command_,
                                       des_deccel_max_da_dt_, period_double_s);
      }
    }
    // Publish diagnostic message
    publishSteerAccelDiagnostics(valid_tracking_result, valid_path, valid_odom,
                                 valid_wpt_distance, valid_target_position,
                                 steering_angle_deg, desired_accel,
                                 track_distance, lqr_tracking_idx_,
                                 this->getReferenceTrajectory()
                                     ->trajectory_path.poses[lqr_tracking_idx_],
                                 error_COG, error, l_desired_velocity);
  }

  if (!this->hasSystemStatus() ||
      (this->getSystemStatus().autonomy_status.lateral_autonomy_enabled ||
       this->getSystemStatus().autonomy_status.longitudinal_autonomy_enabled) &&
          !(valid_path && valid_odom && valid_wpt_distance &&
            valid_target_position)) {
    node_status = common::NODE_ERROR;
    this->setNodeStatus(node_status);
    return nullptr;
  }

  last_steering_command_ = steering_angle_deg;
  last_accel_command_ = desired_accel;
  // for steering command
  this->control_cmd->steering_control_cmd.data =
      invert_steering_ ? -last_steering_command_ : last_steering_command_;
  // for acceleration command

  if (m_use_acc) {
    this->control_cmd->desired_accel_cmd.data =
        std::min(desired_accel, acc_accel_cmd_mpss);
  } else {
    this->control_cmd->desired_accel_cmd.data = desired_accel;
  }

  node_status = common::NODE_OK;
  this->setNodeStatus(node_status);
  return this->control_cmd;
}

rcl_interfaces::msg::SetParametersResult
nif::control::ControlLQRNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &vector) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  for (const auto &param : vector) {
    if (param.get_name() == "pure_pursuit_max_max_dist_m") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 8.0 && param.as_double() <= 20.0) {
          this->pure_pursuit_max_max_dist_m_ = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "pure_pursuit_max_dist_m") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 3.0 && param.as_double() <= 20.0) {
          this->pure_pursuit_max_dist_m_ = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "pure_pursuit_min_dist_m") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 3.0 && param.as_double() <= 20.0) {
          this->pure_pursuit_min_dist_m_ = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "pure_pursuit_1st_vel_ms") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 8.0 && param.as_double() <= 40.0) {
          this->pure_pursuit_1st_vel_ms_ = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "pure_pursuit_k_vel_m_ms") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (param.as_double() >= 0.0 && param.as_double() <= 1.0) {
          this->pure_pursuit_k_vel_m_ms_ = param.as_double();
          result.successful = true;
        }
      }
    }
  }
  return result;
}
