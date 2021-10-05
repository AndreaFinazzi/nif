#include "nif_control_joint_lqr_nodes/control_lqr_node.h"
#include <nif_common/vehicle_model.h>

using nif::control::ControlLQRNode;

ControlLQRNode::ControlLQRNode(const std::string &node_name)
    : IControllerNode(node_name) {

  control_cmd = std::make_shared<nif::common::msgs::ControlCmd>();

  // Debug Publishers
  lqr_command_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "control_joint_lqr/tracking_valid", nif::common::constants::QOS_DEFAULT);
  lqr_steering_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/lqr_command", nif::common::constants::QOS_DEFAULT);
  lqr_accel_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/accel_command", nif::common::constants::QOS_DEFAULT);
  track_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "control_joint_lqr/track_distance", nif::common::constants::QOS_DEFAULT);
  lqr_tracking_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "control_joint_lqr/track_point", nif::common::constants::QOS_DEFAULT);
  lqr_error_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "control_joint_lqr/lqr_error", nif::common::constants::QOS_DEFAULT);

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

  this->declare_parameter("lqr_config_file", "");
  // Automatically boot with lat_autonomy_enabled
  //  this->declare_parameter("lat_autonomy_enabled", false);
  // Max Steering Angle in Degrees
  this->declare_parameter("max_steering_angle_deg", 20.0);
  // convert from degress to steering units (should be 1 - 1 ?)
  this->declare_parameter("steering_units_multiplier", 1.0);
  // Minimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_min_dist_m", 4.0);
  // Maximimum pure pursuit tracking distance
  this->declare_parameter("pure_pursuit_max_dist_m", 8.);
  // Factor to increase the pure pursuit tracking distance as a function of
  // speed (m/s)
  this->declare_parameter("pure_pursuit_k_vel_m_ms", 0.75);
  // Use tire speed instead of gps velocity estimate
  this->declare_parameter("use_tire_velocity", true);
  // Safety timeouts for odometry and the path (set negative to ignore)
  this->declare_parameter("odometry_timeout_sec", 0.1);
  this->declare_parameter("path_timeout_sec", 0.5);
  // Limit the max change in the steering signal over time
  this->declare_parameter("steering_max_ddeg_dt", 5.0);
  // Limit the max change in the des_accel signal over time
  this->declare_parameter("des_accel_max_da_dt", 5.0);
  // Minimum length of the reference path
  this->declare_parameter("path_min_length_m", 30.0);

  //  Invert steering command for simulation
  this->declare_parameter("invert_steering", false);
  // Use mission status maximum desired velocity
  this->declare_parameter("use_mission_max_vel", true);

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
  pure_pursuit_k_vel_m_ms_ =
      this->get_parameter("pure_pursuit_k_vel_m_ms").as_double();
  use_tire_velocity_ = this->get_parameter("use_tire_velocity").as_bool();
  odometry_timeout_sec_ =
      this->get_parameter("odometry_timeout_sec").as_double();
  path_timeout_sec_ = this->get_parameter("path_timeout_sec").as_double();
  steering_max_ddeg_dt_ =
      this->get_parameter("steering_max_ddeg_dt").as_double();
  des_accel_max_da_dt_ = this->get_parameter("des_accel_max_da_dt").as_double();
  invert_steering_ = this->get_parameter("invert_steering").as_bool();
  m_use_mission_max_vel_ = this->get_parameter("use_mission_max_vel").as_bool();
  m_path_min_length_m = this->get_parameter("path_min_length_m").as_double();

  if (odometry_timeout_sec_ <= 0. || path_timeout_sec_ <= 0.) {
    RCLCPP_ERROR(this->get_logger(),
                 "path and ego_odometry timeouts must be greater than zero. "
                 "Got odometry_timeout_sec_: %f; path_timeout_sec_: %f",
                 odometry_timeout_sec_, path_timeout_sec_);
    throw std::range_error("Parameter out of range.");
  }
}

void ControlLQRNode::publishSteerAccelDiagnostics(
    bool lqr_command_valid, double lqr_steering_command,
    double lqr_accel_command, double track_distance,
    geometry_msgs::msg::PoseStamped lqr_track_point,
    joint_lqr::lqr::JointLQR::ErrorMatrix lqr_err) {
  std_msgs::msg::Bool command_valid_msg;
  command_valid_msg.data = lqr_command_valid;
  lqr_command_valid_pub_->publish(command_valid_msg);

  std_msgs::msg::Float32 steering_command_msg;
  steering_command_msg.data = lqr_steering_command;
  lqr_steering_command_pub_->publish(steering_command_msg);

  std_msgs::msg::Float32 desired_accel_command_msg;
  desired_accel_command_msg.data = lqr_accel_command;
  lqr_accel_command_pub_->publish(desired_accel_command_msg);

  std_msgs::msg::Float32 track_distance_msg;
  track_distance_msg.data = track_distance;
  track_distance_pub_->publish(track_distance_msg);

  lqr_tracking_point_pub_->publish(lqr_track_point);
  lqr_error_pub_->publish(joint_lqr::utils::ROSError(lqr_err));
}

nif::common::msgs::ControlCmd::SharedPtr ControlLQRNode::solve() {
  auto now = this->now();
  nif::common::NodeStatusCode node_status = common::NODE_ERROR;

  //  bool lateral_tracking_enabled =
  //      this->get_parameter("lat_autonomy_enabled").as_bool();

  //  Check whether we have updated data
  bool valid_path =
      this->hasReferencePath() && !this->getReferencePath()->poses.empty() &&
      this->getReferencePathLastPointDistance() > m_path_min_length_m &&
      nif::common::utils::time::secs(now - this->getReferencePathUpdateTime()) <
          path_timeout_sec_;
  bool valid_odom =
      this->hasEgoOdometry() &&
      nif::common::utils::time::secs(now - this->getEgoOdometryUpdateTime()) <
          odometry_timeout_sec_;
  bool valid_tracking_result = false;

  double steering_angle_deg = 0.0;
  double desired_accel = 0.0;
  // Perform Tracking if path is good
  if (valid_path && valid_odom) {
    valid_tracking_result = true;

    // Check whether path is global/local
    bool is_local_path =
        this->getReferencePath()->header.frame_id == this->getBodyFrameId();

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
    if (track_distance > pure_pursuit_max_dist_m_)
      track_distance = pure_pursuit_max_dist_m_;
    if (track_distance < pure_pursuit_min_dist_m_)
      track_distance = pure_pursuit_min_dist_m_;

    // Track on the trajectory
    double target_distance = 0.0;
    bool target_reached_end = false;
    joint_lqr::utils::track(this->getReferencePath()->poses,
                            this->getEgoOdometry(), track_distance, // inputs
                            lqr_tracking_idx_, target_distance,
                            target_reached_end); // outputs

    // Run LQR :)

    // Desired velocity check
    double l_desired_velocity = 0.0;
    if (this->hasDesiredVelocity() &&
        (this->now() - this->getDesiredVelocityUpdateTime() <=
         rclcpp::Duration(1, 0))) {
      l_desired_velocity = this->getDesiredVelocity()->data;
    }
    if (!m_use_mission_max_vel_) {
      // if not using mission status maximum velocity,
      // directly use des_vel from velocity planner
      l_desired_velocity = direct_desired_velocity_;
    }

    auto goal = joint_lqr::utils::LQRGoal(
        this->getReferencePath()->poses[lqr_tracking_idx_], l_desired_velocity);
    auto error = joint_lqr_->computeError(state, goal);
    auto cmd = joint_lqr_->process(state, goal);
    steering_angle_deg = cmd(0, 0) * nif::common::constants::RAD2DEG;
    desired_accel = cmd(1, 0);

    // Make sure steering angle is within range
    if (steering_angle_deg > max_steering_angle_deg_)
      steering_angle_deg = max_steering_angle_deg_;
    if (steering_angle_deg < -max_steering_angle_deg_)
      steering_angle_deg = -max_steering_angle_deg_;

    //    Adapt to steering ratio (ControlCommand sends steering wheel's angle)
    //    steering_angle_deg *= nif::common::vehicle_param::STEERING_RATIO;

    // Smooth and publish diagnostics
    double period_double_s =
        nif::common::utils::time::secs(this->getGclockPeriodDuration());
    RCLCPP_DEBUG(this->get_logger(), "Smoothing with dt: [s] %f",
                 period_double_s);
    joint_lqr::utils::smoothSignal(steering_angle_deg, last_steering_command_,
                                   steering_max_ddeg_dt_, period_double_s);
    joint_lqr::utils::smoothSignal(desired_accel, last_accel_command_,
                                   des_accel_max_da_dt_, period_double_s);
    publishSteerAccelDiagnostics(
        true, steering_angle_deg, desired_accel, track_distance,
        this->getReferencePath()->poses[lqr_tracking_idx_], error);
  }

  if (!this->hasSystemStatus() ||
      (this->getSystemStatus().autonomy_status.lateral_autonomy_enabled ||
       this->getSystemStatus().autonomy_status.longitudinal_autonomy_enabled) &&
          !(valid_path && valid_odom)) {
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
  this->control_cmd->desired_accel_cmd.data = desired_accel;

  node_status = common::NODE_OK;
  this->setNodeStatus(node_status);
  return this->control_cmd;
}