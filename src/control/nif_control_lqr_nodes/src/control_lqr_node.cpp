#include "nif_control_lqr_nodes/control_lqr_node.h"
#include <nif_common/vehicle_model.h>

using nif::control::ControlLQRNode;

ControlLQRNode::ControlLQRNode(const std::string &node_name)
: IControllerNode(node_name) {

  control_cmd = std::make_shared<nif::common::msgs::ControlCmd>();

  // Debug Publishers
  lqr_command_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "bvs_controller/tracking_valid", 1);
  lqr_steering_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "bvs_controller/lqr_command", 1);
  track_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "bvs_controller/track_distance", 1);
  lqr_tracking_point_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseStamped>(
          "bvs_controller/track_point", 1);
  lqr_error_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "bvs_controller/lqr_error", 1);

  // Command Publishers
  steering_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/steering_cmd", 1);
  throttle_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/accelerator_cmd", 1);
  brake_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/brake_cmd", 1);
  gear_command_pub_ =
      this->create_publisher<std_msgs::msg::UInt8>("/joystick/gear_cmd", 1);

  // Subscribers
  velocity_sub_ =
      this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
          "/raptor_dbw_interface/wheel_speed_report", 1,
          std::bind(&ControlLQRNode::velocityCallback, this,
                    std::placeholders::_1));
  steering_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/joystick/steering_cmd_not_used", 1,
      std::bind(&ControlLQRNode::steeringCallback, this,
                std::placeholders::_1));
  throttle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/joystick/accelerator_cmd_max", 1,
      std::bind(&ControlLQRNode::throttleCallback, this,
                std::placeholders::_1));
  brake_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/joystick/brake_cmd_override", 1,
      std::bind(&ControlLQRNode::brakeCallback, this,
                std::placeholders::_1));
  gear_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
      "/joystick/gear_cmd_not_used", 1,
      std::bind(&ControlLQRNode::gearCallback, this,
                std::placeholders::_1));

  // Gives memory exceptions with my test rosbag :(
  // pt_report_sub_ =
  // this->create_subscription<deep_orange_msgs::msg::PtReport>("/raptor_dbw_interface/pt_report",
  //     1, std::bind(&PathFollowerNode::ptReportCallback, this,
  //     std::placeholders::_1));

  //! Timer to execute control at 25Hz
//  control_timer_ = this->create_wall_timer(
//      std::chrono::milliseconds(static_cast<int>(1000. / update_rate_hz)),
//      std::bind(&PathFollowerNode::executeControl, this));

  this->declare_parameter("lqr_config_file", "");
  // Automatically boot with lateral_tracking_enabled
  this->declare_parameter("lateral_tracking_enabled", false);
  // Max Steering Angle in Degrees
  this->declare_parameter("max_steering_angle_deg", 20.0);
  // Degrees at which to automatically revert to the override
  this->declare_parameter("steering_auto_override_deg", 4.0);
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

//  Invert steering command for simulation
  this->declare_parameter("invert_steering", false);

  // Create Lateral LQR Controller from yaml file
  std::string lqr_config_file =
      this->get_parameter("lqr_config_file").as_string();
  if (lqr_config_file.empty())
    throw std::runtime_error("Parameter lqr_config_file not declared, or empty.");

  RCLCPP_INFO(get_logger(), "Loading control params: %s",
              lqr_config_file.c_str());
  lateral_lqr_ = bvs_control::lqr::LateralLQR::newPtr(lqr_config_file);

  // Read in misc. parameters
  max_steering_angle_deg_ =
      this->get_parameter("max_steering_angle_deg").as_double();
  steering_auto_override_deg_ =
      this->get_parameter("steering_auto_override_deg").as_double();
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

  if (  (odometry_timeout_sec_) < 0. ||
        (path_timeout_sec_) < 0.) {
    throw rclcpp::exceptions::InvalidParametersException("odometry_timeout_sec_ or path_timeout_sec_ parameter is negative.");
  }
}

void
ControlLQRNode::publishSteeringDiagnostics(bool lqr_command_valid,
                           double lqr_steering_command, double track_distance,
                           geometry_msgs::msg::PoseStamped lqr_track_point,
                           bvs_control::lqr::LateralLQR::ErrorMatrix lqr_err) {
  std_msgs::msg::Bool command_valid_msg;
  command_valid_msg.data = lqr_command_valid;
  lqr_command_valid_pub_->publish(command_valid_msg);

  std_msgs::msg::Float32 steering_command_msg;
  steering_command_msg.data = lqr_steering_command;
  lqr_steering_command_pub_->publish(steering_command_msg);

  std_msgs::msg::Float32 track_distance_msg;
  track_distance_msg.data = track_distance;
  track_distance_pub_->publish(track_distance_msg);

  lqr_tracking_point_pub_->publish(lqr_track_point);
  lqr_error_pub_->publish(bvs_control::utils::ROSError(lqr_err));
}

nif::common::msgs::ControlCmd::SharedPtr ControlLQRNode::solve() {
  auto now = this->now();

  bool lateral_tracking_enabled =
      this->get_parameter("lateral_tracking_enabled").as_bool();

  bool invert_steering =
      this->get_parameter("invert_steering").as_bool();
//  Check whether we have updated data
  bool valid_path = this->hasReferencePath() &&
                    this->getReferencePath()->poses.size() > 0 &&
                    (secs(now - this->getReferencePathUpdateTime()) < path_timeout_sec_ ||
                     path_timeout_sec_ < 0.0);
  bool valid_odom = this->hasEgoOdometry() &&
                    secs(now - this->getEgoOdometryUpdateTime()) < odometry_timeout_sec_ ||
                    odometry_timeout_sec_ < 0.0;
  bool valid_tracking_result = false;

  double steering_angle_deg = 0.0;
  // Perform Tracking if path is good
  if (valid_path && valid_odom) {
    valid_tracking_result = true;

    auto state = bvs_control::utils::LQRState(this->getEgoOdometry());
    if (use_tire_velocity_)
      state(2, 0) = current_speed_ms_;

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
    bvs_control::utils::track(
        this->getReferencePath()->poses, this->getEgoOdometry(), track_distance, // inputs
        lqr_tracking_idx_, target_distance, target_reached_end); // outputs

    // Run LQR :)
    auto goal = bvs_control::utils::LQRGoal(this->getReferencePath()->poses[lqr_tracking_idx_]);
    auto error = lateral_lqr_->computeError(state, goal);
    steering_angle_deg = lateral_lqr_->process(state, goal); // * nif::common::constants::RAD2DEG;

    // Make sure steering angle is within range
    if (steering_angle_deg > max_steering_angle_deg_)
      steering_angle_deg = max_steering_angle_deg_;
    if (steering_angle_deg < -max_steering_angle_deg_)
      steering_angle_deg = -max_steering_angle_deg_;

//    Adapt to steering ratio (ControlCommand sends steering wheel's angle)
    steering_angle_deg *= nif::common::vehicle_param::STEERING_RATIO;

    // Smooth and publish diagnostics
    std::chrono::milliseconds period_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            this->getGclockPeriodNs());
    double period_double_s = period_ms.count() / 1000. ;
    RCLCPP_DEBUG(this->get_logger(), "Smoothing with dt: [s] %f", period_double_s );
    bvs_control::utils::smoothSignal(steering_angle_deg, last_steering_command_,
                                     steering_max_ddeg_dt_, period_double_s );
    publishSteeringDiagnostics(true, steering_angle_deg, track_distance,
                               this->getReferencePath()->poses[lqr_tracking_idx_], error);
  }

  // Check / Process Overrides
  bool override_sig = false;
  if (std::abs(override_steering_target_) >
          std::abs(steering_auto_override_deg_) ||
      !lateral_tracking_enabled) {
      steering_angle_deg = override_steering_target_;
      override_sig = true;
  }

  if ( !(valid_path && valid_odom) && !override_sig)
    return nullptr;

  last_steering_command_ = steering_angle_deg;
  this->control_cmd->steering_control_cmd.data = invert_steering ? -last_steering_command_ : last_steering_command_;

  return this->control_cmd;
}