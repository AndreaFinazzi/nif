//
// Created by usrg on 7/17/21.
//

#include "nif_control_minimal_nodes/control_minimal_node.h"

using nif::control::ControlMinimalNode;

ControlMinimalNode::ControlMinimalNode(const std::string& node_name)
  : IControllerNode(node_name),
      recv_time_(this->now()) {
  // Publishers
  pubSteeringCmd_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/steering_cmd", 1);
  pubLookaheadError_ =
      this->create_publisher<std_msgs::msg::Float64>("lookahead_error", 10);
  pubLatError_ =
      this->create_publisher<std_msgs::msg::Float64>("lateral_error", 10);
  // Subscribers
  subPath_ = this->create_subscription<nav_msgs::msg::Path>(
      "target_path",
      1,
      std::bind(&ControlMinimalNode::receivePath, this, std::placeholders::_1));
  subVelocity_ =
      this->create_subscription<novatel_gps_msgs::msg::NovatelVelocity>(
          "novatel_bottom/bestvel",
          1,
          std::bind(&ControlMinimalNode::receiveVelocity,
                    this,
                    std::placeholders::_1));
  subJoySteering_ = this->create_subscription<std_msgs::msg::Float32>(
      "/joystick/steering_cmd_not_used",
      1,
      std::bind(
          &ControlMinimalNode::receiveJoySteer, this, std::placeholders::_1));

  // Create Timer
  //  control_timer_ = this->create_wall_timer(
  //      std::chrono::milliseconds(10),
  //      std::bind(&ControlMinimalNode::controlCallback, this));

  // Parameters
  this->declare_parameter("min_lookahead", 4.0);
  this->declare_parameter("max_lookahead", 50.0);
  this->declare_parameter("lookahead_speed_ratio", 0.75);
  this->declare_parameter("proportional_gain", 0.2);
  this->declare_parameter("vehicle.wheelbase", 2.97);
  this->declare_parameter("max_steer_angle", 30.0); // 15 deg * 2 because ratio is wrong
  this->declare_parameter("steering_override_threshold", 4.0);

  lookahead_error.data = 0.0;
  lat_error.data = 0.0;
  steering_cmd.data = 0.0;
  control_cmd = std::make_shared<nif::common::msgs::ControlCmd>();
}

void nif::control::ControlMinimalNode::initParameters() {}
void nif::control::ControlMinimalNode::getParameters() {}

nif::common::msgs::ControlCmd& ControlMinimalNode::solve() {
  if (this->path_ready)
  {

  rclcpp::Time control_time = this->now();
  rclcpp::Duration time_diff = control_time - this->recv_time_;
  double dt = static_cast<double>(time_diff.seconds()) +
      static_cast<double>(time_diff.nanoseconds()) * 1e-9;

  if (dt < 0.5) {
    calculateFFW();
    calculateFB();
    calculateSteeringCmd();
  } else {
    RCLCPP_DEBUG(
        this->get_logger(), "%s\n", "Have not received new path in > 0.5s !");
    setCmdsToZeros();
  }

  publishSteering();
  publishDebugSignals();
  this->control_cmd->steering_control_cmd.data = this->steering_cmd.data;

  }
  return *(this->control_cmd);
}

void ControlMinimalNode::calculateFFW() {
  // Desired yaw rate from feedforward
  this->feedforward_ = 0.0; // can add feedforward with a curvature:
                            // this->speed_ * this->curvature_;
}

void ControlMinimalNode::calculateFB() {
  // Desired yaw rate from feedback
  double Kp = this->get_parameter("proportional_gain").as_double();
  this->feedback_ = -Kp * this->lookahead_error.data;
}

void ControlMinimalNode::calculateSteeringCmd() {
  // Convert desired yaw rate to steering angle using kinematic model
  double L = this->get_parameter("vehicle.wheelbase").as_double();
  this->steering_cmd.data = (this->speed_ > 1.0) ?
      L * (this->feedback_ + this->feedforward_) / this->speed_ :
      L * (this->feedback_ + this->feedforward_);
  this->steering_cmd.data = this->steering_cmd.data * 57.296 * 19.0 /
      9.0; // times 19.0/9 because ratio is wrong
}

void ControlMinimalNode::setCmdsToZeros() {
  this->feedforward_ = 0.0;
  this->feedback_ = 0.0;
  this->lookahead_error.data = 0.0;
  this->curvature_ = 0.0;
  this->speed_ = 0.0;

  this->steering_cmd.data = 0.0;
}

void ControlMinimalNode::publishSteering() {
  double ms = this->get_parameter("max_steer_angle").as_double();
  if (this->steering_cmd.data > ms) {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Max");
    this->steering_cmd.data = ms;
  }
  if (this->steering_cmd.data < -ms) {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Min");
    this->steering_cmd.data = -ms;
  }

  double steer_override_threshold =
      this->get_parameter("steering_override_threshold").as_double();
  this->steering_cmd.data =
      (fabs(this->steering_override_) >= steer_override_threshold) ?
      this->steering_override_ :
      this->steering_cmd.data;

  pubSteeringCmd_->publish(this->steering_cmd);
}

void ControlMinimalNode::publishDebugSignals() {
  pubLookaheadError_->publish(this->lookahead_error);
  pubLatError_->publish(this->lat_error);
}

void ControlMinimalNode::receivePath(const nav_msgs::msg::Path::SharedPtr msg) {
  // Determines lookahead distance based on speed and bounds
  double min_la = this->get_parameter("min_lookahead").as_double();
  double max_la = this->get_parameter("max_lookahead").as_double();
  double la_ratio = this->get_parameter("lookahead_speed_ratio").as_double();
  double lookahead_distance =
      std::max(min_la, std::min(max_la, this->speed_ * la_ratio));

  // Unpacks the message and finds the index correlated to the lookahead
  // distance
  std::vector<geometry_msgs::msg::PoseStamped> path = msg->poses;
  int idx = findLookaheadIndex(path, lookahead_distance);

  // Sets the lookahead and lateral error
  if (path.size() > 0) {
    this->lookahead_error.data = path[idx].pose.position.y;
    this->lat_error.data = path[0].pose.position.y;
  } else {
    this->lookahead_error.data = 0.0;
  }

  this->recv_time_ = this->now();
  this->path_ready = true;
}

int ControlMinimalNode::findLookaheadIndex(
    std::vector<geometry_msgs::msg::PoseStamped> refPath,
    double desLookaheadValue) {
  // Finds first path pose that has x distance > lookahead distance
  for (int i = 0; i < refPath.size(); i++) {
    if (refPath[i].pose.position.x > desLookaheadValue) {
      return i;
    }
  }
  return refPath.size() - 1;
}

void ControlMinimalNode::receiveVelocity(
    const novatel_gps_msgs::msg::NovatelVelocity::SharedPtr msg) {
  this->speed_ = msg->horizontal_speed;
}

void ControlMinimalNode::receiveJoySteer(
    const std_msgs::msg::Float32::SharedPtr msg) {
  this->steering_override_ = msg->data;
}
