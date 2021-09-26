//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

#include "nif_control_common_nodes/i_controller_node.h"

using nif_msgs::msg::MissionStatus;

nif::control::IControllerNode::IControllerNode(const std::string &node_name)
    : nif::common::IBaseSynchronizedNode(node_name, common::NodeType::CONTROL, common::constants::SYNC_PERIOD_DEFAULT_US) {

  //  ==== DECLARE PARAMETERS ====

  //  ==== DECLARE PARAMETERS (EMD) ====

  this->control_cmd_prev_sub =
      this->create_subscription<nif::common::msgs::ControlCmd>(
          "in_control_cmd_prev", nif::common::constants::QOS_CONTROL_CMD,
          std::bind(&IControllerNode::controlCmdPrevCallback, this,
                    std::placeholders::_1));

  this->reference_trajectory_sub =
      this->create_subscription<nif::common::msgs::Trajectory>(
          "in_reference_trajectory", nif::common::constants::QOS_PLANNING,
        std::bind(&IControllerNode::referenceTrajectoryCallback, this,
                  std::placeholders::_1));

  this->reference_path_sub =
      this->create_subscription<nif::common::msgs::Path>(
          "in_reference_path", nif::common::constants::QOS_PLANNING,
          std::bind(&IControllerNode::referencePathCallback, this,
                    std::placeholders::_1));

  this->desired_velocity_sub = this->create_subscription<std_msgs::msg::Float32>(
          "velocity_planner/des_vel", nif::common::constants::QOS_CONTROL_CMD,
      std::bind(&IControllerNode::desiredVelocityCallback, this,
                std::placeholders::_1));

  this->control_cmd_pub = this->create_publisher<nif::common::msgs::ControlCmd>(
      "out_control_cmd", nif::common::constants::QOS_CONTROL_CMD);

  this->desired_velocity = std::make_shared<std_msgs::msg::Float32>();
}

/**
 * Called every IBaseSynchronizedNode::period.
 * Call solve() and publish the control message.
 */
void nif::control::IControllerNode::run() {
  nif_msgs::msg::ControlCommand::SharedPtr msg = this->solve();
  if (msg) {
    msg->header.stamp = this->now();
    this->control_cmd_pub->publish(*msg);

  }
}


void nif::control::IControllerNode::controlCmdPrevCallback(
    const nif::common::msgs::ControlCmd::SharedPtr msg) {
  this->has_control_cmd_prev = true;
  this->control_cmd_prev_update_time = this->now();
  this->control_cmd_prev = msg;
}

void nif::control::IControllerNode::referenceTrajectoryCallback(
    const nif::common::msgs::Trajectory::SharedPtr msg) {
  this->has_reference_trajectory = true;
  this->reference_trajectory_update_time = this->now();
  this->reference_trajectory = msg;
  this->afterReferenceTrajectoryCallback();
}

void nif::control::IControllerNode::referencePathCallback(
    nif::common::msgs::Path::SharedPtr msg) {
  this->has_reference_path = true;
  this->reference_path_update_time = this->now();
  this->reference_path = msg;
  this->afterReferencePathCallback();
}

void nif::control::IControllerNode::desiredVelocityCallback(
    std_msgs::msg::Float32::SharedPtr msg) {
  this->has_desired_velocity = true;
  this->desired_velocity_update_time = this->now();

    // CHECK MISSION CONDITIONS
    if (this->hasSystemStatus()) {
      if (msg->data > this->getSystemStatus().mission_status.max_velocity_mps) {
        this->desired_velocity->data =  this->getSystemStatus().mission_status.max_velocity_mps;
      } else {
        this->desired_velocity->data = msg->data;
      }
  } else {
    this->desired_velocity->data = 0;
  }

  this->afterDesiredVelocityCallback();
}

const nif::common::msgs::Trajectory::SharedPtr &
nif::control::IControllerNode::getReferenceTrajectory() const {
  return reference_trajectory;
}
const nif_msgs::msg::ControlCommand::SharedPtr &
nif::control::IControllerNode::getControlCmdPrev() const {
  return control_cmd_prev;
}
const nav_msgs::msg::Path::SharedPtr &
nif::control::IControllerNode::getReferencePath() const {
  return reference_path;
}
const std_msgs::msg::Float32::SharedPtr &
nif::control::IControllerNode::getDesiredVelocity() const {
  return desired_velocity;
}
const rclcpp::Time &
nif::control::IControllerNode::getReferenceTrajectoryUpdateTime() const {
  return reference_trajectory_update_time;
}
const rclcpp::Time &
nif::control::IControllerNode::getReferencePathUpdateTime() const {
  return reference_path_update_time;
}
const rclcpp::Time &
nif::control::IControllerNode::getControlCmdPrevUpdateTime() const {
  return control_cmd_prev_update_time;
}
const rclcpp::Time &
nif::control::IControllerNode::getDesiredVelocityUpdateTime() const {
  return desired_velocity_update_time;
}

/**
 * Return true if reference_trajectory has been initialized.
 * false otherwise
 */
bool nif::control::IControllerNode::hasReferenceTrajectory() const {
  return has_reference_trajectory;
}

/**
 * Return true if reference_path has been initialized.
 * false otherwise
 */
bool nif::control::IControllerNode::hasReferencePath() const {
  return has_reference_path;
}

/**
 * Return true if control_cmd_prev has been initialized.
 * false otherwise
 */
bool nif::control::IControllerNode::hasControlCmdPrev() const {
  return has_control_cmd_prev;
}

/**
 * Return true if desired_velocity has been initialized.
 * false otherwise
 */
bool nif::control::IControllerNode::hasDesiredVelocity() const {
  return has_desired_velocity;
}

