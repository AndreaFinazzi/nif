//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

#include "nif_control_common_nodes/i_controller_node.h"

nif::control::IControllerNode::IControllerNode(const std::string &node_name)
    : nif::common::IBaseSynchronizedNode(node_name, common::NodeType::CONTROL, common::constants::SYNC_PERIOD_DEFAULT) {

  //  ==== DECLARE PARAMETERS ====

  //  ==== DECLARE PARAMETERS (EMD) ====

  this->control_cmd_prev_sub =
      this->create_subscription<nif::common::msgs::ControlCmd>(
          "in_control_cmd_prev", nif::common::constants::QOS_DEFAULT,
          std::bind(&IControllerNode::controlCmdPrevCallback, this,
                    std::placeholders::_1));

  this->reference_trajectory_sub =
      this->create_subscription<nif::common::msgs::Trajectory>(
        "in_reference_trajectory", nif::common::constants::QOS_DEFAULT,
        std::bind(&IControllerNode::referenceTrajectoryCallback, this,
                  std::placeholders::_1));

  this->reference_path_sub =
      this->create_subscription<nif::common::msgs::Path>(
          "in_reference_path", nif::common::constants::QOS_DEFAULT,
          std::bind(&IControllerNode::referencePathCallback, this,
                    std::placeholders::_1));

  this->control_cmd_pub = this->create_publisher<nif::common::msgs::ControlCmd>(
      "out_control_cmd", rclcpp::QoS{1});
}

/**
 * Called every IBaseSynchronizedNode::period.
 * Call solve() and publish the control message.
 */
void nif::control::IControllerNode::run() {
  auto msg = this->solve();

  msg.header.stamp = this->now();
  this->control_cmd_pub->publish(msg);
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
