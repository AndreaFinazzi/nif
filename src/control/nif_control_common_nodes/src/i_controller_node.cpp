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

  this->control_cmd_pub = this->create_publisher<nif::common::msgs::ControlCmd>(
      "out_control_cmd", rclcpp::QoS{1});
}

void nif::control::IControllerNode::controlCmdPrevCallback(
    const nif::common::msgs::ControlCmd::SharedPtr msg) {
  this->control_cmd_prev = msg;
}

void nif::control::IControllerNode::referenceTrajectoryCallback(
    const nif::common::msgs::Trajectory::SharedPtr msg) {
//  TODO clean things out
  this->reference_trajectory = msg;
  this->storeReferenceTrajectory(msg);
}

const nif::common::msgs::Trajectory::SharedPtr &
nif::control::IControllerNode::getReferenceTrajectory() const {
  return reference_trajectory;
}

const nif_msgs::msg::ControlCommand::SharedPtr &
nif::control::IControllerNode::getControlCmdPrev() const {
  return control_cmd_prev;
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
