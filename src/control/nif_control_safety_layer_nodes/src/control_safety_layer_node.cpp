//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"

void nif::control::ControlSafetyLayerNode::initParameters() {}

void nif::control::ControlSafetyLayerNode::getParameters() {}

void nif::control::ControlSafetyLayerNode::controlCallback(
    const nif::common::msgs::ControlCmd::SharedPtr &msg) {

  //  this->control_buffer.push(msg.control_cmd);
}

void nif::control::ControlSafetyLayerNode::run() {
  //  const auto selected_cmd = this->control_buffer.pop();
  nif::common::msgs::ControlCmd msg;
  msg.header.stamp = this->gclock_current;

  RCLCPP_DEBUG(this->get_logger(), "ControlSafetyLayerNode::run(): START");
  //  msg.stamp = this->gclock_current;
  try {
    RCLCPP_DEBUG(this->get_logger(),
                "ControlSafetyLayerNode::run(): PUBLISH START");

   this->control_pub->publish(msg);

    RCLCPP_DEBUG(this->get_logger(),
                "ControlSafetyLayerNode::run(): PUBLISH END");
  } catch (std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
  RCLCPP_DEBUG(this->get_logger(), "ControlSafetyLayerNode::run(): END");
}

bool nif::control::ControlSafetyLayerNode::publishSteeringCmd(
    const nif::common::msgs::ControlSteeringCmd &msg) {
  //  TODO implement safety checks
  if (true) {
    this->steering_control_pub->publish(msg);
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishAcceleratorCmd(
    const nif::common::msgs::ControlAcceleratorCmd &msg) {
  //  TODO implement safety checks
  if (true) {
    this->accelerator_control_pub->publish(msg);
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishBrakingCmd(
    const nif::common::msgs::ControlBrakingCmd &msg) {

  //  TODO implement safety checks
  if (true) {
    this->braking_control_pub->publish(msg);
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishGearCmd(
    const nif::common::msgs::ControlGearCmd &msg) {

  //  TODO implement safety checks
  if (true) {
    this->gear_control_pub->publish(msg);
  }
  return false;
}
