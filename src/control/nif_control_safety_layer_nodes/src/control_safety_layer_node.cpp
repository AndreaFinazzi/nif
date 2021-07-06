//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"

nif::control::ControlSafetyLayerNode::ControlSafetyLayerNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options):
  IBaseSynchronizedNode(node_name, options)
{}

void nif::control::ControlSafetyLayerNode::controlCallback(
    const nif::common::msgs::ControlCmd::SharedPtr &msg) {

//  this->control_buffer.push(msg.control_cmd);
}

void nif::control::ControlSafetyLayerNode::run() {
//  const auto selected_cmd = this->control_buffer.pop();
  nif::common::msgs::ControlCmd msg;
//  msg.header.stamp = this->gclock_current;
//  try {
//    this->control_pub->publish(msg);
//  } catch (std::exception &e) {
//    RCLCPP_ERROR(get_logger(), e.what());
//  }
}
