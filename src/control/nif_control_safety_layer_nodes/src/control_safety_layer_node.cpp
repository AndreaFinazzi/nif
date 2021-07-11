//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"

nif::control::ControlSafetyLayerNode::ControlSafetyLayerNode(
    const std::string &node_name,
    const rclcpp::NodeOptions &options):
  IBaseSynchronizedNode(node_name, options)
{
  this->control_pub = this->create_publisher<nif::common::msgs::ControlCmd>("/control_safety_layer/control_cmd", rclcpp::QoS{5});
}

void nif::control::ControlSafetyLayerNode::initParameters() {}

void nif::control::ControlSafetyLayerNode::getParameters() {}

void nif::control::ControlSafetyLayerNode::controlCallback(
    const nif::common::msgs::ControlCmd::SharedPtr &msg) {

//  this->control_buffer.push(msg.control_cmd);
}


void nif::control::ControlSafetyLayerNode::run() {
//  const auto selected_cmd = this->control_buffer.pop();
  auto msg = nif::common::msgs::ControlCmd();

  RCLCPP_INFO(this->get_logger(), "ControlSafetyLayerNode::run(): START");
//  msg.stamp = this->gclock_current;
  try {
    RCLCPP_INFO(this->get_logger(), "ControlSafetyLayerNode::run(): PUBLISH START");
    this->control_pub->publish(msg);
    RCLCPP_INFO(this->get_logger(), "ControlSafetyLayerNode::run(): PUBLISH END");
  } catch (std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
  RCLCPP_INFO(this->get_logger(), "ControlSafetyLayerNode::run(): END");
}
