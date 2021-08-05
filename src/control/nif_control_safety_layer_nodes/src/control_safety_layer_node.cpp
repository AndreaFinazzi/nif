//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"

void nif::control::ControlSafetyLayerNode::initParameters() {}

void nif::control::ControlSafetyLayerNode::getParameters() {}

void nif::control::ControlSafetyLayerNode::controlCallback(
    const nif::common::msgs::ControlCmd::SharedPtr msg) {
//  TODO consider not to accept commands from the future!
  //  Store control command if it's not too old
  if ((this->now().nanoseconds() - msg->header.stamp.nanosec) <
      this->getGclockPeriod().count() || true) // TODO REMOVE THIS!!!
    this->bufferStore(msg);
}

void nif::control::ControlSafetyLayerNode::run() {

  nif::common::msgs::ControlCmd::SharedPtr msg;
  if (!this->control_buffer.empty()) {

    msg = (this->control_buffer.top());

    // RE-STAMP
    msg->header.stamp = this->now();
    msg->header.frame_id = this->getBodyFrameId();
    try {
      this->control_pub->publish(*msg);

    } catch (std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), e.what());
    }

//    this->flush
  }
}

bool nif::control::ControlSafetyLayerNode::publishSteeringCmd(
    const nif::common::msgs::ControlSteeringCmd &msg) const {
  //  TODO implement safety checks
  if (true) {
    this->steering_control_pub->publish(msg);

    return true; // OK
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishAcceleratorCmd(
    const nif::common::msgs::ControlAcceleratorCmd &msg) const {
  //  TODO implement safety checks
  if (true) {
    this->accelerator_control_pub->publish(msg);

    return true; // OK
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishBrakingCmd(
    const nif::common::msgs::ControlBrakingCmd &msg) const {

  //  TODO implement safety checks
  if (true) {
    this->braking_control_pub->publish(msg);

    return true; // OK
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishGearCmd(
    const nif::common::msgs::ControlGearCmd &msg) const {

  //  TODO implement safety checks
  if (msg.data >= 0 && msg.data < 6) {
    this->gear_control_pub->publish(msg);

    return true; // OK
  }
  return false;
}

void nif::control::ControlSafetyLayerNode::bufferStore(
    const nif::common::msgs::ControlCmd::SharedPtr msg) {

  this->control_buffer.push(msg);
}

void nif::control::ControlSafetyLayerNode::bufferFlush() {
//  this->control_buffer.
}
