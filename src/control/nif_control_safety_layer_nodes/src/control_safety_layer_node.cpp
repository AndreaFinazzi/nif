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
          this->getGclockPeriodNs().count() ||
      true) // TODO REMOVE THIS!!!
    this->bufferStore(msg);
}

void nif::control::ControlSafetyLayerNode::controlOverrideCallback(
    const nif::common::msgs::ControlCmd::SharedPtr msg) {
  // TODO Declare thresholds params and implement override mechanism.
}


void nif::control::ControlSafetyLayerNode::run() {

  if (!this->control_buffer.empty()) {
  nif::common::msgs::ControlCmd::SharedPtr msg;

    msg = (this->control_buffer.top());

    // RE-STAMP
    msg->header.stamp = this->now();
    msg->header.frame_id = this->getBodyFrameId();
    try {
//      TODO implement safety checks
      this->control_pub->publish(*msg);

      this->publishSteeringCmd(msg->steering_control_cmd);
      this->publishAcceleratorCmd(msg->accelerator_control_cmd);
      this->publishBrakingCmd(msg->braking_control_cmd);
      this->publishGearCmd(msg->gear_control_cmd);

    } catch (std::exception &e) {
//      TODO handle critical error in the safest way
      RCLCPP_ERROR(this->get_logger(), e.what());
    }

    this->bufferFlush();
  }
}

bool nif::control::ControlSafetyLayerNode::publishSteeringCmd(
    const nif::common::msgs::ControlSteeringCmd &msg) const {
  //  TODO implement safety checks
  if (this->steering_control_pub) {
    this->steering_control_pub->publish(msg);

    return true; // OK
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishAcceleratorCmd(
    const nif::common::msgs::ControlAcceleratorCmd &msg) const {
  //  TODO implement safety checks
  if (this->accelerator_control_pub) {
    this->accelerator_control_pub->publish(msg);

    return true; // OK
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishBrakingCmd(
    const nif::common::msgs::ControlBrakingCmd &msg) const {

  //  TODO implement safety checks
  if (this->braking_control_pub) {
    this->braking_control_pub->publish(msg);

    return true; // OK
  }
  return false;
}

bool nif::control::ControlSafetyLayerNode::publishGearCmd(
    const nif::common::msgs::ControlGearCmd &msg) const {

  //  TODO implement safety checks
  //  TODO use min/max as constants here
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
  while (!this->control_buffer.empty())
    this->control_buffer.pop();
}
