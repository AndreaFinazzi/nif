//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"

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
  this->override_control_cmd = std::move(*msg);
  this->override_last_update = this->now();
}

void nif::control::ControlSafetyLayerNode::run() {
  // Check / Process Overrides
  // If override.steering is greater than the threshold, use it
  // TODO If override.brake is greater than 0.0, use it
  // TODO If autonomous_mode is disabled, override has full control
  bool is_overriding_steering = false;
  if (std::abs(override_control_cmd.steering_control_cmd.data) >
          std::abs(steering_auto_override_deg) ||
      !lateral_tracking_enabled) {
    this->control_cmd.steering_control_cmd =
        this->override_control_cmd.steering_control_cmd;
    is_overriding_steering = true;
  }

  if (!is_overriding_steering && !this->control_buffer.empty()) {
    this->control_cmd = std::move(*this->control_buffer.top());
  }

  // RE-STAMP
  this->control_cmd.header.stamp = this->now();
  this->control_cmd.header.frame_id = this->getBodyFrameId();

  try {
    //      TODO implement safety checks
    this->control_pub->publish(this->control_cmd);

    this->publishSteeringCmd(this->control_cmd.steering_control_cmd);

    //      TODO long_control is in charge of these, at the moment
    //      this->publishAcceleratorCmd(msg->accelerator_control_cmd);
    //      this->publishBrakingCmd(msg->braking_control_cmd);
    //      this->publishGearCmd(msg->gear_control_cmd);

  } catch (std::exception &e) {
    //      TODO handle critical error in the safest way
    RCLCPP_ERROR(this->get_logger(), e.what());
  }
  this->bufferFlush();
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
rcl_interfaces::msg::SetParametersResult
nif::control::ControlSafetyLayerNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &vector) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  for (const auto &param : vector) {
    if (param.get_name() == "lateral_tracking_enabled") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (true) // TODO implement switching policy, if needed
        {
          this->lateral_tracking_enabled = param.as_bool();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "max_steering_angle_deg") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->max_steering_angle_deg = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "steering_auto_override_deg") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->steering_auto_override_deg = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "steering_units_multiplier") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->steering_units_multiplier = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "odometry_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->odometry_timeout_sec = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "path_timeout_sec") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->path_timeout_sec = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "steering_max_ddeg_dt") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->steering_max_ddeg_dt = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "invert_steering") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (true) {
          this->invert_steering = param.as_bool();
          result.successful = true;
        }
      }
    }
  }
  return result;
}
