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

void nif::control::ControlSafetyLayerNode::afterSystemStatusCallback()
{
    if (this->getSystemStatus().health_status.system_failure ||
        this->getSystemStatus().health_status.communication_failure || // too conservative
        this->getSystemStatus().health_status.localization_failure ||
        this->getSystemStatus().health_status.commanded_stop)
    {
        this->emergency_lane_enabled = true;
    } else {
        this->emergency_lane_enabled = false;
    }
}

void nif::control::ControlSafetyLayerNode::controlOverrideCallback(
    const nif::common::msgs::ControlCmd::UniquePtr msg) {
  // TODO Declare thresholds params and implement override mechanism.
  this->override_control_cmd = std::move(*msg);
  this->override_last_update = this->now();
}

void nif::control::ControlSafetyLayerNode::run() {
  bool is_overriding_steering = false;
  nif::common::NodeStatusCode node_status = common::NODE_OK;

  if (
          this->emergency_lane_enabled ||
          !this->hasSystemStatus()     ||
          this->now().nanoseconds() - this->getSystemStatusUpdateTime().nanoseconds() > rclcpp::Duration::from_seconds(0.5).nanoseconds())
  {
//      TODO EMERGENCY LANE HANDLING
        double error = calculateVelocityError();
        this->control_cmd.steering_control_cmd =
            this->override_control_cmd.steering_control_cmd;
        this->control_cmd.gear_control_cmd.data = this->getGearCmd();
        this->control_cmd.accelerator_control_cmd.data = 0.0;
        this->control_cmd.braking_control_cmd.data = this->getBrakeCmd(error);

        this->publishAcceleratorCmd(this->control_cmd.accelerator_control_cmd);
        this->publishBrakingCmd(this->control_cmd.braking_control_cmd);
        this->publishGearCmd(this->control_cmd.gear_control_cmd);
        this->control_pub->publish(this->control_cmd);
  } else {

  // Check / Process Overrides
  // If override.steering is greater than the threshold, use it
  // TODO If override.brake is greater than 0.0, use it
  // TODO If autonomous_mode is disabled, override has full control

  if (std::abs(override_control_cmd.steering_control_cmd.data) >
          std::abs(steering_auto_override_deg) ||
      !lat_autonomy_enabled) {
    this->control_cmd.steering_control_cmd =
        this->override_control_cmd.steering_control_cmd;
    is_overriding_steering = true;
  }

  try {
    if (!is_overriding_steering) {
        if (!this->control_buffer.empty()) {
            this->control_cmd = std::move(*this->control_buffer.top());
            //      TODO implement safety checks
        } else {
            // Set error, but keep going
            // TODO implement fallback policy
            node_status = common::NODE_ERROR;
        }
    }

    // RE-STAMP
    this->control_cmd.header.stamp = this->now();
    this->control_cmd.header.frame_id = this->getBodyFrameId();

    this->control_pub->publish(this->control_cmd);

    this->publishSteeringCmd(this->control_cmd.steering_control_cmd);
    this->publishDesiredAcceleration(this->control_cmd.desired_accel_cmd);
    this->setNodeStatus(node_status);
    //      TODO long_control is in charge of these, at the moment
    //      this->publishAcceleratorCmd(msg->accelerator_control_cmd);
    //      this->publishBrakingCmd(msg->braking_control_cmd);
    //      this->publishGearCmd(msg->gear_control_cmd);

  } catch (...) {
    //      TODO handle critical error in the safest way
      RCLCPP_ERROR(this->get_logger(), "ControlSafetyLayerNode has caught an exception, enabling emergency lane control");
//    Notify the SystemStatusManager of the change.
    this->emergency_lane_enabled = true;
    this->setNodeStatus(common::NodeStatusCode::NODE_ERROR);
  }
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
    if (param.get_name() == "lat_autonomy_enabled") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (true) // TODO implement switching policy, if needed
        {
          this->lat_autonomy_enabled = param.as_bool();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "long_autonomy_enabled") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            if (true) {
                this->long_autonomy_enabled = param.as_bool();
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
