//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"

void nif::control::ControlSafetyLayerNode::controlCallback(
        const nif::common::msgs::ControlCmd::SharedPtr msg) {
    //  TODO consider not to accept commands from the future!
    //  Store control command if it's not too old
    if ((this->now() - msg->header.stamp) <
        this->getGclockPeriodDuration() )
        this->bufferStore(msg);
}

void nif::control::ControlSafetyLayerNode::afterSystemStatusCallback() {
    //    this->getSystemStatus().health_status.system_failure
    if (this->getSystemStatus().health_status.communication_failure || // too conservative
        (this->lat_autonomy_enabled && this->getSystemStatus().health_status.localization_failure) ||
        this->getSystemStatus().health_status.commanded_stop ||
        this->getSystemStatus().autonomy_status.emergency_mode_enabled ) 
    {
        this->emergency_lane_enabled = true;
    } else {
        this->emergency_lane_enabled = false;
    }

    this->lat_autonomy_enabled = this->getSystemStatus().autonomy_status.lateral_autonomy_enabled;
    this->long_autonomy_enabled = this->getSystemStatus().autonomy_status.longitudinal_autonomy_enabled;
}

void nif::control::ControlSafetyLayerNode::controlOverrideCallback(
        const nif::common::msgs::ControlCmd::UniquePtr msg) {
    // TODO Declare thresholds params and implement override mechanism.
    this->override_control_cmd = std::move(*msg);
    this->override_last_update = this->now();
}

void nif::control::ControlSafetyLayerNode::perceptionSteeringCallback(
        const std_msgs::msg::Float32::UniquePtr msg) {
    this->has_perception_steering = true;
    this->perception_steering_cmd = msg->data;
    this->perception_steering_last_update = this->now();
}

void nif::control::ControlSafetyLayerNode::run() {
    // send diagnostic hb to vehicle interface
    auto joy_hb = std_msgs::msg::Int32();
    joy_hb.data = this->counter_hb;
    this->diagnostic_hb_pub->publish(joy_hb);
    this->counter_hb++;
    if (this->counter_hb == 8) {
        this->counter_hb = 0;
    }

    this->control_cmd.header.stamp = this->now();
    
    bool is_overriding_steering = false;
    bool is_buffer_empty = this->control_buffer.empty();
    
    nif::common::NodeStatusCode node_status = common::NODE_ERROR;

    if (    this->emergency_lane_enabled ||
            this->emergency_buffer_empty ||
            !this->hasSystemStatus())
    {
//      TODO EMERGENCY LANE HANDLING
        double velocity_error = emergencyVelocityError();
        double safe_brake_cmd = this->emergencyBrakeCmd(velocity_error);
//        RCLCPP_INFO(this->get_logger(), "safe_brake_cmd: %f", safe_brake_cmd);
        safe_brake_cmd = std::max(safe_brake_cmd,
                                  static_cast<double>(this->override_control_cmd.braking_control_cmd.data));
//        RCLCPP_INFO(this->get_logger(), "override_brake_cmd: %f", safe_brake_cmd);
        this->control_cmd.braking_control_cmd.data = safe_brake_cmd;
        this->publishBrakingCmd(this->control_cmd.braking_control_cmd);

        this->control_cmd.gear_control_cmd.data = this->getGearCmd();
        this->publishGearCmd(this->control_cmd.gear_control_cmd);

        this->control_cmd.accelerator_control_cmd.data = 0.0;
        this->publishAcceleratorCmd(this->control_cmd.accelerator_control_cmd);

//        TODO this steering command should derive from perception-based controller
        if (!this->getSystemStatus().health_status.communication_failure && 
            std::abs(override_control_cmd.steering_control_cmd.data) > std::abs(steering_auto_override_deg)
            ) {
            this->control_cmd.steering_control_cmd =
                    this->override_control_cmd.steering_control_cmd;
            is_overriding_steering = true;

        } else if ( !this->emergency_buffer_empty   &&
                    !is_buffer_empty                &&
                    !this->getSystemStatus().health_status.localization_failure ) {
            
                auto top_control_cmd = std::move(*this->control_buffer.top());
                this->control_cmd.steering_control_cmd = top_control_cmd.steering_control_cmd;

        } else if ( this->has_perception_steering && 
                    this->now() - this->perception_steering_last_update < rclcpp::Duration(1, 0)) {
            this->control_cmd.steering_control_cmd.data =
                this->perception_steering_cmd;

        } else {
            this->control_cmd.steering_control_cmd.data = 0.0;
        }
        this->publishSteeringCmd(this->control_cmd.steering_control_cmd);

        this->control_pub->publish(this->control_cmd);
        if (this->emergency_buffer_empty ||
            !this->hasSystemStatus())
        { 
            node_status = common::NODE_ERROR;
        } else {
            node_status = common::NODE_OK;
        }

        this->setNodeStatus(node_status);
        this->bufferFlush();
        
        // Recover emergency buffer empty in manual mode
        if (this->emergency_buffer_empty &&
            !lat_autonomy_enabled &&
            !long_autonomy_enabled )
            this->emergency_buffer_empty = false;

        return;
    } else {
        node_status = common::NODE_OK;
        // Check / Process Overrides
        // If override.steering is greater than the threshold, use it
        // TODO If override.brake is greater than 0.0, use it
        // TODO If autonomous_mode is disabled, override has full control
        // RE-STAMP
        this->control_cmd.header.frame_id = this->getBodyFrameId();

        if (!lat_autonomy_enabled ||
            std::abs(override_control_cmd.steering_control_cmd.data) >
            std::abs(steering_auto_override_deg)
            ) {
            this->control_cmd.steering_control_cmd =
                    this->override_control_cmd.steering_control_cmd;
            is_overriding_steering = true;
        }

        try {
            if (lat_autonomy_enabled || long_autonomy_enabled) {
                if (!is_buffer_empty) {
                    auto top_control_cmd = std::move(*this->control_buffer.top());
                    this->buffer_empty_counter = 0;

                    if (!is_overriding_steering)
                        this->control_cmd.steering_control_cmd = top_control_cmd.steering_control_cmd;

                    if (long_autonomy_enabled) {
                        this->control_cmd.desired_accel_cmd = top_control_cmd.desired_accel_cmd;
                    } else {
                        this->control_cmd.desired_accel_cmd.data = 0.0;
                    }
                    //      TODO implement safety checks
                } else {
                    // Set error, but keep going
                    // TODO implement fallback policy
                    this->buffer_empty_counter++;
                    if (this->buffer_empty_counter >= this->buffer_empty_counter_threshold) {
                        // TODO recover mechanism should be enabled
                        this->emergency_buffer_empty = true;
                        node_status = common::NODE_ERROR;
                        this->setNodeStatus(node_status);
                        return;
                    } else {
                        return;
                    }
                }
            } else {
                this->emergency_buffer_empty = false;
            }

            this->publishSteeringCmd(this->control_cmd.steering_control_cmd);

            //    Always allow braking override
            if (this->override_control_cmd.braking_control_cmd.data > 100.0)
            {
                this->control_cmd.braking_control_cmd =
                        this->override_control_cmd.braking_control_cmd;
                this->control_cmd.accelerator_control_cmd.data = 0.0;
                this->publishBrakingCmd(this->control_cmd.braking_control_cmd);
                this->publishAcceleratorCmd(this->control_cmd.accelerator_control_cmd);
            //  Publish desired velocity if long_autonomy_enabled
            } else if (long_autonomy_enabled) {
                this->publishDesiredAcceleration(this->control_cmd.desired_accel_cmd);
            // Publish joystck commands if long_autonomy_enable is false and no braking is commanded
            } else {
                this->publishBrakingCmd(this->override_control_cmd.braking_control_cmd);
                this->publishAcceleratorCmd(this->override_control_cmd.accelerator_control_cmd);
            }

            this->control_pub->publish(this->control_cmd);

            this->setNodeStatus(node_status);
            //      TODO long_control is in charge of these, at the moment
            //      this->publishAcceleratorCmd(msg->accelerator_control_cmd);
            //      this->publishBrakingCmd(msg->braking_control_cmd);
            //      this->publishGearCmd(msg->gear_control_cmd);

        } catch (...) {
            //      TODO handle critical error in the safest way
            RCLCPP_ERROR(this->get_logger(),
                         "ControlSafetyLayerNode caught an exception, enabling emergency lane control");
            // Notify the SystemStatusManager of the change.
            // Proceed in emergency mode from the next iteration
            this->emergency_lane_enabled = true;
            this->setNodeStatus(common::NodeStatusCode::NODE_FATAL_ERROR);
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
        common::msgs::ControlAcceleratorCmd msg) const {
    //  TODO implement safety checks
    if (this->accelerator_control_pub) {
        if (msg.data > this->throttle_cmd_max)
            msg.data = this->throttle_cmd_max;
        this->accelerator_control_pub->publish(msg);

        return true; // OK
    }
    return false;
}

bool nif::control::ControlSafetyLayerNode::publishBrakingCmd(
        common::msgs::ControlBrakingCmd msg) const {

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

bool nif::control::ControlSafetyLayerNode::publishDesiredAcceleration(std_msgs::msg::Float32 msg) const
{
    //  TODO implement safety checks
    if (this->desired_acceleration_pub) {
        if (msg.data > this->desired_acceleration_cmd_max)
            msg.data = this->desired_acceleration_cmd_max;
        this->desired_acceleration_pub->publish(msg);

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
//                    this->lat_autonomy_enabled = param.as_bool();
                    result.successful = false;
                }
            }
        } else if (param.get_name() == "long_autonomy_enabled") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                if (true) {
//                    this->long_autonomy_enabled = param.as_bool();
                    result.successful = false;
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
