//
// Created by usrg on 9/9/21.
//

#ifndef ROS2MASTER_OVERRIDE_DEVICE_INTERFACE_H
#define ROS2MASTER_OVERRIDE_DEVICE_INTERFACE_H

#include "rclcpp/rclcpp.hpp"

#include "nif_common/types.h"

namespace nif {
namespace control {

class OverrideDeviceInterfaceNode : public rclcpp::Node {
public:
  OverrideDeviceInterfaceNode(const std::string & node_name,
                              const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node(node_name, options) {

    this->override_msg_sub =
            this->create_subscription<nif::common::msgs::OverrideControlCmd>(
            "/joystick/command", nif::common::constants::QOS_CONTROL_CMD_OVERRIDE,
            std::bind(&OverrideDeviceInterfaceNode::joystickCallback, this,
                      std::placeholders::_1));

    this->nif_control_cmd_pub =
        this->create_publisher<nif::common::msgs::ControlCmd>(
            "control_pool/override_cmd", nif::common::constants::QOS_CONTROL_CMD_OVERRIDE);
  }

private:
  nif::common::msgs::ControlCmd control_cmd;

  rclcpp::Subscription<nif::common::msgs::OverrideControlCmd>::SharedPtr override_msg_sub;
  rclcpp::Publisher<nif::common::msgs::ControlCmd>::SharedPtr nif_control_cmd_pub;

  void joystickCallback(
          const nif::common::msgs::OverrideControlCmd::SharedPtr msg) {
    this->control_cmd.header.stamp = msg->stamp;
    this->control_cmd.order = 0;
    this->control_cmd.steering_control_cmd.data = msg->steering_cmd;
    this->control_cmd.accelerator_control_cmd.data = msg->accelerator_cmd;
    this->control_cmd.braking_control_cmd.data = msg->brake_cmd;
    this->control_cmd.gear_control_cmd.data = msg->gear_cmd;

    this->nif_control_cmd_pub->publish(this->control_cmd);
  }
};
} // namespace control
} // namespace nif
#endif // ROS2MASTER_OVERRIDE_DEVICE_INTERFACE_H
