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

    this->nif_control_cmd_pub = this->create_publisher<nif::common::msgs::ControlCmd>("control_pool/override_cmd", 10);
    std::weak_ptr<std::remove_pointer<decltype(this->nif_control_cmd_pub.get())>::type> captured_pub = this->nif_control_cmd_pub;

    // Publish a converted message
    auto callback = [this, captured_pub](nif::common::msgs::OverrideControlCmd::SharedPtr msg) -> void {
        auto pub_ptr = captured_pub.lock();
        if (!pub_ptr) {
            return;
        }
        nif::common::msgs::ControlCmd::UniquePtr control_cmd_ptr(new nif::common::msgs::ControlCmd());
        control_cmd_ptr->header.stamp = msg->stamp;
        control_cmd_ptr->order = 0;
        control_cmd_ptr->steering_control_cmd.data = msg->steering_cmd;
        control_cmd_ptr->accelerator_control_cmd.data = msg->accelerator_cmd;
        control_cmd_ptr->braking_control_cmd.data = msg->brake_cmd;
        control_cmd_ptr->gear_control_cmd.data = msg->gear_cmd;

        pub_ptr->publish(std::move(control_cmd_ptr));
    };

    this->override_msg_sub =
            this->create_subscription<nif::common::msgs::OverrideControlCmd>(
                    "/joystick/command", nif::common::constants::QOS_CONTROL_CMD_OVERRIDE,
                    callback);

//    this->override_msg_sub =
//            this->create_subscription<nif::common::msgs::OverrideControlCmd>(
//                    "/joystick/command", nif::common::constants::QOS_CONTROL_CMD_OVERRIDE,
//                    std::bind(&OverrideDeviceInterfaceNode::joystickCallback, this,
//                              std::placeholders::_1));
//
//    this->nif_control_cmd_pub =
//        this->create_publisher<nif::common::msgs::ControlCmd>(
//            "control_pool/override_cmd", nif::common::constants::QOS_CONTROL_CMD_OVERRIDE);
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
