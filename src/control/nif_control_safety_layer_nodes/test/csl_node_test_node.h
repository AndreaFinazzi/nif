//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

//
// Created by usrg on 7/10/21.
//

#include "nif_control_common_nodes/i_controller_node.h"
#include <python3.8/grammar.h>

class MockControlNode : public nif::control::IControllerNode {
public:
//  using nif::control::IControllerNode::IControllerNode;
  explicit MockControlNode(const std::string& node_name) : IControllerNode(node_name) {
    this->control_command = std::make_shared<nif::common::msgs::ControlCmd>();

    this->csl_cmd_sub = this->create_subscription<nif::common::msgs::ControlCmd>(
        "csl_control_cmd", nif::common::constants::QOS_CONTROL_CMD, std::bind(&MockControlNode::controlCallback, this, std::placeholders::_1)
        );
  }

  void stateReport() {
    RCLCPP_INFO(this->get_logger(), "ego_odometry: %s", (this->getEgoOdometry().header.stamp));
    RCLCPP_INFO(this->get_logger(),
                "race_control_state: %s",
                (this->getRaceControlState().track_cond));
    RCLCPP_INFO(this->get_logger(), "system_state: %s", this->getSystemStatus().health_status.is_system_healthy);
  }

  std::shared_ptr<rclcpp::Subscription<nif::common::msgs::ControlCmd>> csl_cmd_sub;
  int passed_controls_counter = 0;
  rclcpp::Time last_control_stamp{};

private:
  rclcpp::Parameter param_one;
  rclcpp::Parameter param_two;

  void initParameters() override {}

  void getParameters() override {}

  nif::common::msgs::ControlCmd::SharedPtr solve() override {
    this->control_command->header.stamp = this->now();
    this->control_command->header.frame_id = this->getBodyFrameId();
    this->control_command->steering_control_cmd.data = this->now().nanoseconds() % 10000000;
    this->control_command->gear_control_cmd.data = 1;
    return this->control_command;
  }

  void controlCallback(const nif::common::msgs::ControlCmd::SharedPtr msg) {
    passed_controls_counter++;
    last_control_stamp = msg->header.stamp;
  }


  nif::common::msgs::ControlCmd::SharedPtr control_command;

};
