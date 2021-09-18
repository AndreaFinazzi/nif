//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

//
// Created by usrg on 7/10/21.
//

#include "nif_common_nodes/i_base_node.h"

class TestNode : public nif::common::IBaseNode {
public:
  TestNode() : IBaseNode("test_node", nif::common::NodeType::SYSTEM) {}

  void stateReport() {
    RCLCPP_INFO(this->get_logger(), "ego_odometry: %s", (this->getEgoOdometry().header.stamp));
    RCLCPP_INFO(this->get_logger(),
                "race_control_state: %s",
                (this->getRaceControlState().track_cond));
    RCLCPP_INFO(this->get_logger(), "system_failure: %s", this->getSystemStatus().health_status.system_failure);
  }

private:
  rclcpp::Parameter param_one;
  rclcpp::Parameter param_two;

  void initParameters() override {
    this->declare_parameter("param_one", "default");
    this->declare_parameter("param_two", -1);

    this->get_parameter("param_one", this->param_one);
    this->get_parameter("param_two", this->param_two);
  }

  void getParameters() override {}
};