//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

//
// Created by usrg on 7/10/21.
//

#include "nif_common_nodes/i_base_node.h"

class TestNode : public nif::common::IBaseNode {
public:
  TestNode() : IBaseNode("test_node", rclcpp::NodeOptions{}) {}

  void stateReport() {
    // RCLCPP_INFO(
    //     this->get_logger(), "ego_vehicle_state: %s",
    //     (this->ego_vehicle_state));
    // RCLCPP_INFO(this->get_logger(),
    //             "race_control_state: %s",
    //             (this->race_control_state));
    // RCLCPP_INFO(this->get_logger(), "system_state: %s",
    // (this->system_state));
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