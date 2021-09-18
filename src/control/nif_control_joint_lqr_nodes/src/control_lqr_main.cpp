//
// Created by usrg on 7/17/21.
//
//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

#include <memory>

#include "nif_common/constants.h"
#include "nif_control_joint_lqr_nodes/control_lqr_node.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

  using nif::control::ControlLQRNode;
  using namespace nif::common::constants;

  const char *node_name = "control_joint_lqr_node";

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating ControlLQRNode with name: %s", node_name);
    rclcpp::NodeOptions options;

    nd = std::make_shared<ControlLQRNode>(node_name);

  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [ControlLQRNode]", node_name);

  return 0;
}