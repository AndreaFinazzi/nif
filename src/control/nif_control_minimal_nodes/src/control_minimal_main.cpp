//
// Created by usrg on 7/17/21.
//
//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

#include <memory>

#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "nif_control_minimal_nodes/control_minimal_node.h"
#include "rcutils/error_handling.h"

int32_t main(int32_t argc, char **argv)
{
  rclcpp::init(argc, argv);

  using nif::control::ControlMinimalNode;
  using namespace nif::common::constants;

  const char* node_name = "control_minimal_node";

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating ControlMinimalNode with name: %s; sync_period: %d", node_name);
    rclcpp::NodeOptions options;

    nd = std::make_shared<ControlMinimalNode>(
        node_name);

  } catch (std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s", e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [ControlMinimalNode]", node_name);

  return 0;
}