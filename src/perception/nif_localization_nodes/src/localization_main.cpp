//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#include "localization_ekf_nodes/localization_ekf_node.h"

#include "rcutils/error_handling.h"
#include <cstdio>
#include <iostream>
#include <memory>

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

  using namespace nif::common::constants;
  using namespace nif::localization::ekf;

  //  const char* node_name = "localization_node";
  const char *node_name = "localization_ekf_node";

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating LocalizationNode with name: %s", &node_name);
    //    nd = std::make_shared<LocalizationNode>(node_name);
    nd = std::make_shared<EKFLocalizer>(node_name);

  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [LocalizationNode]", node_name);

  return 0;
}