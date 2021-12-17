//
// Created by usrg on 7/11/21.
//

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//
#include "nif_objects_tracker_nodes/objects_tracker_nodes.h"
#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_utils/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include <memory>

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

  using namespace nif::common::constants;

  const char *node_name = "objects_tracking_node";

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating IMMObjectTrackerNode with name: %s;", node_name);

    nd = std::make_shared<IMMObjectTrackerNode>(node_name);
  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [IMMObjectTrackerNode]");

  return 0;
}