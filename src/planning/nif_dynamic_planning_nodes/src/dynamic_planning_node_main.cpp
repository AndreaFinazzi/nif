//
// Created by usrg on 12/11/21.
//

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Chanyoung Jung

//
// Created by usrg on 12/11/21.
//
#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_dynamic_planning_nodes/dynamic_planning_node_v2.h"
#include "nif_utils/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include <memory>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  using namespace nif::common::constants;
  using namespace nif::planning;

  const char *node_name = "dynamic_planning_node";

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating DynamicPlannerNode with name: %s;", node_name);

    nd = std::make_shared<DynamicPlannerNode>(node_name);
  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [DynamicPlannerNode]", node_name);

  return 0;
}