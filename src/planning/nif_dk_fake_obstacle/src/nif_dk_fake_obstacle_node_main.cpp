//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_dk_fake_obstacle/nif_dk_fake_obstacle_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include <cstdio>
#include <iostream>
#include <memory>

using namespace nif::common::constants;
using namespace nif::planning;

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

  const char *node_name_1 = "nif_dk_fake_obstacle_node";

  auto FAKE_OBS = std::make_shared<FakeObsNode>(node_name_1);

  rclcpp::executors::SingleThreadedExecutor executor;
  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating DKGraphPlannerNode with name: %s", &node_name_1);

    executor.add_node(FAKE_OBS);

  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  executor.spin();
  rclcpp::shutdown();


  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [DKGraphPlannerNode]", node_name_1);
  return 0;
}