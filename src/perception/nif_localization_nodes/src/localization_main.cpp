//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#include "localization_ekf_nodes/load_geofence_node.h"
#include "localization_ekf_nodes/localization_ekf_node.h"
#include "rcutils/error_handling.h"
#include <cstdio>
#include <iostream>
#include <memory>

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

  using namespace nif::common::constants;
  using namespace nif::localization::ekf;
  using namespace nif::localization::geofence;

  //  const char* node_name_1 = "localization_node";
  const char *node_name_1 = "localization_ekf_node";
  const char *node_name_2 = "localization_geofence_node";

  auto PC_FILTER = std::make_shared<EKFLocalizer>(node_name_1);
  auto EGO_FILTER = std::make_shared<GeoFenceLoader>(node_name_2);

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::executor::create_default_executor_arguments(), 2);

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating LocalizationNode with name: %s", &node_name_1);
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating LocalizationNode with name: %s", &node_name_2);

    executor.add_node(PC_FILTER);
    executor.add_node(EGO_FILTER);

  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }
  executor.spin();
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [LocalizationNode]", node_name_1);
  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [LocalizationNode]", node_name_2);

  return 0;
}