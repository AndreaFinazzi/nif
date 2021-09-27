//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//

#include "localization_ekf_nodes/load_geofence_node.h"
#include "localization_ekf_nodes/localization_ekf_node.h"
#include "localization_ekf_nodes/resilient_localization_node.h"
#include "localization_ekf_nodes/load_globalmap_node.h"

#include "rcutils/error_handling.h"
#include <cstdio>
#include <iostream>
#include <memory>

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

  using namespace nif::common::constants;
  using namespace nif::localization::ekf;
  using namespace nif::localization::geofence;
  using namespace nif::localization::resilient;
  using namespace nif::localization::maploader;

  const char *node_name_1 = "localization_ekf_node";
  const char *node_name_2 = "localization_geofence_node";
  const char *node_name_3 = "localization_resilient_node";
  const char *node_name_4 = "localization_globalmap_node";

  auto EKF_NODE = std::make_shared<EKFLocalizer>(node_name_1);
  auto GEOFENCE_NODE = std::make_shared<GeoFenceLoader>(node_name_2);
  auto RESILIENT_NODE = std::make_shared<ResilientLocalization>(node_name_3);
  auto GLOBALMAP_NODE = std::make_shared<GlobalmapLoader>(node_name_4);

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::executor::create_default_executor_arguments(), 3);

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating LocalizationNode with name: %s", node_name_1);
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating LocalizationNode with name: %s", node_name_2);
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating LocalizationNode with name: %s", node_name_3);
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating LocalizationNode with name: %s", node_name_4);

    executor.add_node(EKF_NODE);
    executor.add_node(GEOFENCE_NODE);
    executor.add_node(RESILIENT_NODE);
    executor.add_node(GLOBALMAP_NODE);

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
  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [LocalizationNode]", node_name_3);
  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [LocalizationNode]", node_name_4);

  return 0;
}