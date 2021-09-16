//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

#include "nif_points_preprocessor_nodes/ego_shape_filter_node.h"
#include "nif_points_preprocessor_nodes/points_concat_node.h"

#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include <cstdio>
#include <iostream>
#include <memory>

using namespace nif::common::constants;
using namespace nif::perception;

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

//  const char *node_name_1 = "point_concat_filter";
  const char *node_name_2 = "ego_shape_filter";

//  auto PC_FILTER = std::make_shared<PointsConcatFilterNode>(node_name_1);
  auto EGO_FILTER = std::make_shared<EgoShapeFilterNode>(node_name_2);
  // Use 2 threads
  rclcpp::executors::SingleThreadedExecutor executor;
  try {
//    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
//                "Instantiating PointsConcatFilterNode with name: %s",
//                &node_name_1);
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating EgoShapeFilterNode with name: %s", &node_name_2);

//    executor.add_node(PC_FILTER);
    executor.add_node(EGO_FILTER);

  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  executor.spin();
  rclcpp::shutdown();

//  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
//              "Shutting down %s [PointsConcatFilterNode]", node_name_1);
  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [EgoShapeFilterNode]", node_name_2);
  return 0;
}