//
// Created by usrg on 7/11/21.
//

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//
#include <memory>

#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"
#include "rcutils/error_handling.h"

int32_t main(int32_t argc, char **argv) 
{
  rclcpp::init(argc, argv);

  using nif::control::ControlSafetyLayerNode;
  using namespace nif::common::constants;

  const char* node_name = "control_safety_layer_node";
  const std::chrono::microseconds sync_period(10000); //  10ms

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating ControlSafetyLayerNode with name: %s; sync_period: %d", node_name, sync_period);
    rclcpp::NodeOptions options;

    nd = std::make_shared<ControlSafetyLayerNode>(
        node_name,
        nif::common::utils::numeric::clip(SYNC_PERIOD_MIN, SYNC_PERIOD_MAX, sync_period));

  } catch (std::range_error & e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "Bad initialization of node: %s. Initializing with SYNC_PERIOD_DEFAULT...\n%s", e.what());

//  Initialize with default period.
//  TODO should we abort in these circumstances?
    nd = std::make_shared<ControlSafetyLayerNode>(
        node_name,
        SYNC_PERIOD_DEFAULT);

  } catch (std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s", e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [ControlSafetyLayerNode]", node_name);

  return 0;
}