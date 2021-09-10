//
// Created by usrg on 7/11/21.
//

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/23/21.
//
#include <memory>
#include <nif_control_safety_layer_nodes/override_device_interface_node.h>

#include "nif_common/constants.h"
#include "nif_control_safety_layer_nodes/control_safety_layer_node.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"

int32_t main(int32_t argc, char **argv) 
{
  rclcpp::init(argc, argv);

  using nif::control::ControlSafetyLayerNode;
  using nif::control::OverrideDeviceInterfaceNode;
  using namespace nif::common::constants;

  const char* node_name_csl = "control_safety_layer_node";
  const char* node_name_override = "override_device_interface_node";
  const std::chrono::microseconds sync_period(10000); //  10ms

  rclcpp::Node::SharedPtr csl_node;
  rclcpp::Node::SharedPtr override_node;

  rclcpp::executors::SingleThreadedExecutor executor;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating ControlSafetyLayerNode with name: %s; sync_period: %d",
        node_name_csl, sync_period);
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Starting OverrideDeviceInterfaceNode in the same process.");

    csl_node = std::make_shared<ControlSafetyLayerNode>(
        node_name_csl,
        nif::common::utils::numeric::clip(SYNC_PERIOD_MIN, SYNC_PERIOD_MAX, sync_period),
        rclcpp::NodeOptions().use_intra_process_comms(true));

    override_node = std::make_shared<OverrideDeviceInterfaceNode>(
        node_name_override,
        rclcpp::NodeOptions().use_intra_process_comms(true));

    executor.add_node(csl_node);
    executor.add_node(override_node);

  } catch (std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s", e.what());
    return -1;
  }

  executor.spin();

  executor.remove_node(csl_node);
  executor.remove_node(override_node);

  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [ControlSafetyLayerNode]", node_name_csl);

  return 0;
}