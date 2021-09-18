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
#include "nif_system_status_manager_nodes/system_status_manager_node.h"
#include "nif_system_status_manager_nodes/rc_interface_node.h"
#include "rcutils/error_handling.h"

int32_t main(int32_t argc, char **argv) 
{
  rclcpp::init(argc, argv);

  using nif::system::SystemStatusManagerNode;
  using nif::RCInterfaceNode;
  using namespace nif::common::constants;

  const std::string node_name("system_status_manager_node");
  const std::string node_name_rci("rc_interface_node");
  rclcpp::Node::SharedPtr nd_ssm;
  rclcpp::Node::SharedPtr nd_rci;

  rclcpp::executors::SingleThreadedExecutor executor;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating SystemStatusManagerNode with name: %s;", node_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating RCInterfaceNode in the same process, with name: %s;", node_name_rci.c_str());

    nd_ssm = std::make_shared<SystemStatusManagerNode>(node_name);
    nd_rci = std::make_shared<RCInterfaceNode>(node_name_rci);

  } catch (std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s", e.what());
    return -1;
  }

  executor.add_node(nd_ssm);
  executor.add_node(nd_rci);
  executor.spin();

  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [SystemStatusManagerNode]", node_name.c_str());

  return 0;
}