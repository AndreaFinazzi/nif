//
// Created by usrg on 12/11/21.
//

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Chanyoung Jung

//
// Created by usrg on 12/11/21.
//
#include <memory>

#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  const char* node_name = "waypoint_manager_node_mission_selective";

  rclcpp::Node::SharedPtr nd;

  // try {
  //   RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
  //               "Instantiating WaypointManagerMissionNode with name: %s;",
  //               node_name);

  //   nd = std::make_shared<WaypointManagerMissionNode>(node_name);
  //   // nd = std::make_shared<WaypointManagerMissionNode>(node_name);
  //   //    nd = std::make_shared<WaypointManagerNode>(node_name);
  // } catch (std::exception& e) {
  //   RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
  //                "FATAL ERROR during node initialization: ABORTING.\n%s",
  //                e.what());
  //   return -1;
  // }

  // rclcpp::spin(nd);
  // rclcpp::shutdown();

  // RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
  //             "Shutting down %s [WaypointManagerMissionNode]",
  //             node_name);

  return 0;
}