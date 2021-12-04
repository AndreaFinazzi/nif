//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by USRG on 9/14/21.
//
#include "ekf_localizer/nif_aw_localization_node.h"

#include "rcutils/error_handling.h"
#include <cstdio>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

int32_t main(int32_t argc, char **argv) {
  rclcpp::init(argc, argv);

  // using namespace nif::common::constants;
  // using namespace nif::perception;

  const char *node_name = "nif_aw_localization_nodes";

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger("nif_aw_localization_nodes"),
                "Instantiating AWLocalizationNode with name: %s", &node_name);
    rclcpp::NodeOptions options;

    nd = std::make_shared<AWLocalizationNode>(node_name);

  } catch (std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("nif_aw_localization_nodes"),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }
  
  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger("nif_aw_localization_nodes"),
              "Shutting down %s [localization_aw_node]", &node_name);

  return 0;
}