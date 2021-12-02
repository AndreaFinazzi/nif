//
// Created by usrg on 8/31/21.
//
//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

#include <memory>

#include "../include/adaptive_cruise_node.hpp"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"

int32_t main(int32_t argc, char **argv)
{
  rclcpp::init(argc, argv);

  using namespace nif::common::constants;
  using namespace nif::control;

  const char *node_name = "nif_idm_based_acc_node";

  rclcpp::Node::SharedPtr nd;

  try
  {
    RCLCPP_INFO(
        rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
        "Instantiating IDMACCNode with name: %s",
        node_name);
    rclcpp::NodeOptions options;

    nd = std::make_shared<IDMACCNode>(node_name);
  }
  catch (std::exception &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [IDMACCNode]", node_name);

  return 0;
}