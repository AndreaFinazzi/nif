//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Calvin Chanyoung Jung

//
// Created by usrg on 11/17/21.
//

#include "nif_common/constants.h"
#include "nif_opponent_prediction_nodes/frenet_based_opponent_predictor.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"

int32_t main(int32_t argc, char** argv) {
  rclcpp::init(argc, argv);

  using nif::perception::FrenetBasedOpponentPredictor;
  using namespace nif::common::constants;

  const char* node_name = "opponent_predictor_node";

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                "Instantiating FrenetBasedOpponentPredictor with name: %s",
                node_name);
    rclcpp::NodeOptions options;

    nd = std::make_shared<FrenetBasedOpponentPredictor>(node_name);
  } catch (std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [FrenetBasedOpponentPredictor]",
              node_name);

  return 0;
}