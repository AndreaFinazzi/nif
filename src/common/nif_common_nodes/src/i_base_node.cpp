//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_common_nodes/i_base_node.h"

using namespace nif::common;

IBaseNode::IBaseNode() : Node("no_name_node") {
  throw std::invalid_argument("Cannot construct IBaseNode without specifying "
                              "node_name. Creating empty node.");
}

IBaseNode::IBaseNode(const std::string& node_name)
  : IBaseNode(node_name, rclcpp::NodeOptions{}) {}

IBaseNode::IBaseNode(const std::string& node_name,
                     const rclcpp::NodeOptions& options)
  : Node(node_name, options) {
  //  Initialize timers
  gclock_node_init = this->now();
  gclock_current = gclock_node_init;

  //  Declare subscriptions
  //                TODO : Define QoS macros
  this->raptor_state_sub =
      this->create_subscription<nif::common::msgs::RaptorState>(
          "topic_raptor_state",
          nif::common::constants::QOS_DEFAULT,
          std::bind(
              &IBaseNode::raptorStateCallback, this, std::placeholders::_1));
  this->race_control_state_sub =
      this->create_subscription<nif::common::msgs::RaceControlState>(
          "topic_race_control_state",
          nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::raceControlStateCallback,
                    this,
                    std::placeholders::_1));
  this->ego_localization_state_sub =
      this->create_subscription<nav_msgs::msg::Odometry>(
          "topic_localization_state",
          nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoLocalizationCallback,
                    this,
                    std::placeholders::_1));
  this->ego_powertrain_state_sub =
      this->create_subscription<nif::common::msgs::PowertrainState>(
          "topic_powertrain_state",
          nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoVehiclePowertrainCallback,
                    this,
                    std::placeholders::_1));

  //  TODO Declare node_state_pub to notify the node state
  //
  //
  //
}

/**
 * TODO implement callback
 * @param msg
 */
void IBaseNode::egoVehiclePowertrainCallback(
    const nif::common::msgs::PowertrainState::SharedPtr msg) {
  this->ego_powertrain_state = *msg;
}

void IBaseNode::egoLocalizationCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  this->ego_localization_state = *msg;
}

/**
 * TODO implement callback
 * @param msg
 */
void IBaseNode::raptorStateCallback(
    const nif::common::msgs::RaptorState::SharedPtr msg) {
  this->raptor_state = *msg;
}

/**
 * TODO implement callback
 * @param msg
 */
void IBaseNode::raceControlStateCallback(
    const nif::common::msgs::RaceControlState::SharedPtr msg) {
  this->race_control_state = *msg;
}
