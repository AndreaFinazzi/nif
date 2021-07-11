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

IBaseNode::IBaseNode(const std::string &node_name,
                     const rclcpp::NodeOptions &options)
    : Node(node_name, options) {
  //  Initialize timers
  gclock_node_init = this->now();
  gclock_current = gclock_node_init;

  //  Declare subscriptions
  //                TODO : Define QoS macros
  this->ego_odometry_sub =
      this->create_subscription<nif::common::msgs::Odometry>(
          "topic_ego_odometry", nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoOdometryCallback, this,
                    std::placeholders::_1));

  this->system_state_sub =
      this->create_subscription<nif::common::msgs::SystemState>(
          "topic_system_state", nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::systemStateCallback, this,
                    std::placeholders::_1));

  this->race_control_state_sub =
      this->create_subscription<nif::common::msgs::RaceControlState>(
          "topic_race_control_state", nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::raceControlStateCallback, this,
                    std::placeholders::_1));

  this->ego_powertrain_state =
      this->create_subscription<nif::common::msgs::PowertrainState>(
          "topic_powertrain_state", nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoPowertrainCallback, this,
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

/**
 *
 * @param msg
 */
void IBaseNode::egoOdometryCallback(
    const nif::common::msgs::Odometry::SharedPtr msg)
{

    this->ego_odometry = *msg;
}

/**
 * TODO implement callback
 * @param msg
 */
void IBaseNode::systemStateCallback(
    const nif::common::msgs::SystemState::SharedPtr msg) {
  this->system_state = *msg;
}

/**
 * TODO implement callback
 * @param msg
 */
void IBaseNode::raceControlStateCallback(
    const nif::common::msgs::RaceControlState::SharedPtr msg) {

  this->race_control_state = *msg;
}
