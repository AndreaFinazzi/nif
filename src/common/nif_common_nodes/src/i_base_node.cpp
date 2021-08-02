//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_common_nodes/i_base_node.h"

using namespace nif::common;

IBaseNode::IBaseNode() : Node("no_name_node"), node_status_manager(*this, nif::common::NodeType::SYSTEM)
{
  throw std::invalid_argument("Cannot construct IBaseNode without specifying "
                              "node_name. Creating empty node.");
}

IBaseNode::IBaseNode(const std::string &node_name)
    : IBaseNode(node_name, rclcpp::NodeOptions{}) {}

IBaseNode::IBaseNode(const std::string &node_name,
                     const rclcpp::NodeOptions &options)
    : Node(node_name, options),
      node_status_manager(*this, nif::common::NodeType::SYSTEM) {
  //  Initialize timers
  gclock_node_init = this->now();

  this->declare_parameter("body_frame_id", "base_link");
  this->declare_parameter("global_frame_id", "odom");

  this->body_frame_id = this->get_parameter("body_frame_id").as_string();
  this->global_frame_id = this->get_parameter("global_frame_id").as_string();

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
  this->ego_powertrain_state_sub =
      this->create_subscription<nif::common::msgs::PowertrainState>(
          "topic_powertrain_state", nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoPowertrainCallback, this,
                    std::placeholders::_1));


  this->node_status_manager.update(nif::common::NodeStatusCode::INITIALIZED);
  //  TODO Declare node_state_pub to notify the node state
  //
  //
  //
}

/**
 * TODO implement callback
 * @param msg
 */
void IBaseNode::egoPowertrainCallback(
    const nif::common::msgs::PowertrainState::SharedPtr msg) {
  this->ego_powertrain_state = *msg;
}

/**
 *
 * @param msg
 */
void IBaseNode::egoOdometryCallback(
    const nif::common::msgs::Odometry::SharedPtr msg) {
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
const std::string &IBaseNode::getBodyFrameId() const {
  return body_frame_id;
}
const std::string &IBaseNode::getGlobalFrameId() const {
  return global_frame_id;
}
const rclcpp::Time &IBaseNode::getGclockNodeInit() const {
  return gclock_node_init;
}
const msgs::Odometry &IBaseNode::getEgoOdometry() const {
  return ego_odometry;
}
const msgs::PowertrainState &IBaseNode::getEgoPowertrainState() const {
  return ego_powertrain_state;
}
const msgs::SystemState &IBaseNode::getSystemState() const {
  return system_state;
}
const msgs::RaceControlState &IBaseNode::getRaceControlState() const {
  return race_control_state;
}
