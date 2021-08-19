//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_common_nodes/i_base_node.h"
#include "nif_common/constants.h"

using namespace nif::common;
using nif::common::NodeStatusCode;

IBaseNode::IBaseNode() : Node("no_name_node"), node_status_manager(*this, nif::common::NodeType::SYSTEM)
{
  this->node_status_manager.update(NodeStatusCode::NODE_FATAL_ERROR);
  throw std::invalid_argument("Cannot construct IBaseNode without specifying "
                              "node_name. Creating empty node.");
}

IBaseNode::IBaseNode(const std::string &node_name)
    : IBaseNode(node_name, NodeType::SYSTEM) {
          RCLCPP_WARN(this->get_logger(), "CALL TO IBaseNode DEPRECATED CONSTRUCTOR, SPECIFY NODE TYPE.");
}

IBaseNode::IBaseNode(const std::string &node_name, const NodeType node_type, const rclcpp::NodeOptions &options)
    : Node(node_name, options),
      node_status_manager(*this, node_type) {
  //  Initialize timers
  gclock_node_init = this->now();

// Get global parameters
// TODO improve readability
  try {
    // Initialize client and wait for service
    this->global_parameters_client =
            std::make_shared<rclcpp::SyncParametersClient>(this, constants::parameters::GLOBAL_PARAMETERS_NODE_NAME);
    this->global_parameters_client->wait_for_service(constants::parameters::GLOBAL_PARAMETERS_NODE_TIMEOUT);

        this->body_frame_id = this->get_global_parameter<std::string>(
          constants::parameters::names::FRAME_ID_BODY);

        this->global_frame_id = this->get_global_parameter<std::string>(
          constants::parameters::names::FRAME_ID_GLOBAL);

  } catch (std::exception & e) {
    // Something else happened, fall back to default values.
    RCLCPP_ERROR(this->get_logger(), "SEVERE PARAMETERS ERROR. Falling back to default values, proceding may be unsafe.");
    RCLCPP_ERROR(this->get_logger(), "What: %s", e.what());

    this->node_status_manager.update(NodeStatusCode::NODE_ERROR);

//  Check if available as local parameters:
    this->body_frame_id = this->get_parameter(constants::parameters::names::FRAME_ID_BODY).as_string();
    this->global_frame_id = this->get_parameter(constants::parameters::names::FRAME_ID_GLOBAL).as_string();

  }

  //  Declare subscriptions
  //                TODO : Define QoS macros

  const std::string& topic_ego_odometry =
      this->get_global_parameter<std::string>(
          constants::parameters::names::TOPIC_ID_EGO_ODOMETRY);

  const std::string& topic_system_status =
      this->get_global_parameter<std::string>(
          constants::parameters::names::TOPIC_ID_SYSTEM_STATUS);

  const std::string& topic_race_control_status =
      this->get_global_parameter<std::string>(
          constants::parameters::names::TOPIC_ID_RACE_CONTROL_STATUS);

  const std::string& topic_ego_powertrain_status =
      this->get_global_parameter<std::string>(
          constants::parameters::names::TOPIC_ID_EGO_POWERTRAIN_STATUS);

  this->ego_odometry_sub =
      this->create_subscription<nif::common::msgs::Odometry>(
          topic_ego_odometry, nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoOdometryCallback, this,
                    std::placeholders::_1));

  this->system_status_sub =
      this->create_subscription<nif::common::msgs::SystemStatus>(
          topic_system_status, nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::systemStatusCallback, this,
                    std::placeholders::_1));

  this->race_control_status_sub =
      this->create_subscription<nif::common::msgs::RaceControlStatus>(
          topic_race_control_status, nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::raceControlStatusCallback, this,
                    std::placeholders::_1));

  this->ego_powertrain_state_sub =
      this->create_subscription<nif::common::msgs::PowertrainState>(
          topic_ego_powertrain_status, nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoPowertrainCallback, this,
                    std::placeholders::_1));


  this->node_status_manager.update(NodeStatusCode::NODE_INITIALIZED);
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
void IBaseNode::systemStatusCallback(
    const nif::common::msgs::SystemStatus::SharedPtr msg) {
  this->system_state = *msg;
}

/**
 * TODO implement callback
 * @param msg
 */
void IBaseNode::raceControlStatusCallback(
    const nif::common::msgs::RaceControlStatus::SharedPtr msg) {
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
const msgs::SystemStatus &IBaseNode::getSystemState() const {
  return system_state;
}
const msgs::RaceControlStatus &IBaseNode::getRaceControlState() const {
  return race_control_state;
}
