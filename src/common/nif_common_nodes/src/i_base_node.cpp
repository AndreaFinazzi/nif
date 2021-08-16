//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_common_nodes/i_base_node.h"

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
            std::make_shared<rclcpp::SyncParametersClient>(this, strcat("/", constants::parameters::GLOBAL_PARAMETERS_NODE_NAME));
    this->global_parameters_client->wait_for_service(constants::parameters::GLOBAL_PARAMETERS_NODE_TIMEOUT);

    if (this->global_parameters_client->has_parameter(constants::parameters::NAME_BODY_FRAME_ID)) {
    // Try to get parameters
        this->body_frame_id = this->global_parameters_client->get_parameter<std::string>(
          std::string(constants::parameters::NAME_BODY_FRAME_ID));
    
    } else {
      RCLCPP_INFO(this->get_logger(), "Declaring global parameter %s", constants::parameters::NAME_BODY_FRAME_ID);
      this->global_parameters_client->set_parameters({
          rclcpp::Parameter(
            std::string(constants::parameters::NAME_BODY_FRAME_ID),
            constants::parameters::VALUE_BODY_FRAME_ID)
      });

      this->body_frame_id = this->global_parameters_client->get_parameter<std::string>(
        std::string(constants::parameters::NAME_BODY_FRAME_ID));
    }

    if (this->global_parameters_client->has_parameter(constants::parameters::NAME_GLOBAL_FRAME_ID)) {
    // Try to get parameters
        this->global_frame_id = this->global_parameters_client->get_parameter<std::string>(
          std::string(constants::parameters::NAME_GLOBAL_FRAME_ID));
    
    } else {
      RCLCPP_INFO(this->get_logger(), "Declaring global parameter %s", constants::parameters::NAME_GLOBAL_FRAME_ID);
      this->global_parameters_client->set_parameters({
        rclcpp::Parameter(
          std::string(constants::parameters::NAME_GLOBAL_FRAME_ID),
          constants::parameters::VALUE_GLOBAL_FRAME_ID)
      });
      
      this->global_frame_id = this->global_parameters_client->get_parameter<std::string>(
          std::string(constants::parameters::NAME_GLOBAL_FRAME_ID));
    }

  // } catch (rclcpp::exceptions::ParameterNotDeclaredException & e) {
  //   // If global params are not available, declares its own params with default values.
  //   RCLCPP_WARN(this->get_logger(), "Couldn't get_parameter() from GlobalParameterNode. Falling back to defaults.");
  //   RCLCPP_WARN(this->get_logger(), e.what());
    
  //   this->declare_parameter(constants::parameters::NAME_BODY_FRAME_ID, constants::parameters::VALUE_BODY_FRAME_ID);
  //   this->declare_parameter(constants::parameters::NAME_BODY_FRAME_ID, constants::parameters::VALUE_GLOBAL_FRAME_ID);

  //   this->body_frame_id = this->get_parameter(constants::parameters::NAME_BODY_FRAME_ID).as_string();
  //   this->global_frame_id = this->get_parameter(constants::parameters::NAME_BODY_FRAME_ID).as_string();

  } catch (std::exception & e) {
    // Something else happened, fall back to default values.
    RCLCPP_ERROR(this->get_logger(), "SEVERE PARAMETERS ERROR. Falling back to default values, proceding may be unsafe.");
    RCLCPP_ERROR(this->get_logger(), e.what());

    this->node_status_manager.update(NodeStatusCode::NODE_ERROR);

    this->body_frame_id = this->get_parameter(constants::parameters::NAME_BODY_FRAME_ID).as_string();
    this->global_frame_id = this->get_parameter(constants::parameters::NAME_BODY_FRAME_ID).as_string();

  }

  //  Declare subscriptions
  //                TODO : Define QoS macros
  this->ego_odometry_sub =
      this->create_subscription<nif::common::msgs::Odometry>(
          "topic_ego_odometry", nif::common::constants::QOS_DEFAULT,
          std::bind(&IBaseNode::egoOdometryCallback, this,
                    std::placeholders::_1));

  this->system_state_sub =
      this->create_subscription<nif::common::msgs::SystemStatus>(
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
void IBaseNode::systemStateCallback(
    const nif::common::msgs::SystemStatus::SharedPtr msg) {
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
const msgs::SystemStatus &IBaseNode::getSystemState() const {
  return system_state;
}
const msgs::RaceControlState &IBaseNode::getRaceControlState() const {
  return race_control_state;
}
