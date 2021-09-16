//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#include "nif_common_nodes/i_base_node.h"
#include "nif_common/constants.h"

using namespace nif::common;
using nif::common::NodeStatusCode;

IBaseNode::IBaseNode() : Node("no_name_node"),
                         node_status_manager(*this, nif::common::NodeType::SYSTEM)
{
  this->setNodeStatus(NodeStatusCode::NODE_FATAL_ERROR);
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

        this->node_status_timer_period_us = nif::common::types::t_clock_period_us(
            this->get_global_parameter<int>(
              constants::parameters::names::PERIOD_NODE_STATUS_CLOCK_US));

  } catch (std::exception & e) {
    // Something else happened, fall back to default values.
    RCLCPP_ERROR(this->get_logger(), "SEVERE PARAMETERS ERROR. Falling back to default values, proceding may be unsafe.");
    RCLCPP_ERROR(this->get_logger(), "What: %s", e.what());

    this->setNodeStatus(NodeStatusCode::NODE_ERROR);

//  Check if available as local parameters:
    this->body_frame_id = this->get_parameter(constants::parameters::names::FRAME_ID_BODY).as_string();
    this->global_frame_id = this->get_parameter(constants::parameters::names::FRAME_ID_GLOBAL).as_string();

  }

//  Declare set parameters callback
//  TODO not ready yet. When enabled, each parameter should be handled in the callback.
//  this->callback_handle = this->add_on_set_parameters_callback(
//      std::bind(&IBaseNode::parametersSetCallback, this, std::placeholders::_1));

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
          topic_ego_odometry, nif::common::constants::QOS_EGO_ODOMETRY,
          std::bind(&IBaseNode::egoOdometryCallback, this,
                    std::placeholders::_1));

  this->system_status_sub =
      this->create_subscription<nif::common::msgs::SystemStatus>(
          topic_system_status, nif::common::constants::QOS_INTERNAL_STATUS,
          std::bind(&IBaseNode::systemStatusCallback, this,
                    std::placeholders::_1));

  this->race_control_status_sub =
      this->create_subscription<nif::common::msgs::RaceControlStatus>(
          topic_race_control_status, nif::common::constants::QOS_INTERNAL_STATUS,
          std::bind(&IBaseNode::raceControlStatusCallback, this,
                    std::placeholders::_1));

  this->ego_powertrain_state_sub =
      this->create_subscription<nif::common::msgs::PowertrainState>(
          topic_ego_powertrain_status, nif::common::constants::QOS_INTERNAL_STATUS,
          std::bind(&IBaseNode::egoPowertrainCallback, this,
                    std::placeholders::_1));


  if (this->node_status_timer_period_us >= nif::common::constants::SYNC_PERIOD_MIN &&
  this->node_status_timer_period_us <= nif::common::constants::SYNC_PERIOD_MAX) {

    this->node_status_timer = this->create_wall_timer(
        this->node_status_timer_period_us,
        std::bind(&IBaseNode::nodeStatusTimerCallback, this));

  } else {
    //    Not allowed to instantiate with this period
    throw std::range_error("Sync Period out of range.");
  }

  this->node_status_pub = this->create_publisher<nif::common::msgs::NodeStatus>(
      this->getNodeStatusTopicName(), nif::common::constants::QOS_INTERNAL_STATUS);

  // TODO make global parameter
  this->register_node_service_client =
      this->create_client<nif_msgs::srv::RegisterNodeStatus>("/system_status_manager/register");

  auto request = std::make_shared<nif_msgs::srv::RegisterNodeStatus::Request>();
  request->node_name = this->get_name();
  request->status_topic_name = getNodeStatusTopicName();
  request->node_type = this->node_status_manager.getNodeType();

  using ServiceResponseFuture =
      rclcpp::Client<nif_msgs::srv::RegisterNodeStatus>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
    const auto& response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response from system_status_manager/register service. Success: %s", response->success ? "true" : "false");
    if (!response->success)
    {
      RCLCPP_ERROR(this->get_logger(), "Couldn't register to System Status Manager.");
      throw std::runtime_error("Couldn't register to System Status Manager. Not safe to proceed.");
    } else {
        this->node_status_manager.setNodeId(response->node_id);
    }
  };
  this->register_node_service_client->wait_for_service(std::chrono::seconds(2));
  if (this->register_node_service_client->service_is_ready())
    auto future_result = this->register_node_service_client->async_send_request(request, response_received_callback);
  else
    throw std::runtime_error("System Status Manager not ready, proceeding is not safe.");

  this->setNodeStatus(NodeStatusCode::NODE_INITIALIZED);
}


void IBaseNode::egoPowertrainCallback(
    const nif::common::msgs::PowertrainState::SharedPtr msg) {
  has_ego_powertrain_state = true;
  this->ego_powertrain_state_update_time = this->now();
  this->ego_powertrain_state = *msg;
  this->afterEgoPowertrainCallback();
}

void IBaseNode::egoOdometryCallback(
    const nif::common::msgs::Odometry::SharedPtr msg) {
  has_ego_odometry = true;
  this->ego_odometry_update_time = this->now();
  this->ego_odometry = *msg;
  this->afterEgoOdometryCallback();
}

void IBaseNode::systemStatusCallback(
    const nif::common::msgs::SystemStatus::SharedPtr msg) {
  has_system_status = true;
  this->system_status_update_time = this->now();
  this->system_status = *msg;
  this->afterSystemStatusCallback();
}

void IBaseNode::raceControlStatusCallback(
    const nif::common::msgs::RaceControlStatus::SharedPtr msg) {
  has_race_control_status   = true;
  this->race_control_status_update_time = this->now();
  this->race_control_status = *msg;
  this->afterRaceControlStatusCallback();
}

void IBaseNode::nodeStatusTimerCallback() {
  auto msg = this->node_status_manager.getNodeStatus();
  msg.stamp = this->now();

  this->node_status_pub->publish(msg);
}

//  ### NODE STATUS COMPONENTS
void IBaseNode::setNodeStatus(NodeStatusCode status_code) noexcept {
  this->node_status_manager.update(status_code);
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
const msgs::RaceControlStatus &IBaseNode::getRaceControlState() const {
  return race_control_status;
}
const msgs::SystemStatus &IBaseNode::getSystemStatus() const {
  return system_status;
}
const msgs::RaceControlStatus &IBaseNode::getRaceControlStatus() const {
  return race_control_status;
}
const rclcpp::Time &IBaseNode::getEgoOdometryUpdateTime() const {
  return ego_odometry_update_time;
}
const rclcpp::Time &IBaseNode::getEgoPowertrainStateUpdateTime() const {
  return ego_powertrain_state_update_time;
}
const rclcpp::Time &IBaseNode::getSystemStatusUpdateTime() const {
  return system_status_update_time;
}
const rclcpp::Time &IBaseNode::getRaceControlStatusUpdateTime() const {
  return race_control_status_update_time;
}
bool IBaseNode::hasEgoOdometry() const { return has_ego_odometry; }
bool IBaseNode::hasEgoPowertrainState() const { return has_ego_powertrain_state; }
bool IBaseNode::hasSystemStatus() const { return has_system_status; }
bool IBaseNode::hasRaceControlStatus() const { return has_race_control_status; }

