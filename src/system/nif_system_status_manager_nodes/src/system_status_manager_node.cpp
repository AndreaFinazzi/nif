//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_system_status_manager_nodes/system_status_manager_node.h"

nif::system::SystemStatusManagerNode::SystemStatusManagerNode(
    const std::string &node_name)
    : Node(node_name)
{
  // Subscribers

  //  Publishers
  this->system_status_pub = this->create_publisher<nif::common::msgs::SystemStatus>(
      "system_status", nif::common::constants::QOS_INTERNAL_STATUS
      );

  // Services
  // TODO make global parameter
  this->register_node_service = this->create_service<nif_msgs::srv::RegisterNodeStatus>(
      "/system_status_manager/register",
      std::bind(
          &SystemStatusManagerNode::registerNodeServiceHandler,
          this,
          std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
      );

  this->system_status_timer = this->create_wall_timer(
      nif::common::constants::SYNC_PERIOD_DEFAULT_US,
      [this] { systemStatusTimerCallback(); });
}

void nif::system::SystemStatusManagerNode::systemStatusTimerCallback()
{
  nodeStatusesAgeCheck();

  // TODO Node status list is ot used, at the moment
  this->system_status_msg.header.stamp = this->now();
  this->system_status_msg.health_status.is_system_healthy = isSystemHealthy();
  this->system_status_msg.health_status.system_status_code = getSystemStatusCode();

  this->system_status_pub->publish(this->system_status_msg);
}

void nif::system::SystemStatusManagerNode::subscribeNodeStatus(
    const std::string &topic_name)
{

  auto subscription = this->create_subscription<nif::common::msgs::NodeStatus>(
      topic_name, nif::common::constants::QOS_INTERNAL_STATUS,
      [&](const nif::common::msgs::NodeStatus::SharedPtr msg)
      {
        nodeStatusUpdate(msg);
      });
  this->node_statuses_subs.push_back(std::move(subscription));
}

void nif::system::SystemStatusManagerNode::registerNodeServiceHandler(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const nif_msgs::srv::RegisterNodeStatus::Request::SharedPtr request,
    nif_msgs::srv::RegisterNodeStatus::Response::SharedPtr response)
{
//  Request data check
  if (
      !request->node_name.empty() &&
      this->status_by_name.find(request->node_name) == this->status_by_name.end() && // Process only if not already subscribed
      checkNodeStatusInRange(static_cast<NodeType>(request->node_type)) &&
      !request->status_topic_name.empty())
  {
    nif::common::types::t_node_id node_id;
    node_id = next_node_id++;

    {
      // Add the calling node to the registry of watched nodes
      auto node_status_record_ptr = std::make_unique<NodeStatusRecord>();
      this->node_status_records.push_back(std::move(node_status_record_ptr));
    }
    auto node_index = this->node_status_records.size() - 1;

    this->system_status_msg.health_status.node_list.push_back(request->node_name);

    this->status_by_id.insert({
            node_id, node_index
        });

    this->status_by_name.insert(
        {
            std::move(request->node_name),
            node_index
        });

    auto node_type = static_cast<const NodeType>(request->node_type);
    auto by_type_registry = this->statuses_by_type.find(node_type);
    // Key is not present
    if (by_type_registry == this->statuses_by_type.end())
    {
      auto type_vector = std::make_unique<std::vector<t_record_index>>();
      type_vector->push_back(node_index);
      this->statuses_by_type.insert({
          node_type,
          std::move(type_vector)
      });
    } else
    {
      this->statuses_by_type[node_type]->push_back(node_index);
    }

    // Subscribe to node status topic
    subscribeNodeStatus(request->status_topic_name);

    response->node_id = node_id;
    response->success = true;
    response->message = "Node " + request->node_name + " ";
  } else {
    response->node_id = 0;
    response->success = false;
    response->message = "ERROR: Bad service request.";
  }
}

void nif::system::SystemStatusManagerNode::nodeStatusUpdate(
    const nif::common::msgs::NodeStatus::SharedPtr msg)
{
  // TODO implement checks over msg.
  // TODO implement check on last_update_stamp to detect inactive nodes.
  auto index = this->status_by_id[msg->node_id];
  this->node_status_records[index]->node_status = msg;
}

bool nif::system::SystemStatusManagerNode::isSystemHealthy()
{
  for (const auto& record : this->node_status_records) {
    if (record->node_status &&
        record->node_status->node_status_code != common::NODE_OK) {
      this->is_system_healthy = false;
      return this->is_system_healthy;
    }
  }
  // All nodes OK
  this->is_system_healthy = true;
  return this->is_system_healthy;
}

SystemStatusCode nif::system::SystemStatusManagerNode::getSystemStatusCode()
{
  // TODO implement meaningful FSM
  if (isSystemHealthy())
  {
    for (const auto& record : this->node_status_records)
    {
    if (  record->node_status &&
          record->node_status->node_status_code != common::NODE_OK) {
        return SystemStatusCode::SYSTEM_ERROR;
      }
    }

  } else
  {
    return SystemStatusCode::SYSTEM_ERROR;
  }
  // All nodes OK
  return SystemStatusCode::SYSTEM_OK;
}
void nif::system::SystemStatusManagerNode::nodeStatusesAgeCheck() {
  // Extra iteration over all the records, it could be moved smw else, but
  // assuming a reasonable number of nodes, it shouldn't affect performance significantly.
  for (const auto &record : this->node_status_records) {
    if (record->node_status &&
        record->node_status->node_status_code != common::NodeStatusCode::NODE_NOT_INITIALIZED)
    {
      // Check if last update is recent enough
      if ((this->now() - record->node_status->stamp).nanoseconds() <=
          std::chrono::duration_cast<std::chrono::nanoseconds>(nif::system::NODE_DEAD_TIMEOUT_US).count()) {
        record->node_status->node_status_code = common::NODE_DEAD;
      }
    }
  }
}