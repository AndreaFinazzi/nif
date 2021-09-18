//
// Created by usrg on 7/28/21.
//

#include "nif_common_nodes/node_status_manager.h"
#include "nif_common_nodes/i_base_node.h"

using nif::common::NodeStatusManager;
using nif::common::msgs::NodeStatus;

// Initialize static member
std::unordered_map<nif::common::types::t_node_id, NodeStatus&> NodeStatusManager::node_statuses_map;

NodeStatusManager::NodeStatusManager(
    const nif::common::IBaseNode & node,
    const NodeType node_type)
  : managed_node(node), node_type(node_type)
{
  NodeStatusManager::node_statuses_map.insert({
      this->node_status.node_id,
      this->node_status
  });
}

NodeStatusManager::~NodeStatusManager() {
  RCLCPP_INFO(rclcpp::get_logger(nif::common::constants::LOG_MAIN_LOGGER_NAME),
              "Destroying NodeStatusManager for node %s", this->managed_node.get_name());
  this->node_status.node_status_code = NodeStatusCode::NODE_DEAD;
}

void NodeStatusManager::update(NodeStatusCode status_code) noexcept {
  this->node_status.stamp_last_update = this->managed_node.now();
  this->node_status.node_status_code = status_code;
}
const std::unordered_map<nif::common::types::t_node_id,
                         nif::common::msgs::NodeStatus &> &
nif::common::NodeStatusManager::getNodeStatusesMap() {
  return node_statuses_map;
}
const rclcpp::Time &nif::common::NodeStatusManager::getTimeLastUpdate() const {
  return this->node_status.stamp_last_update;
}
const nif::common::NodeType
nif::common::NodeStatusManager::getNodeType() const {
  return node_type;
}
const NodeStatus &nif::common::NodeStatusManager::getNodeStatus() const {
  return node_status;
}

// NOT A SINGLETON ANYMORE!
//const NodeStatusManager* NodeStatusManager::getInstance() {
//  return instance;
//}
//
//
//NodeStatusManager::NodeStatusManager() {
//  instance = this;
//}
