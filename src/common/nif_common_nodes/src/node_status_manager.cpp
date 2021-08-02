//
// Created by usrg on 7/28/21.
//

#include "nif_common_nodes/node_status_manager.h"
#include "nif_common_nodes/i_base_node.h"

using nif::common::NodeStatusManager;
using nif::common::NodeStatus;

// Initialize static member
std::unordered_map<nif::common::types::t_node_id, NodeStatus&> NodeStatusManager::node_statuses_map;

NodeStatusManager::NodeStatusManager(
    const nif::common::IBaseNode &node,
    const NodeType node_type)
  : node_status(node, node_type, node.now())
{
  NodeStatusManager::node_statuses_map.insert({
      this->node_status.getNodeId(),
      this->node_status
  });
}

NodeStatusManager::~NodeStatusManager() {
  RCLCPP_INFO(rclcpp::get_logger(nif::common::constants::LOG_MAIN_LOGGER_NAME),
              "Destroying NodeStatusManager for node %s", this->node_status.getNode().get_name());
  this->node_status.setStatusCode(NodeStatusCode::DEAD);
}

void NodeStatusManager::update(NodeStatusCode status_code) {
  this->node_status.setStatusCode(status_code);
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
