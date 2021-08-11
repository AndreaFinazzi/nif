//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_system_status_manager_nodes/system_status_manager_node.h"

nif::system::SystemStatusManagerNode::SystemStatusManagerNode(
    const std::string &node_name)
    : IBaseSynchronizedNode(node_name, common::NodeType::SYSTEM, common::constants::SYNC_PERIOD_DEFAULT) {

}
void nif::system::SystemStatusManagerNode::initParameters() {
  IBaseSynchronizedNode::initParameters();
}

void nif::system::SystemStatusManagerNode::getParameters() {
  IBaseSynchronizedNode::getParameters();
}

void nif::system::SystemStatusManagerNode::run() {
  this->node_status_manager.getNodeStatusesMap().find(common::NodeStatusCode::NODE_OK);
}
