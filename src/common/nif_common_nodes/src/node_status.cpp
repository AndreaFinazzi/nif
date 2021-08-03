//
// Created by usrg on 7/29/21.
//

#include "nif_common_nodes/node_status.h"
#include "nif_common_nodes/i_base_node.h"

using nif::common::NodeStatus;

NodeStatus::NodeStatus(
    const nif::common::IBaseNode& node,
    const NodeType node_type,
    rclcpp::Time time_init)
      : node(node),
        node_id(std::rand()),
        node_type(node_type),
        time_last_update(time_init),
        status_code(INITIALIZED)
  {}

  void NodeStatus::setStatusCode(NodeStatusCode status_code_) {
    this->status_code = status_code_;
    this->time_last_update = this->node.now();
  }
