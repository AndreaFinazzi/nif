//
// Created by usrg on 7/29/21.
//

#ifndef ROS2MASTER_NODE_STATUS_H
#define ROS2MASTER_NODE_STATUS_H

#include <random>

#include "nif_common_nodes/i_base_node.h"

enum NodeStatusCode : std::uint8_t {
  OK = 0,
  INITIALIZED = 1,
  NOT_INITIALIZED = 200,
  FATAL_ERROR = 254,
  DEAD = 255
};

enum NodeType : std::int8_t {
  PERCEPTION,
  PLANNING,
  PREDICTION,
  CONTROL,
  TOOL,
  SYSTEM
};

/**
 * NodeStatus is intended to be used by each node to provide a description of its operational status.
 *
 */
class NodeStatus {

public:

  NodeStatus(const nif::common::IBaseNode& node,
             const NodeType node_type,
             rclcpp::Time time_init)
      : node(node),
        node_id(std::rand()),
        node_type(node_type),
        time_last_update(time_init),
        status_code(INITIALIZED)
  {}

  /**
   * Update status code and update time; rclcpp::Node::now() is used to retrieve ros time.
   * @param status_code_ the status code to save as current status code.
   */
  void setStatusCode(NodeStatusCode status_code_) {
    this->status_code = status_code_;
    this->time_last_update = this->node.now();
  }

  const nif::common::IBaseNode& getNode() const { return node; }
  const nif::common::types::t_node_id& getNodeId() const { return node_id; }
  const NodeType& getNodeType() const { return node_type; }
  NodeStatusCode getStatusCode() const { return status_code; }
  const rclcpp::Time& getTimeLastUpdate() const { return time_last_update; }

private:
  const nif::common::IBaseNode& node;
  const nif::common::types::t_node_id node_id;
  const NodeType node_type;

  rclcpp::Time time_last_update;

  NodeStatusCode status_code = NOT_INITIALIZED;

};
#endif // ROS2MASTER_NODE_STATUS_H
