//
// Created by usrg on 7/29/21.
//

#ifndef ROS2MASTER_NODE_STATUS_H
#define ROS2MASTER_NODE_STATUS_H

#include <random>
#include <rclcpp/rclcpp.hpp>

#include "nif_common/types.h"

namespace nif {
namespace common {

class IBaseNode;
using nif::common::NodeType;
using nif::common::NodeStatusCode;

/**
 * NodeStatus is intended to be used by each node to provide a description of its operational status.
 *
 */
class NodeStatus {

public:

  NodeStatus(const nif::common::IBaseNode& node,
             const NodeType node_type,
             rclcpp::Time time_init);

  /**
   * Update status code and update time; rclcpp::Node::now() is used to retrieve ros time.
   * @param status_code_ the status code to save as current status code.
   */
  void setStatusCode(NodeStatusCode);

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

  NodeStatusCode status_code = NODE_NOT_INITIALIZED;

};

}
}
#endif // ROS2MASTER_NODE_STATUS_H
