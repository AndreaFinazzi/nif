//
// Created by usrg on 8/3/21.
//

#ifndef ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H
#define ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H

#include "nif_common_nodes/i_base_synchronized_node.h"

using nif::common::NodeStatusManager;

namespace nif {
namespace system {

class SystemStatusManagerNode : public nif::common::IBaseSynchronizedNode {

public:
  SystemStatusManagerNode(const std::string &node_name);

protected:
  void initParameters() override;
  void getParameters() override;
  void run() override;

private:
  /**
   * SystemStatus publisher. Publishes the latest system_status message, after
   * checking each node status.
   */
  rclcpp::Publisher<nif::common::msgs::SystemStatus>::SharedPtr
      system_status_pub;
};

} // namespace system
} // namespace nif

#endif // ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H
