//
// Created by usrg on 8/3/21.
//

#ifndef ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H
#define ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "nif_common/types.h"
#include "nif_msgs/srv/register_node_status.hpp"

using nif::common::NodeStatusCode;
using nif::common::NodeType;
using nif::common::SystemStatusCode;

using t_record_index = unsigned int;


namespace nif {
namespace system {

const std::chrono::microseconds NODE_DEAD_TIMEOUT_US(250000); // 0.25s

struct NodeStatusRecord {
  nif::common::msgs::NodeStatus::SharedPtr node_status;

};

class SystemStatusManagerNode : public rclcpp::Node {

public:
  explicit SystemStatusManagerNode(const std::string &node_name);

private:
  nif::common::types::t_node_id next_node_id = 1000;

  nif::common::msgs::SystemStatus system_status_msg;

  bool is_system_healthy = false;

  common::SystemStatusCode system_status_code = common::SYSTEM_NOT_INITIALIZED;

  /**
   * SystemStatus publisher. Publishes the latest system_status message, after
   * checking each node status.
   */
  rclcpp::Publisher<nif::common::msgs::SystemStatus>::SharedPtr
      system_status_pub;

  std::vector<
      rclcpp::Subscription<nif::common::msgs::NodeStatus>::SharedPtr>
      node_statuses_subs;

  rclcpp::Service<nif_msgs::srv::RegisterNodeStatus>::SharedPtr
      register_node_service;

  rclcpp::TimerBase::SharedPtr system_status_timer;

//  Node statuses storage and indices
  std::vector<std::shared_ptr<NodeStatusRecord>> node_status_records{};

  std::unordered_map<nif::common::types::t_node_id, t_record_index > status_by_id{};

  std::unordered_map<std::string, t_record_index> status_by_name{};

  std::unordered_map<NodeType,
                     std::unique_ptr<
                         std::vector<t_record_index>>> statuses_by_type{};


  void subscribeNodeStatus(const std::string & topic_name);

  void registerNodeServiceHandler(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const nif_msgs::srv::RegisterNodeStatus::Request::SharedPtr request,
      nif_msgs::srv::RegisterNodeStatus::Response::SharedPtr response);

  void nodeStatusUpdate(const nif::common::msgs::NodeStatus::SharedPtr msg);

  bool checkNodeStatusInRange(nif::common::NodeType type_id)
  {
    if (
        type_id == common::NodeType::SYSTEM         ||
        type_id == common::NodeType::TOOL           ||
        type_id == common::NodeType::PERCEPTION     ||
        type_id == common::NodeType::LOCALIZATION   ||
        type_id == common::NodeType::PREDICTION     ||
        type_id == common::NodeType::PLANNING       ||
        type_id == common::NodeType::CONTROL
        ) return true;
    return false;
  }

  void systemStatusTimerCallback();

  /**
   * System Healthy state machine, with boolean output
   * @return boolean system health state. true if system health is nominal.
   */
  bool isSystemHealthy();

  /**
   * System status state machine, with status output code.
   * @return system status code
   */
  common::SystemStatusCode getSystemStatusCode();
  void nodeStatusesAgeCheck();
};

} // namespace system
} // namespace nif

#endif // ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H
