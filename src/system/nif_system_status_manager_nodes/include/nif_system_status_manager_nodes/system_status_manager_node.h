//
// Created by usrg on 8/3/21.
//

#ifndef ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H
#define ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H

#include <rclcpp/rclcpp.hpp>
#include "nif_common/types.h"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "nif_msgs/srv/register_node_status.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/insstdev.hpp"

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

  // TODO : finalize RaceControlState class
  /**
  * Race Control input from the race control interface.
  * It's automatically stored by its callback along with race_control_status_update_time.
  */
  nif::common::msgs::RCFlagSummary rc_flag_summary;

  /**
  * Race Control input last update time.
  */
  rclcpp::Time rc_flag_summary_update_time;

  bool has_rc_flag_summary = false;

  unsigned int mode = 255;
//  TODO make parameter
  const int max_counter_drop = 20;
  int t = 0;
  int counter_hb = 0;

  const int default_counter = 502;
  int counter_joy_prev = default_counter-1;
  int counter_joy = default_counter;
  bool joy_emergency_stop = false;
  bool recovery_enabled = true;


  /**
   * SystemStatus publisher. Publishes the latest system_status message, after
   * checking each node status.
   */
  rclcpp::Publisher<nif::common::msgs::SystemStatus>::SharedPtr system_status_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr joy_emergency_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hb_emergency_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr diagnostic_hb_pub;
  rclcpp::Publisher<nif::common::msgs::SystemStatus>::SharedPtr system_status_telem_pub;

  rclcpp::Subscription<nif::common::msgs::OverrideControlCmd>::SharedPtr joystick_sub;
  std::vector<
      rclcpp::Subscription<nif::common::msgs::NodeStatus>::SharedPtr> node_statuses_subs;
  rclcpp::Subscription<nif::common::msgs::RCFlagSummary>::SharedPtr rc_flag_summary_sub;
  rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subscriber_bestpos;
  rclcpp::Subscription<novatel_oem7_msgs::msg::INSSTDEV>::SharedPtr subscriber_insstdev;

  rclcpp::TimerBase::SharedPtr system_status_timer;
  rclcpp::TimerBase::SharedPtr telemetry_timer;

  rclcpp::Service<nif_msgs::srv::RegisterNodeStatus>::SharedPtr
      register_node_service;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service;

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

  nif::common::types::t_node_id newStatusRecord(const nif_msgs::srv::RegisterNodeStatus::Request::SharedPtr request);
  void nodeStatusUpdate(const nif::common::msgs::NodeStatus::SharedPtr msg);

  void systemStatusTimerCallback();

  void joystickCallback(const nif::common::msgs::OverrideControlCmd::SharedPtr msg);
  void RCFlagSummaryCallback(const nif::common::msgs::RCFlagSummary::UniquePtr msg);

  unsigned int insstdev_time_since_last_update_ = 0;
  double lat_stdev_ = 0.0;
  double long_stdev_ = 0.0;
  double best_pos_lat_stdev_ = 0.0;
  double best_pos_long_stdev_ = 0.0;

  void telemetry_timer_callback() {
      this->system_status_telem_pub->publish(this->system_status_msg);
  }

  void receive_bestpos(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg) {
      this->best_pos_lat_stdev_ = msg->lat_stdev;
      this->best_pos_long_stdev_ = msg->lon_stdev;
  }

  void receive_insstdev(
          const novatel_oem7_msgs::msg::INSSTDEV::SharedPtr msg) {
      this->lat_stdev_ = msg->latitude_stdev;
      this->long_stdev_ = msg->longitude_stdev;
      this->insstdev_time_since_last_update_ = msg->time_since_last_update;
  }

  bool gps_health_ok() {
      double pose_stdev_thres = 2.0;
      bool std_dev_trigger = (this->lat_stdev_ >  pose_stdev_thres ||
              this->long_stdev_ > pose_stdev_thres);
      bool time_since_trigger = (this->insstdev_time_since_last_update_ > 0);
      return (!std_dev_trigger && !time_since_trigger);
  }


  void recoveryServiceHandler(
          const std::shared_ptr<rmw_request_id_t> request_header,
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response);

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

  bool heartbeatOk();

  rclcpp::Duration node_inactive_timeout = rclcpp::Duration(1, 0);
};

} // namespace system
} // namespace nif

#endif // ROS2MASTER_SYSTEM_STATUS_MANAGER_NODE_H
