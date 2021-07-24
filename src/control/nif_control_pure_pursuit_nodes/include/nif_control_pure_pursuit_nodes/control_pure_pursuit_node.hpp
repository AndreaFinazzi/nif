//
// Created by usrg on 7/19/21.
//

#ifndef ROS2MASTER_CONTROL_PURE_PURSUIT_NODE_H
#define ROS2MASTER_CONTROL_PURE_PURSUIT_NODE_H

#include "nif_control_common_nodes/i_controller_node.h"
#include "pure_pursuit.h"

#include <chrono>
#include <functional>
#include <math.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "novatel_gps_msgs/msg/novatel_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

namespace nif {
namespace control {

class ControlPurePursuitNode : public nif::control::IControllerNode {
public:
  explicit ControlPurePursuitNode(const std::string &node_name);

  nif::common::msgs::ControlCmd::SharedPtr control_cmd;
  std_msgs::msg::Float32 steering_cmd;
  std_msgs::msg::Float64 lookahead_error;
  std_msgs::msg::Float64 lat_error;

private:
  void mapTrackCallback(const nav_msgs::msg::Path::SharedPtr msg);
  nif::common::msgs::ControlCmd &solve() override;

protected:
  void initParameters() override;
  void getParameters() override;

private:
  void egoUpdateTimerCallback() const;

  rclcpp::TimerBase::SharedPtr m_ego_update_timer;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_steer_cmd_pub;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_map_track_sub;

  rclcpp::Time maptrack_recv_time_;
  nav_msgs::msg::Path m_maptrack;

  double m_param_min_lookahead_dist;
  double m_param_max_lookahead_dist;
  double m_param_lookahead_speed_ratio;
  bool m_param_use_lpf_flg;
  double m_param_lfp_gain;
  bool m_param_is_steer_sign_inver;

  bool m_ego_status_first_run = false;
  bool m_maptrack_first_run = false;

  std::shared_ptr<PurePursuit> m_pure_pursuit_handler_ptr;
  nif::common::msgs::ControlCmd::SharedPtr steer_control_cmd_msg;
};

} // namespace control
} // namespace nif
#endif // ROS2MASTER_CONTROL_PURE_PURSUIT_NODE_H