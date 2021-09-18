//
// Created by usrg on 7/17/21.
//

#ifndef ROS2MASTER_CONTROL_MINIMAL_NODE_H
#define ROS2MASTER_CONTROL_MINIMAL_NODE_H

#include "nif_control_common_nodes/i_controller_node.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "novatel_gps_msgs/msg/novatel_velocity.hpp"

namespace nif {
namespace control {

class ControlMinimalNode : public nif::control::IControllerNode {
public:
  explicit ControlMinimalNode(const std::string &node_name);

  nif::common::msgs::ControlCmd::SharedPtr control_cmd;
  std_msgs::msg::Float32 steering_cmd;
  std_msgs::msg::Float64 lookahead_error;
  std_msgs::msg::Float64 lat_error;

private:
  void controlCallback();
  void calculateFFW();
  void calculateFB();
  void calculateSteeringCmd();
  void setCmdsToZeros();
  void checkSteeringCmd();
  void publishDebugSignals();
  void receivePath(const nav_msgs::msg::Path::SharedPtr msg);
  static int findLookaheadIndex(std::vector<geometry_msgs::msg::PoseStamped> refPath, double desLookaheadValue);
  void receiveVelocity(const novatel_gps_msgs::msg::NovatelVelocity::SharedPtr msg);
  void receiveJoySteer(const std_msgs::msg::Float32::SharedPtr msg);
  nif::common::msgs::ControlCmd::SharedPtr solve() override;

protected:
  void initParameters() override;
  void getParameters() override;

private:
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubSteeringCmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLookaheadError_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubLatError_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subPath_;
  rclcpp::Subscription<novatel_gps_msgs::msg::NovatelVelocity>::SharedPtr subVelocity_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subJoySteering_;
  rclcpp::Time recv_time_;

  double curvature_ = 0.0;
  double speed_ = 0.0;
  double feedforward_ = 0.0;
  double feedback_ = 0.0;
  double steering_override_ = 0.0;

  bool path_ready = false;
};

} // namespace control
} // namespace nif
#endif // ROS2MASTER_CONTROL_MINIMAL_NODE_H
