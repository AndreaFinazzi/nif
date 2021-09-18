//
// Created by usrg on 7/17/21.
//

#ifndef ROS2MASTER_COMMAND_MERGER_NODE_H
#define ROS2MASTER_COMMAND_MERGER_NODE_H

#include <chrono>
#include <functional>
#include <math.h>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_msgs/msg/control_command.hpp"
#include "novatel_gps_msgs/msg/novatel_velocity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nif_common/types.h"
#include <map>
#include <rclcpp/rclcpp.hpp>

using namespace std;

namespace nif {
namespace control {
class CommandsMergerNode : public rclcpp::Node {
public:
  CommandsMergerNode();

  // Single callback for lateral control cmd
  void
  lateralControlCmdCallback(const std_msgs::msg::Float32::SharedPtr lat_msg,
                            int sub_lat_idx);
  // Single callback for longitudinal control cmd
  void longitudinalControlCmdCallback(
      const std_msgs::msg::Float32::SharedPtr longi_msg, int sub_long_idx);

private:
  // NOTE : make sure that the topic name for each controller is properly setup
  vector<string> sub_topic_name_of_lat_cmds{"lat_a", "lat_b", "lat_c"};
  vector<string> sub_topic_name_of_long_cmds{"long_a", "long_b", "long_c"};
  vector<int> longi_control_cmd_idx_matched_w_lat{0, 0, 0};
  vector<string> put_topic_name_of_cmds{"a_out", "b_out", "c_out"};

  int num_of_lat_controller;
  int num_of_long_controller;

  vector<double> recieved_steer_cmd_vec;
  vector<double> recieved_vel_cmd_vec;

  vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr>
      subcribers_vec_lateral_cmd;
  vector<rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr>
      subcribers_vec_longitudinal_cmd;

  vector<rclcpp::Publisher<nif_msgs::msg::ControlCommand>::SharedPtr>
      publishers_vec_command_cmd;

  // To control safety layer <data>
  uint control_priority_order;
  //   _Float32 velocity_control_cmd;
  //   _Float32 steering_control_cmd;

  // To control safety layer <msg>
  nif_msgs::msg::ControlCommand last_control_cmd;

  std::function<void(const std_msgs::msg::Float32::SharedPtr msg)> fcn_lat;
  std::function<void(const std_msgs::msg::Float32::SharedPtr msg)> fcn_long;
};

} // namespace control
} // namespace nif
#endif // ROS2MASTER_COMMAND_MERGER_NODE_H
