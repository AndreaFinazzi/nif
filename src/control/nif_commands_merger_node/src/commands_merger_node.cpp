//
// Created by usrg on 8/31/21.
//

#include "nif_commands_merger_node/commands_merger_node.h"

using nif::control::CommandsMergerNode;

CommandsMergerNode::CommandsMergerNode() : Node("CommandsMergerNode") {
  num_of_lat_controller = sub_topic_name_of_lat_cmds.size();
  num_of_long_controller = sub_topic_name_of_long_cmds.size();

  if (sub_topic_name_of_lat_cmds.size() !=
      longi_control_cmd_idx_matched_w_lat.size()) {
    throw std::runtime_error("Controller merger : combination of lat and long "
                             "controller is not properly setup.");
  }
  if (num_of_lat_controller == 0 || num_of_long_controller == 0) {
    throw std::runtime_error(
        "Controller merger : subscribe topics are not properly setup.");
  }
  if (std::max(num_of_lat_controller, num_of_long_controller) !=
      put_topic_name_of_cmds.size()) {
    throw std::runtime_error(
        "Controller merger : publish topics are not properly setup.");
  }

  recieved_steer_cmd_vec.resize(num_of_lat_controller);
  recieved_vel_cmd_vec.resize(num_of_long_controller);

  // then all good
  // make lateral subscribers
  for (int sub_lat_idx = 0; sub_lat_idx < num_of_lat_controller;
       sub_lat_idx++) {

    fcn_lat = std::bind(&CommandsMergerNode::lateralControlCmdCallback, this,
                        std::placeholders::_1, sub_lat_idx);

    auto subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        sub_topic_name_of_lat_cmds[sub_lat_idx], nif::common::constants::QOS_CONTROL_CMD, fcn_lat);

    // auto subscription_ =
    // this->create_subscription<std_msgs::msg::Float32>(
    //     sub_topic_name_of_lat_cmds[sub_lat_idx],
    //     [this, sub_lat_idx](const std_msgs::msg::Float32::SharedPtr msg)
    //     {
    //       this->lateralControlCmdCallback(msg, sub_lat_idx);
    //     });

    subcribers_vec_lateral_cmd.push_back(subscription_);
  }

  // make longitudinal subscribers
  for (int sub_long_idx = 0; sub_long_idx < num_of_long_controller;
       sub_long_idx++) {
    // std::function<void(const std_msgs::msg::Float32::SharedPtr msg)> fcn =
    //     std::bind(&CommandsMergerNode::, std::placeholders::_1,
    //     sub_long_idx);

    // auto subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    //     sub_topic_name_of_long_cmds[sub_long_idx],
    //     [this, sub_long_idx](const std_msgs::msg::Float32::SharedPtr msg) {
    //       this->lateralControlCmdCallback(msg, sub_long_idx);
    //     });
    // subcribers_vec_longitudinal_cmd.push_back(subscription_);
    fcn_long = std::bind(&CommandsMergerNode::longitudinalControlCmdCallback,
                         this, std::placeholders::_1, sub_long_idx);

    auto subscription_ = this->create_subscription<std_msgs::msg::Float32>(
        sub_topic_name_of_long_cmds[sub_long_idx], nif::common::constants::QOS_CONTROL_CMD, fcn_long);

    subcribers_vec_longitudinal_cmd.push_back(subscription_);
  }

  // make publishers
  for (int pub_idx = 0; pub_idx < put_topic_name_of_cmds.size(); pub_idx++) {
    auto publisher_ = this->create_publisher<nif_msgs::msg::ControlCommand>(
        put_topic_name_of_cmds[pub_idx], nif::common::constants::QOS_CONTROL_CMD);
    publishers_vec_command_cmd.push_back(publisher_);
  }
}

void CommandsMergerNode::lateralControlCmdCallback(
    const std_msgs::msg::Float32::SharedPtr lat_msg, int sub_lat_idx) {
  recieved_steer_cmd_vec[sub_lat_idx] = lat_msg->data;

  ////////////////////////////////////////////////////////
  // Here, we give more priority to the lateral controller
  ////////////////////////////////////////////////////////

  double desired_steer_cmd = lat_msg->data;
  double desired_velocity_cmd =
      recieved_vel_cmd_vec[longi_control_cmd_idx_matched_w_lat[sub_lat_idx]];

  last_control_cmd.desired_steer_cmd.data = desired_steer_cmd;
  last_control_cmd.desired_velocity_cmd.data = desired_velocity_cmd;

  publishers_vec_command_cmd[sub_lat_idx]->publish(last_control_cmd);
}

void CommandsMergerNode::longitudinalControlCmdCallback(
    const std_msgs::msg::Float32::SharedPtr long_msg, int sub_long_idx) {
  recieved_vel_cmd_vec[sub_long_idx] = long_msg->data;
}