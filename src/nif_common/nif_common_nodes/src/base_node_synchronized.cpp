//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/BaseNodeSynchronized.h"

void BaseNodeSynchronized::timer_callback() {
  std_msgs::msg::String message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}