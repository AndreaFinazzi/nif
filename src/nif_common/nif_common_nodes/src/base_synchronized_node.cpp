//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/base_synchronized_node.h"
#include "std_msgs/msg/string.hpp"

nif::common::IBaseSynchronizedNode(const std::string &node_name, const rclcpp::NodeOptions &options, const std::chrono::duration<DurationRepT, DurationT> period)
    : IBaseNode(node_name, options) {
  gclock_current = gclock_node_init;
  gclock_timer = IBaseNode::Node::create_wall_timer(period, std::bind(&gClockCallback, this));
  );
}

void nif::common::IBaseSynchronizedNode::gClockCallback() {
  /// Update clock value
  this->gclock_current = this->now();
  this->run();

}