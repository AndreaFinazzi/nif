//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/i_base_synchronized_node.h"
#include "std_msgs/msg/string.hpp"

using namespace nif::common;

template<typename DurationRepT, typename DurationT>
IBaseSynchronizedNode::IBaseSynchronizedNode(const std::string &node_name, const rclcpp::NodeOptions &options, const std::chrono::duration<DurationRepT, DurationT> period)
    : IBaseNode(node_name, options) {
  gclock_current = gclock_node_init;
  gclock_timer = this->create_wall_timer(period, std::bind(&IBaseSynchronizedNode::gClockCallback, this));
}

void IBaseSynchronizedNode::gClockCallback() {
  /// Update clock value
//  TODO : make now() available in this context.
//  this->gclock_current = this->now();
  this->run();

}