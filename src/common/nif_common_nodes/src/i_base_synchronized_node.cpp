//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/i_base_synchronized_node.h"
#include "nif_common/constants.h"
#include "std_msgs/msg/string.hpp"

using namespace nif::common;

IBaseSynchronizedNode::IBaseSynchronizedNode(const std::string& node_name,
                                             const rclcpp::NodeOptions& options)
  : IBaseSynchronizedNode(node_name, options, constants::SYNC_PERIOD_DEFAULT) {}

void IBaseSynchronizedNode::gClockCallback() {
  /// Update clock value
  //  TODO : make now() available in this context.
  this->gclock_current = this->now();
  this->run();
}

void nif::common::IBaseSynchronizedNode::declareParameters() {}
void nif::common::IBaseSynchronizedNode::getParameters() {}
