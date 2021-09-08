//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/i_base_synchronized_node.h"
#include "nif_common/constants.h"
#include "std_msgs/msg/string.hpp"

using nif::common::IBaseSynchronizedNode;

void IBaseSynchronizedNode::gClockCallback() {
  /// Update clock value
  //  TODO : make now() available in this context.
  this->run();
}

//TODO make it a template function returning duration<>
const nif::common::types::t_clock_period_ns &IBaseSynchronizedNode::getGclockPeriodNs() const {
  return gclock_period_ns;
}
