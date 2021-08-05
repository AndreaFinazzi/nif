//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/i_base_synchronized_node.h"
#include "nif_common/constants.h"
#include "std_msgs/msg/string.hpp"

using namespace nif::common;

void IBaseSynchronizedNode::gClockCallback() {
  /// Update clock value
  //  TODO : make now() available in this context.
  this->run();
}

const std::chrono::nanoseconds &IBaseSynchronizedNode::getGclockPeriod() const {
  return gclock_period;
}
