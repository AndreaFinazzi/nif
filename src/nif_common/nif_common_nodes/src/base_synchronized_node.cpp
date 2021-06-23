//
// Created by usrg on 6/18/21.
//

#include "nif_common_nodes/base_node_synchronized.h"

#include "std_msgs/msg/string.hpp"

void nif::common::IBaseSynchronizedNode::gClockCallback() {
  /// Update clock value
  this->gclock_current = this->now();
  this->run();

}