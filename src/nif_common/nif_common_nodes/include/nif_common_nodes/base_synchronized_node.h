//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H
#define NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H

#include "base_node.h"

namespace nif {
namespace common {


class IBaseSynchronizedNode : public IBaseNode {

public:


protected:
  IBaseSynchronizedNode(const std::string &node_name, const rclcpp::NodeOptions &options)
      : IBaseNode(node_name, options) {
    gclock_current = gclock_node_init;
  }

  virtual void run() = 0;

private:
  rclcpp::Time gclock_current;

  rclcpp::TimerBase::SharedPtr gclock_timer;

  void gClockCallback();
};

}
}
#endif // NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H
