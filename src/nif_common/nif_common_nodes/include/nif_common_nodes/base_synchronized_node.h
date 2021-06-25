//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H
#define NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H

#include "rclcpp/rclcpp.hpp"
#include "i_base_node.h"

namespace nif {
namespace common {


class IBaseSynchronizedNode : public IBaseNode {

public:


protected:
  template<class DurationRepT>
  template<class DurationT>
  IBaseSynchronizedNode(const std::string &node_name, const rclcpp::NodeOptions &options, const std::chrono::duration<DurationRepT, DurationT> period);

  /**
   * run() is called by the timer callback, and must be defined by the derived classes.
   */
  virtual void run() = 0;

private:
  rclcpp::Time gclock_current;

  rclcpp::WallTimer<rclcpp::TimerCallbackType>::SharedPtr gclock_timer;

  void gClockCallback();
};

}
}
#endif // NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H
