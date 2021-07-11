//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H
#define NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H

#include "i_base_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nif {
namespace common {

class IBaseSynchronizedNode : public IBaseNode {
public:
  virtual ~IBaseSynchronizedNode();

protected:
  IBaseSynchronizedNode(const std::string &node_name,
                        const rclcpp::NodeOptions &options);

  template <typename DurationRepT, typename DurationT>
  IBaseSynchronizedNode(
      const std::string &node_name, const rclcpp::NodeOptions &options,
      const std::chrono::duration<DurationRepT, DurationT> period)
      : IBaseNode(node_name, options) {
    if (period >= nif::common::constants::SYNC_PERIOD_MIN &&
        period <= nif::common::constants::SYNC_PERIOD_MAX) {
      gclock_timer = this->create_wall_timer(
          period, std::bind(&IBaseSynchronizedNode::gClockCallback, this));
    } else {
      //    Not allowed to instantiate with this period
      throw std::range_error("Sync Period out of range.");
    }
  }

  void initParameters() override = 0;
  void getParameters() override = 0;

  /**
   * run() is called by the timer callback, and must be defined by the derived
   * classes.
   */
  virtual void run() = 0;

private:
  rclcpp::TimerBase::SharedPtr gclock_timer;

  void gClockCallback();
};

} // namespace common
} // namespace nif
#endif // NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H
