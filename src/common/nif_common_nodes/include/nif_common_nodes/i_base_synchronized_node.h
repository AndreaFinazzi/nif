//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H
#define NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H

#include "nif_common_nodes/i_base_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nif {
namespace common {

class IBaseSynchronizedNode : public IBaseNode {
public:
protected:

  template <typename DurationRepT, typename DurationT>
  IBaseSynchronizedNode(
      const std::string &node_name,
      const NodeType node_type,
      const std::chrono::duration<DurationRepT, DurationT> period = constants::SYNC_PERIOD_DEFAULT_US,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{},
      const bool register_to_system_manager = true)

      : IBaseNode(node_name, node_type, options, register_to_system_manager),
        gclock_period_ns(
            std::chrono::duration_cast<decltype(gclock_period_ns)>(period)),
        gclock_period_duration(period)

  {
    if (this->gclock_period_ns >= nif::common::constants::SYNC_PERIOD_MIN &&
        this->gclock_period_ns <= nif::common::constants::SYNC_PERIOD_MAX) {

      gclock_timer = this->create_wall_timer(
          this->gclock_period_ns,
          std::bind(&IBaseSynchronizedNode::gClockCallback, this));

    } else {
      //    Not allowed to instantiate with this period
      throw std::range_error("Sync Period out of range.");
    }
  }

  const std::chrono::nanoseconds &getGclockPeriodNs() const;
  const rclcpp::Duration &getGclockPeriodDuration() const;

protected:
  /**
   * run() is called by the timer callback, and must be defined by the derived
   * classes.
   */
  virtual void run() = 0;

private:
  rclcpp::TimerBase::SharedPtr gclock_timer;

  //  Not const to keep a door open (changing period at runtime)
  nif::common::types::t_clock_period_ns gclock_period_ns;
  rclcpp::Duration gclock_period_duration;

  void gClockCallback();
};

} // namespace common
} // namespace nif
#endif // NIF_COMMON_NODES_BASESYNCHRONIZEDNODE_H