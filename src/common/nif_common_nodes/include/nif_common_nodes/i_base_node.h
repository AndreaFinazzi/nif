//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/18/21.
//

#ifndef NIF_COMMON_NODES_BASENODE_H
#define NIF_COMMON_NODES_BASENODE_H

#include <string>

#include "../../../nif_common/include/nif_common/types.h"
#include "nif_common/types.h"
#include "nif_utils/utils.h"

#include <rclcpp/rclcpp.hpp>

namespace nif {
namespace common {

class IBaseNode : public rclcpp::Node {
public:
protected:
  IBaseNode(const std::string& node_name, const rclcpp::NodeOptions& options);

  /// Expose time to children

  rclcpp::Time gclock_node_init, gclock_current;

  nif::common::msgs::PowertrainState ego_powertrain_state;

  // TODO : finalize RaptorState class
  nif::common::msgs::RaptorState raptor_state;

  // TODO : finalize RaceControlState class
  nif::common::msgs::RaceControlState race_control_state;

  //  Reference to utils not needed, as it'll be everything static (probably)
  //  nif::common::utils:: utils;

private:
  /**
   * The default constructor is hidden from the outside to prevent unnamed
   * nodes.
   */
  IBaseNode();

  rclcpp::Subscription<nif::common::msgs::PowertrainState>::SharedPtr
      ego_powertrain_state_sub;
  rclcpp::Subscription<nif::common::msgs::RaptorState>::SharedPtr
      raptor_state_sub;
  rclcpp::Subscription<nif::common::msgs::RaceControlState>::SharedPtr
      race_control_state_sub;

  virtual void declareParameters() = 0;
  virtual void getParameters() = 0;

  void egoVehiclePowertrainCallback(
      const nif::common::msgs::PowertrainState::SharedPtr msg);
  void raptorStateCallback(const nif::common::msgs::RaptorState::SharedPtr msg);
  void raceControlStateCallback(
      const nif::common::msgs::RaceControlState::SharedPtr msg);
};

} // namespace common
} // namespace nif
#endif // NIF_COMMON_NODES_BASENODE_H
