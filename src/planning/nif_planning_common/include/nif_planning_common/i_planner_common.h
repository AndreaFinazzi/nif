
//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_I_PLANNER_ALGORITHM_H
#define ROS2MASTER_I_PLANNER_ALGORITHM_H

#include <memory>

#include "nif_common/types.h"
#include "nif_common/vehicle_model.h"
namespace nif {
namespace planning {
namespace common {

class IPlannerAlgorithm {
public:
protected:
private:
  /**
   * Last known vehicle state.
   */
  nif::common::msgs::PowertrainState::SharedPtr vehicle_state_prev;

  /**
   * Collection of opponent vehicles' kinematic states.
   */
  std::shared_ptr<nif::common::types::t_oppo_collection_states>
      opponents_states;

  nif::common::msgs::TerrainState track_state;

  /**
   * Ego vehicle model. It stores the model dynamics parameters.
   */
  nif::common::VehicleModel vehicle_model;

  /**
   * TODO: still not sure if we need this or not. Commented for now.
   * @return
   */
  //    BehaviorState behavior_state;

  virtual nif::common::msgs::Trajectory solve() = 0;
};
} // namespace common
} // namespace planning
} // namespace nif

#endif // ROS2MASTER_I_PLANNER_ALGORITHM_H
