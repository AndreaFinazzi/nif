//
// Created by usrg on 7/10/21.
//

#ifndef ROS2MASTER_WAYPOINT_MANAGER_NODE_H
#define ROS2MASTER_WAYPOINT_MANAGER_NODE_H

#include "nif_common_nodes/i_base_node.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"
#include "nif_waypoint_manager_minimal/waypoint_manager_minimal.h"

class WaypointManagerNode : public nif::common::IBaseNode {
public:
  WaypointManagerNode()
    : WaypointManagerNode(std::make_shared<WaypointManagerMinimal>()) {
    //        this->ego_vehicle_state
  }

  WaypointManagerNode(std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr) {
    wpt_manager = wpt_manager_ptr;

    //        this->ego_vehicle_state
  }

private:
  std::shared_ptr<WaypointManagerMinimal> wpt_manager;

  //    Subscribers
  //
};

#endif // ROS2MASTER_WAYPOINT_MANAGER_NODE_H
