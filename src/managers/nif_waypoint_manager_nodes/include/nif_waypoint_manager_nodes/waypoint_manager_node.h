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
  /**
   *
   * Using default WaypointManager -> WaypointManagerMinimal
   *
   **/
  WaypointManagerNode(std::string node_name_,
                      string& wpt_yaml_path_,
                      string& body_frame_id_,
                      string& global_frame_id_);

  WaypointManagerNode(std::string node_name_,
                      std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr);

private:
  WaypointManagerNode();

  std::shared_ptr<WaypointManagerMinimal> wpt_manager;

  //    Subscribers
  //
};

#endif // ROS2MASTER_WAYPOINT_MANAGER_NODE_H
