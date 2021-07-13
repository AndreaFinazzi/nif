//
// Created by usrg on 7/10/21.
//

#include "nif_waypoint_manager_nodes/waypoint_manager_node.h"

// TODO sohuld pass node_name_ as a reference here
WaypointManagerNode::WaypointManagerNode(std::string node_name_,
                                         string& wpt_yaml_path_,
                                         string& body_frame_id_,
                                         string& global_frame_id_)
  : WaypointManagerNode(node_name_,
                        std::make_shared<WaypointManagerMinimal>(
                            wpt_yaml_path_, body_frame_id_, global_frame_id_)) {
}

// TODO should pass node_name_ as a reference here
// TODO IBaseNode should be initialized first
WaypointManagerNode::WaypointManagerNode(
    std::string node_name_,
    std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr)
  : wpt_manager(wpt_manager_ptr), IBaseNode(node_name_) {}