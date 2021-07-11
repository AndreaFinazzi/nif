//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#ifndef NIF_WAYPOINT_MANAGER_MINIMAL_H
#define NIF_WAYPOINT_MANAGER_MINIMAL_H

#include <iostream>
#include <string>
#include <vector>

#include "nif_common_nodes/i_base_node.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"

class WaypointManagerMinimal : public IWaypointManager {
public:
  WaypointManagerMinimal() {}
  WaypointManagerMinimal(string& wpt_yaml_path_,
                         string& body_frame_id_,
                         string& global_frame_id_);
  ~WaypointManagerMinimal();

private:
  string m_wpt_yaml_path;
  string m_body_frame_id_;
  string m_global_frame_id_;
};

#endif // NIF_WAYPOINT_MANAGER_MINIMAL_H