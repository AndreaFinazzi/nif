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
  WaypointManagerMinimal(const vector<string>& wpt_file_path_list_,
                         const string& body_frame_id_,
                         const string& global_frame_id_,
                         const int& spline_interval_);

private:
  WaypointManagerMinimal() {}

  string m_wpt_yaml_path;
};

#endif // NIF_WAYPOINT_MANAGER_MINIMAL_H
