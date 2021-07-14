//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#include "nif_waypoint_manager_minimal/waypoint_manager_minimal.h"

WaypointManagerMinimal::WaypointManagerMinimal(
    vector<string>& wpt_file_path_list_,
    string& body_frame_id_,
    string& global_frame_id_)
  : IWaypointManager(wpt_file_path_list_, body_frame_id_, global_frame_id_) {}
