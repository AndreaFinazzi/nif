//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#include "nif_waypoint_manager_minimal/waypoint_manager_minimal.h"

WaypointManagerMinimal::WaypointManagerMinimal(string& wpt_yaml_path_,
                                               string& body_frame_id_,
                                               string& global_frame_id_)
  : IWaypointManager(wpt_yaml_path_, body_frame_id_, global_frame_id_) {
  m_wpt_yaml_path = wpt_yaml_path_;
  m_body_frame_id_ = body_frame_id_;
  m_global_frame_id_ = global_frame_id_;
}
