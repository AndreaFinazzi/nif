//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#ifndef NIF_WAYPOINT_MANAGER_MISSION_V2_H
#define NIF_WAYPOINT_MANAGER_MISSION_V2_H

#include "memory"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string>
#include <vector>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "nif_common_nodes/i_base_node.h"
#include "nif_planning_common/path_cost_calculator.hpp"
#include "nif_utils/frenet_path_generator.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"

using namespace std;
using namespace Frenet;

// #define LONGI_MIN_T 4.4         // in sec
// #define LONGI_MAX_T 4.51        // 3.81 //in sec
// #define LONGI_DT 0.1            // in sec

#define LONGI_MIN_T 2.4  // in sec
#define LONGI_MAX_T 2.49 // 3.81 //in sec
#define LONGI_DT 0.1     // in sec

#define LEFT_WIDTH_MARGIN -5.5  // meter
#define RIGHT_WIDTH_MARGIN 5.51 // meter
#define WIDTH_DELTA 1.0         // meter

class WaypointManagerMissionV2 {
public:
  WaypointManagerMissionV2(const string &rl_wpt_file_path_,
                           const string &warm_up_wpt_file_path_,
                           const string &pit_wpt_file_path_,
                           const string &body_frame_id_,
                           const string &global_frame_id_,
                           const double &spline_interval_);

  void setCurrentOdometry(const nav_msgs::msg::Odometry &ego_vehicle_odom);
  void setSystemStatus(const nif_msgs::msg::SystemStatus &sys_status);
  void setOccupancyGridMap(const nav_msgs::msg::OccupancyGrid &occupancy_map);
  void calcMapTrack();

  void setCollisionAvoidanceGraphPath(const nav_msgs::msg::Path &coll_free_msg);

  // from dk
  void frenetPathsToPointCloud(
      std::vector<std::shared_ptr<FrenetPath>> &frenet_paths);

  void genCandidates();
  nav_msgs::msg::Path getDesiredMapTrackInGlobal() {
    calcMapTrack();
    return m_map_track_path_global;
  }
  nav_msgs::msg::Path getDesiredMapTrackInBody() {
    calcMapTrack();
    return m_map_track_path_body;
  }

  sensor_msgs::msg::PointCloud2 getFrenetCandidatesAsPc() {
    return m_frenet_candidates_pc;
  }

  nav_msgs::msg::Path getMinCostFrenetPath() {
    return m_collision_avoidance_path_body;
  }

private:
  WaypointManagerMissionV2() {}

  string m_rl_wpt_file_path;
  string m_warmup_wpt_file_path;
  string m_pit_wpt_file_path;
  string m_body_frame_id_str;
  string m_global_frame_id_str;
  double m_spline_interval;

  bool m_odom_first_callbacked;
  bool m_on_track_flg = true;

  double m_planning_vel_default = 15.0;     // in [m/s]
  double m_left_side_sampling_width = -5.0; // in [m]
  double m_right_side_sampling_width = 5.0; // in [m]
  double m_sampling_width_d = 1.0;          // in [m]

  double m_planning_t = 4.4;

  nav_msgs::msg::Odometry m_cur_odom;
  nif_msgs::msg::SystemStatus m_cur_sys_status;
  nif_msgs::msg::MissionStatus m_cur_mission_code;

  nav_msgs::msg::Path m_map_track_path_global;
  nav_msgs::msg::Path m_map_track_path_body;
  nav_msgs::msg::Path m_collision_avoidance_path_body;
  std::shared_ptr<FrenetPath> m_collision_avoidance_fp_body_ptr;
  sensor_msgs::msg::PointCloud2 m_frenet_candidates_pc;

  nav_msgs::msg::Path m_graph_based_path;

  std::shared_ptr<IWaypointManager> m_rl_wpt_manager;
  std::shared_ptr<IWaypointManager> m_warmup_wpt_manager;
  std::shared_ptr<IWaypointManager> m_pit_wpt_manager;

  // NOTE : only working in the body coordinate
  std::shared_ptr<FrenetPathGenerator> m_frenet_generator;
  std::shared_ptr<nif::planning::cost_calculator::costCalculator>
      m_frenet_cost_calculator;

  std::vector<double> splined_x, splined_y, splined_yaw;
  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D_xy;
};

#endif // NIF_WAYPOINT_MANAGER_MISSION_V2_H
