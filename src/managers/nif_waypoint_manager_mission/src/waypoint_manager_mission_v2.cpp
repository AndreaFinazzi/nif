//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 7/6/21.
//

#include "nif_waypoint_manager_mission/waypoint_manager_mission_v2.h"

WaypointManagerMissionV2::WaypointManagerMissionV2(
    const string &rl_wpt_file_path_, const string &warm_up_wpt_file_path_,
    const string &pit_wpt_file_path_, const string &body_frame_id_,
    const string &global_frame_id_, const double &spline_interval_)
    : m_rl_wpt_file_path(rl_wpt_file_path_),
      m_warmup_wpt_file_path(warm_up_wpt_file_path_),
      m_pit_wpt_file_path(pit_wpt_file_path_),
      m_body_frame_id_str(body_frame_id_),
      m_global_frame_id_str(global_frame_id_),
      m_spline_interval(spline_interval_)
{

  // std::cout<<"sibal++++++++++++++++++++++++++++++++++++++="<<std::endl;
  m_map_track_path_global.header.frame_id = m_global_frame_id_str;
  m_map_track_path_body.header.frame_id = m_body_frame_id_str;
  m_collision_avoidance_path_body.header.frame_id = m_body_frame_id_str;

  m_frenet_generator = std::make_shared<FrenetPathGenerator>();
  m_frenet_cost_calculator =
      std::make_shared<nif::planning::cost_calculator::costCalculator>();

  vector<string> rl_file_path_vec_tmp{m_rl_wpt_file_path};
  vector<string> warmup_file_path_vec_tmp{m_warmup_wpt_file_path};
  vector<string> pit_file_path_vec_tmp{m_pit_wpt_file_path};

  m_odom_first_callbacked = false;

  m_rl_wpt_manager = std::make_shared<IWaypointManager>(
      rl_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
  m_warmup_wpt_manager = std::make_shared<IWaypointManager>(
      warmup_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
  m_pit_wpt_manager = std::make_shared<IWaypointManager>(
      pit_file_path_vec_tmp, m_body_frame_id_str, m_global_frame_id_str,
      int(m_spline_interval));
}

void WaypointManagerMissionV2::setCurrentOdometry(
    const nav_msgs::msg::Odometry &ego_vehicle_odom)
{
  m_cur_odom = ego_vehicle_odom;
  m_rl_wpt_manager->setCurrentOdometry(ego_vehicle_odom);
  m_warmup_wpt_manager->setCurrentOdometry(ego_vehicle_odom);
  m_pit_wpt_manager->setCurrentOdometry(ego_vehicle_odom);
  if (!m_odom_first_callbacked)
    m_odom_first_callbacked = true;
}

void WaypointManagerMissionV2::setSystemStatus(
    const nif_msgs::msg::SystemStatus &sys_status)
{
  m_cur_sys_status = sys_status;
  m_cur_mission_code = sys_status.mission_status;
}

void WaypointManagerMissionV2::setOccupancyGridMap(
    const nav_msgs::msg::OccupancyGrid &occupancy_map)
{
  this->m_frenet_cost_calculator->setOccupancyMap(occupancy_map);
}

void WaypointManagerMissionV2::calcMapTrack()
{

  if (m_odom_first_callbacked)
  {
    if (m_cur_mission_code.mission_status_code ==
        m_cur_mission_code.MISSION_PIT_IN)
    {
      m_map_track_path_global = m_pit_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

      // std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
      //     m_frenet_generator->applyCubicSpliner_2d_ros(m_map_track_path_body,
      //                                                  1.0);
      // if (splined_x.empty())
      //   return;

      // int cur_idx = 0;
      // double current_position_d = 0.0;
      // if (!splined_y.empty())
      //   current_position_d = -1 * splined_y[cur_idx];

      // // vector<double> target_speed_vector_in{10.0};
      // vector<double> target_speed_vector_in;
      // double clipped_speed = nif::common::utils::numeric::clip(4.0, 40.0,
      //                                                          m_cur_odom.twist.twist.linear.x);
      // target_speed_vector_in.push_back(clipped_speed);
      // std::vector<double> lat_width_left_vector{LEFT_WIDTH_MARGIN};
      // std::vector<double> lat_width_right_vector{RIGHT_WIDTH_MARGIN};
      // std::vector<double> lat_width_d_vector{WIDTH_DELTA};

      // for (int target_vel_idx = 0;
      //      target_vel_idx < target_speed_vector_in.size(); target_vel_idx++)
      // {
      //   // set constant speed for globally planned wpt
      //   std::tuple<std::shared_ptr<FrenetPath>,
      //              std::vector<std::shared_ptr<FrenetPath>>>
      //       frenet_path_generation_result =
      //           m_frenet_generator->calc_frenet_paths_v2(
      //               current_position_d, 0.1 * cur_idx, 0.0,
      //               (target_speed_vector_in[target_vel_idx]), 0,
      //               cubic_spliner_2D_xy, LONGI_MIN_T, LONGI_MAX_T, LONGI_DT,
      //               (lat_width_left_vector[target_vel_idx]),
      //               (lat_width_right_vector[target_vel_idx]),
      //               (lat_width_d_vector[target_vel_idx]), 0);
      //   // opt_frenet_path = std::get<0>(frenet_path_generation_result);
      //   std::vector<std::shared_ptr<FrenetPath>> &frenet_paths =
      //       std::get<1>(frenet_path_generation_result);
      //   this->frenetPathsToPointCloud(frenet_paths);
      //   this->m_frenet_cost_calculator->setReferencePath(m_map_track_path_body);
      //   this->m_frenet_cost_calculator->setFrenetPathArray(frenet_paths);
      //   m_collision_avoidance_fp_body_ptr =
      //       this->m_frenet_cost_calculator->getMincostFrenetPath();
      // }

      // m_collision_avoidance_path_body.poses.clear();
      // m_collision_avoidance_path_body.header.frame_id = "base_link";
      // for (int i = 0; i <
      //                 m_collision_avoidance_fp_body_ptr->points_x().size();
      //      i++)
      // {
      //   geometry_msgs::msg::PoseStamped ps;
      //   ps.pose.position.x =
      //       m_collision_avoidance_fp_body_ptr->points_x()[i];
      //   ps.pose.position.y =
      //   m_collision_avoidance_fp_body_ptr->points_y()[i]; ps.pose.position.z
      //   = 0.0; ps.pose.orientation.x = 0.0; ps.pose.orientation.y = 0.0;
      //   ps.pose.orientation.z =
      //       sin(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
      //   ps.pose.orientation.w =
      //       cos(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
      //   m_collision_avoidance_path_body.poses.push_back(ps);
      // }
    }
    else if (m_cur_mission_code.mission_status_code ==
             m_cur_mission_code.MISSION_TIRE_WARMUP)
    {
      m_map_track_path_global =
          m_warmup_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_warmup_wpt_manager->getDesiredMapTrackInBody();
    }
    else if (m_cur_mission_code.mission_status_code ==
             m_cur_mission_code.MISSION_STANDBY)
    {
      // NOTE : check out of track
      if (m_on_track_flg)
      {
        // if the vehicle is on the track
        m_map_track_path_global =
            m_rl_wpt_manager->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_rl_wpt_manager->getDesiredMapTrackInBody();
      }
      else
      {
        // if the vehicle is out of the track
        m_map_track_path_global =
            m_pit_wpt_manager->getDesiredMapTrackInGlobal();
        m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

        // ///////////////////////
        // NOTE : collision check
        // NOTE : Need to load frenet generator default params. it can cause an
        // error.
        // ///////////////////////
        // std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
        //     m_frenet_generator->applyCubicSpliner_2d_ros(m_map_track_path_body,
        //                                                  1.0);
        // double cte = -1 * splined_y[0];
        // double target_vel =
        //     m_planning_vel_default + m_cur_odom.twist.twist.linear.x; //
        //     [m/s]
        // std::tuple<std::shared_ptr<FrenetPath>,
        //            std::vector<std::shared_ptr<FrenetPath>>>
        //     frenet_path_generation_result =
        //         m_frenet_generator->calcOptimalFrenetPathByMode(
        //             FRENET_GEN_MODE::MULTIPLE_LAT_FPS, cubic_spliner_2D_xy,
        //             0.0, 0.0, 0.0, m_cur_odom.twist.twist.linear.x, 0.0,
        //             0.0, 1.0, m_left_side_sampling_width,
        //             m_right_side_sampling_width, target_vel, 0.0, 0.0,
        //             m_sampling_width_d, m_planning_t, 0.1);
        // std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
        //     std::get<1>(frenet_path_generation_result);
        // // NOTE : must call the function "setReferencePath" beforehand
        // // "setFrenetPathArray"
        // this->m_frenet_cost_calculator->setReferencePath(m_map_track_path_body);
        // this->m_frenet_cost_calculator->setFrenetPathArray(frenet_paths);
        // m_collision_avoidance_fp_body_ptr =
        //     this->m_frenet_cost_calculator->getMincostFrenetPath();
        // this->frenetPathsToPointCloud(frenet_paths);

        // m_collision_avoidance_path_body.poses.clear();
        // m_collision_avoidance_path_body.header.frame_id = "base_link";
        // for (int i = 0;
        //      i < m_collision_avoidance_fp_body_ptr->points_x().size(); i++) {
        //   geometry_msgs::msg::PoseStamped ps;
        //   ps.pose.position.x =
        //   m_collision_avoidance_fp_body_ptr->points_x()[i];
        //   ps.pose.position.y =
        //   m_collision_avoidance_fp_body_ptr->points_y()[i];
        //   ps.pose.position.z = 0.0;
        //   ps.pose.orientation.x = 0.0;
        //   ps.pose.orientation.y = 0.0;
        //   ps.pose.orientation.z =
        //       sin(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
        //   ps.pose.orientation.w =
        //       sin(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
        //   m_collision_avoidance_path_body.poses.push_back(ps);
        // }
      }
    }
    else if (m_cur_mission_code.mission_status_code ==
                 m_cur_mission_code.MISSION_PIT_OUT ||
             m_cur_mission_code.mission_status_code ==
                 m_cur_mission_code.MISSION_PIT_STANDBY ||
             m_cur_mission_code.mission_status_code ==
                 m_cur_mission_code.MISSION_PIT_INIT)
    {
      m_map_track_path_global = m_pit_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_pit_wpt_manager->getDesiredMapTrackInBody();

      // ///////////////////////
      // NOTE : collision check
      // NOTE : Need to load frenet generator default params. it can cause an
      // error.
      // ///////////////////////
      //   std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
      //       m_frenet_generator->applyCubicSpliner_2d_ros(m_map_track_path_body,
      //                                                    1.0);
      //   double cte = -1 * splined_y[0];
      //   double target_vel =
      //       m_planning_vel_default + m_cur_odom.twist.twist.linear.x; //
      //       [m/s]
      //   std::tuple<std::shared_ptr<FrenetPath>,
      //              std::vector<std::shared_ptr<FrenetPath>>>
      //       frenet_path_generation_result =
      //           m_frenet_generator->calcOptimalFrenetPathByMode(
      //               FRENET_GEN_MODE::MULTIPLE_LAT_FPS, cubic_spliner_2D_xy,
      //               0.0, 0.0, 0.0, m_cur_odom.twist.twist.linear.x, 0.0,
      //               0.0, 1.0, m_left_side_sampling_width,
      //               m_right_side_sampling_width, target_vel, 0.0, 0.0,
      //               m_sampling_width_d, m_planning_t, 0.1);
      //   std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
      //       std::get<1>(frenet_path_generation_result);
      //   // NOTE : must call the function "setReferencePath" beforehand
      //   // "setFrenetPathArray"
      //   this->m_frenet_cost_calculator->setReferencePath(m_map_track_path_body);
      //   this->m_frenet_cost_calculator->setFrenetPathArray(frenet_paths);
      //   m_collision_avoidance_fp_body_ptr =
      //       this->m_frenet_cost_calculator->getMincostFrenetPath();
      //   this->frenetPathsToPointCloud(frenet_paths);

      //   m_collision_avoidance_path_body.poses.clear();
      //   m_collision_avoidance_path_body.header.frame_id = "base_link";
      //   for (int i = 0; i <
      //   m_collision_avoidance_fp_body_ptr->points_x().size();
      //        i++) {
      //     geometry_msgs::msg::PoseStamped ps;
      //     ps.pose.position.x =
      //     m_collision_avoidance_fp_body_ptr->points_x()[i];
      //     ps.pose.position.y =
      //     m_collision_avoidance_fp_body_ptr->points_y()[i];
      //     ps.pose.position.z = 0.0;
      //     ps.pose.orientation.x = 0.0;
      //     ps.pose.orientation.y = 0.0;
      //     ps.pose.orientation.z =
      //         sin(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
      //     ps.pose.orientation.w =
      //         sin(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
      //     m_collision_avoidance_path_body.poses.push_back(ps);
      //   }
    }
    else if (m_cur_mission_code.mission_status_code ==
             m_cur_mission_code.MISSION_SLOW_DRIVE)
    {
      m_map_track_path_global = m_rl_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_rl_wpt_manager->getDesiredMapTrackInBody();
    }
    else if (m_cur_mission_code.mission_status_code ==
             m_cur_mission_code.MISSION_COLLISION_AVOIDNACE)
    {
      // m_map_track_path_global =
      // m_rl_wpt_manager->getDesiredMapTrackInGlobal(); m_map_track_path_body =
      // m_rl_wpt_manager->getDesiredMapTrackInBody();

      // ///////////////////////
      // NOTE : collision check
      // NOTE : Need to load frenet generator default params. it can cause an
      // error.
      // ///////////////////////
      //   std::tie(splined_x, splined_y, splined_yaw, cubic_spliner_2D_xy) =
      //       m_frenet_generator->applyCubicSpliner_2d_ros(m_map_track_path_body,
      //                                                    1.0);
      //   double cte = -1 * splined_y[0];
      //   double target_vel =
      //       m_planning_vel_default + m_cur_odom.twist.twist.linear.x; //
      //       [m/s]
      //   std::tuple<std::shared_ptr<FrenetPath>,
      //              std::vector<std::shared_ptr<FrenetPath>>>
      //       frenet_path_generation_result =
      //           m_frenet_generator->calcOptimalFrenetPathByMode(
      //               FRENET_GEN_MODE::MULTIPLE_LAT_FPS, cubic_spliner_2D_xy,
      //               0.0, 0.0, 0.0, m_cur_odom.twist.twist.linear.x, 0.0,
      //               0.0, 1.0, m_left_side_sampling_width,
      //               m_right_side_sampling_width, target_vel, 0.0, 0.0,
      //               m_sampling_width_d, m_planning_t, 0.1);
      //   std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
      //       std::get<1>(frenet_path_generation_result);
      //   // NOTE : must call the function "setReferencePath" beforehand
      //   // "setFrenetPathArray"
      //   this->m_frenet_cost_calculator->setReferencePath(m_map_track_path_body);
      //   this->m_frenet_cost_calculator->setFrenetPathArray(frenet_paths);
      //   m_collision_avoidance_fp_body_ptr =
      //       this->m_frenet_cost_calculator->getMincostFrenetPath();
      //   this->frenetPathsToPointCloud(frenet_paths);

      //   m_collision_avoidance_path_body.poses.clear();
      //   m_collision_avoidance_path_body.header.frame_id = "base_link";
      //   for (int i = 0; i <
      //   m_collision_avoidance_fp_body_ptr->points_x().size();
      //        i++) {
      //     geometry_msgs::msg::PoseStamped ps;
      //     ps.pose.position.x =
      //     m_collision_avoidance_fp_body_ptr->points_x()[i];
      //     ps.pose.position.y =
      //     m_collision_avoidance_fp_body_ptr->points_y()[i];
      //     ps.pose.position.z = 0.0;
      //     ps.pose.orientation.x = 0.0;
      //     ps.pose.orientation.y = 0.0;
      //     ps.pose.orientation.z =
      //         sin(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
      //     ps.pose.orientation.w =
      //         sin(m_collision_avoidance_fp_body_ptr->yaw()[i] / 2.0);
      //     m_collision_avoidance_path_body.poses.push_back(ps);
      //   }

      // m_map_track_path_global = m_graph_based_path;
      int cur_idx =
          m_rl_wpt_manager->getCurrentIdx(m_graph_based_path, m_cur_odom);

      m_map_track_path_global.poses.clear();
      for (int i = cur_idx; i < m_graph_based_path.poses.size(); i++)
      {
        geometry_msgs::msg::PoseStamped ps;
        ps.pose.position.x = m_graph_based_path.poses[i].pose.position.x;
        ps.pose.position.y = m_graph_based_path.poses[i].pose.position.y;
        ps.pose.position.z = m_graph_based_path.poses[i].pose.position.z;

        ps.pose.orientation.x = m_graph_based_path.poses[i].pose.orientation.x;
        ps.pose.orientation.y = m_graph_based_path.poses[i].pose.orientation.y;
        ps.pose.orientation.z = m_graph_based_path.poses[i].pose.orientation.z;
        ps.pose.orientation.w = m_graph_based_path.poses[i].pose.orientation.w;

        m_map_track_path_global.poses.push_back(ps);
      }

      m_map_track_path_body.poses.clear();
      m_map_track_path_body =
          nif::common::utils::coordination::getPathGlobaltoBody(
              m_cur_odom, m_map_track_path_global);
    }
    else
    {
      // nominal case
      m_map_track_path_global = m_rl_wpt_manager->getDesiredMapTrackInGlobal();
      m_map_track_path_body = m_rl_wpt_manager->getDesiredMapTrackInBody();
    }
  }
  else
  {
    // odom is not setted yet
  }
}

void WaypointManagerMissionV2::setCollisionAvoidanceGraphPath(
    const nav_msgs::msg::Path &coll_free_msg)
{
  // TODO : check whether it is in the global frame
  m_graph_based_path = coll_free_msg;
}

void WaypointManagerMissionV2::frenetPathsToPointCloud(
    std::vector<std::shared_ptr<FrenetPath>> &frenet_paths)
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (int i = 0; i < frenet_paths.size(); i++)
  {
    int count = 0;
    for (int j = 0; j < int(frenet_paths[i]->points_x().size() - 1); j++)
    {
      std::shared_ptr<FrenetPath> &frenet_path = frenet_paths[i];

      pcl::PointXYZI current_point;
      current_point.x = frenet_path->points_x()[j];
      current_point.y = frenet_path->points_y()[j];
      current_point.z = 0;
      current_point.intensity = count;
      cloud_ptr->points.push_back(current_point);
      count++;
    }
  }
  pcl::toROSMsg(*cloud_ptr, m_frenet_candidates_pc);
  m_frenet_candidates_pc.header.frame_id = "base_link";
}