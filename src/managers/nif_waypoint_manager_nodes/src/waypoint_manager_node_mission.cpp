//
// Created by usrg on 7/10/21.
//

#include "nif_waypoint_manager_nodes/waypoint_manager_node_mission.h"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"
#include <stdlib.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

nif::managers::WaypointManagerMissionNode::WaypointManagerMissionNode(
    const std::string &node_name_)
    : IBaseNode(node_name_, common::NodeType::PLANNING)
{
  std::string package_share_directory;

  try
  {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_waypoint_manager_nodes");
  }
  catch (std::exception e)
  {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }
  package_share_directory = package_share_directory.append("/");

  this->declare_parameter("race_wpt_path", race_wpt_file_path);
  this->declare_parameter("warmup_wpt_path", warmup_wpt_file_path);
  this->declare_parameter("pit_wpt_path", pit_wpt_file_path);
  this->declare_parameter("spline_interval", spline_interval);
  this->declare_parameter("maptrack_size", maptrack_size);
  this->declare_parameter("maptrack_size_safety_threshold", 20);

  this->race_wpt_file_path = this->get_parameter("race_wpt_path").as_string();
  this->warmup_wpt_file_path =
      this->get_parameter("warmup_wpt_path").as_string();
  this->pit_wpt_file_path = this->get_parameter("pit_wpt_path").as_string();
  this->spline_interval = this->get_parameter("spline_interval").as_double();
  this->maptrack_size = this->get_parameter("maptrack_size").as_int();
  this->maptrack_size_safety_threshold =
      this->get_parameter("maptrack_size_safety_threshold").as_int();

  // Could also inherit from IBaseSynchronizedNode
  //  TODO convert to parameter
  m_timer = this->create_wall_timer(
      20ms, std::bind(&WaypointManagerMissionNode::timerCallback, this));

  m_map_track_global_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "wpt_manager/maptrack_path/global", nif::common::constants::QOS_PLANNING);
  m_map_track_body_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "wpt_manager/maptrack_path/body", nif::common::constants::QOS_PLANNING);
  m_frenet_candidates_publisher =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "wpt_manager/frenet_candidates/body",
          nif::common::constants::QOS_PLANNING);
  m_frenet_min_cost_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "wpt_manager/mincost_frenet/body", nif::common::constants::QOS_PLANNING);
  m_occu_map_subscriber =
      this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/semantics/costmap_generator/occupancy_grid",
          nif::common::constants::QOS_PLANNING,
          std::bind(&WaypointManagerMissionNode::occupancyCallback, this,
                    std::placeholders::_1));
  m_graph_planner = this->create_subscription<nav_msgs::msg::Path>(
      "/graph_planner/final_path", nif::common::constants::QOS_PLANNING,
      std::bind(&WaypointManagerMissionNode::graphCallback, this,
                std::placeholders::_1));

  this->race_wpt_file_path.insert(0, package_share_directory);
  this->warmup_wpt_file_path.insert(0, package_share_directory);
  this->pit_wpt_file_path.insert(0, package_share_directory);

  // this->setWaypointManager(std::make_shared<WaypointManagerMissionV2>(
  //     this->race_wpt_file_path, this->pit_wpt_file_path,
  //     this->getBodyFrameId(), this->getGlobalFrameId(),
  //     this->spline_interval));
  this->setWaypointManager(std::make_shared<WaypointManagerMissionV2>(
      this->race_wpt_file_path, this->warmup_wpt_file_path,
      this->pit_wpt_file_path, "base_link", "odom", this->spline_interval));

  this->setNodeStatus(common::NODE_INITIALIZED);
}

void nif::managers::WaypointManagerMissionNode::graphCallback(
    const nav_msgs::msg::Path::SharedPtr graph_path_msg)
{
  m_graph_path_in_global = *graph_path_msg;
  this->wpt_manager->setCollisionAvoidanceGraphPath(m_graph_path_in_global);
  // std::cout << "sibal" << std::endl;
}

void nif::managers::WaypointManagerMissionNode::occupancyCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_msg)
{
  this->wpt_manager->setOccupancyGridMap(*occupancy_map_msg);
}

// TODO should pass node_name_ as a reference here
// TODO IBaseNode should be initialized first
nif::managers::WaypointManagerMissionNode::WaypointManagerMissionNode(
    const std::string &node_name_,
    const std::shared_ptr<WaypointManagerMissionV2> wpt_manager_ptr)
    : WaypointManagerMissionNode(node_name_)
{
  this->setWaypointManager(wpt_manager_ptr);
}

void nif::managers::WaypointManagerMissionNode::timerCallback()
try
{
  //  RCLCPP_DEBUG(this->get_logger(), "WaypointManagerMissionNode timer
  //  callback");

  this->wpt_manager->setSystemStatus(this->getSystemStatus());
  this->wpt_manager->setCurrentOdometry(this->getEgoOdometry());

  // TODO : for visualiztion - test
  this->wpt_manager->calcMapTrack();
  m_frenet_candidates_publisher->publish(
      this->wpt_manager->getFrenetCandidatesAsPc());
  m_frenet_min_cost_publisher->publish(
      this->wpt_manager->getMinCostFrenetPath());

  nav_msgs::msg::Path path_in_global =
      this->wpt_manager->getDesiredMapTrackInGlobal();
  nav_msgs::msg::Path path_in_body =
      this->wpt_manager->getDesiredMapTrackInBody();
  path_in_body.header.stamp = this->now();
  path_in_body.header.frame_id = "base_link";
  path_in_global.header.stamp = this->now();
  path_in_global.header.frame_id = "odom";

  m_map_track_global_publisher->publish(path_in_global);
  m_map_track_body_publisher->publish(path_in_body);

  // m_frenet_candidates_publisher->publish(
  //     this->wpt_manager->getFrenetCandidatesAsPc());
  // m_frenet_min_cost_publisher->publish(this->wpt_manager->getMinCostFrenetPath());

  if (path_in_body.poses.size() >= this->maptrack_size_safety_threshold &&
      path_in_global.poses.size() >= this->maptrack_size_safety_threshold)
  {
    this->setNodeStatus(common::NODE_OK);
  }
  else
  {
    this->setNodeStatus(common::NODE_ERROR);
  }
}
catch (std::exception &e)
{
  RCLCPP_ERROR(this->get_logger(), e.what());
  this->setNodeStatus(common::NODE_FATAL_ERROR);
}
catch (...)
{
  RCLCPP_ERROR(this->get_logger(),
               "Unknown exception thrown in WaypointManagerMissionNode.");
  this->setNodeStatus(common::NODE_FATAL_ERROR);
}