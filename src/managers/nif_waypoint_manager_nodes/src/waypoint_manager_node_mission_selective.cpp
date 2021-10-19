//
// Created by usrg on 7/10/21.
//

#include "nif_waypoint_manager_nodes/waypoint_manager_node_mission_selective.h"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"
#include <stdlib.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

nif::managers::WaypointManagerMissionSelectiveNode::
    WaypointManagerMissionSelectiveNode(const std::string &node_name_)
    : IBaseNode(node_name_, common::NodeType::PLANNING) {

  std::string package_share_directory;

  try {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_waypoint_manager_nodes");
  } catch (std::exception e) {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }
  package_share_directory = package_share_directory.append("/maps/LOR/");

  // this->declare_parameter("race_wpt_path", race_wpt_file_path);
  // this->declare_parameter("pit_wpt_path", pit_wpt_file_path);
  // this->declare_parameter("spline_interval", spline_interval);
  // this->declare_parameter("maptrack_size", maptrack_size);
  // this->declare_parameter("maptrack_size_safety_threshold", 20);

  // this->race_wpt_file_path =
  // this->get_parameter("race_wpt_path").as_string(); this->pit_wpt_file_path =
  // this->get_parameter("pit_wpt_path").as_string(); this->spline_interval =
  // this->get_parameter("spline_interval").as_double(); this->maptrack_size =
  // this->get_parameter("maptrack_size").as_int();
  // this->maptrack_size_safety_threshold =
  //     this->get_parameter("maptrack_size_safety_threshold").as_int();

  // Could also inherit from IBaseSynchronizedNode
  //  TODO convert to parameter
  m_timer = this->create_wall_timer(
      10ms,
      std::bind(&WaypointManagerMissionSelectiveNode::timerCallback, this));

  m_map_track_global_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "wpt_manager/maptrack_path/global", nif::common::constants::QOS_PLANNING);
  m_map_track_body_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "wpt_manager/maptrack_path/body", nif::common::constants::QOS_PLANNING);

  m_obstacle_subscriber =
      this->create_subscription<visualization_msgs::msg::MarkerArray>(
          "/clustered_markers", nif::common::constants::QOS_PLANNING,
          std::bind(&WaypointManagerMissionSelectiveNode::obstacleCallback,
                    this, std::placeholders::_1));

  // this->race_wpt_file_path.insert(0, package_share_directory);
  // this->pit_wpt_file_path.insert(0, package_share_directory);

  std::string config_file_name = "race_line_param.yaml";
  config_file_name.insert(0, package_share_directory);

  // this->setWaypointManager(std::make_shared<WaypointManagerMissionSelective>(
  //     config_file_name, this->getBodyFrameId(), this->getGlobalFrameId(),
  //     this->spline_interval));

  std::string raceline = "race_line.csv";
  std::string centerline = "center_line.csv";
  std::string pitline = "pit_lane.csv";

  raceline.insert(0, package_share_directory);
  centerline.insert(0, package_share_directory);
  pitline.insert(0, package_share_directory);

  this->setWaypointManager(std::make_shared<WaypointManagerMissionSelective>(
      raceline, centerline, pitline, this->getBodyFrameId(),
      this->getGlobalFrameId(), this->spline_interval));

  this->setNodeStatus(common::NODE_INITIALIZED);
}

// TODO should pass node_name_ as a reference here
// TODO IBaseNode should be initialized first
nif::managers::WaypointManagerMissionSelectiveNode::
    WaypointManagerMissionSelectiveNode(
        const std::string &node_name_,
        const std::shared_ptr<WaypointManagerMissionSelective> wpt_manager_ptr)
    : WaypointManagerMissionSelectiveNode(node_name_) {
  this->setWaypointManager(wpt_manager_ptr);
}

void nif::managers::WaypointManagerMissionSelectiveNode::obstacleCallback(
    const visualization_msgs::msg::MarkerArray::SharedPtr obj_array) {
  this->wpt_manager->setObstacle(*obj_array);
}

void nif::managers::WaypointManagerMissionSelectiveNode::timerCallback() try {
  //  RCLCPP_DEBUG(this->get_logger(), "WaypointManagerMissionSelectiveNode
  //  timer callback");

  this->wpt_manager->setSystemStatus(this->getSystemStatus());
  this->wpt_manager->setCurrentOdometry(this->getEgoOdometry());

  nav_msgs::msg::Path path_in_global =
      this->wpt_manager->getDesiredMapTrackInGlobal();
  nav_msgs::msg::Path path_in_body =
      this->wpt_manager->getDesiredMapTrackInBody();

  path_in_body.header.stamp = this->now();
  path_in_body.header.frame_id = this->getBodyFrameId();
  path_in_global.header.stamp = this->now();
  path_in_global.header.frame_id = this->getGlobalFrameId();

  m_map_track_global_publisher->publish(path_in_global);
  m_map_track_body_publisher->publish(path_in_body);

  if (path_in_body.poses.size() >= this->maptrack_size_safety_threshold &&
      path_in_global.poses.size() >= this->maptrack_size_safety_threshold) {
    this->setNodeStatus(common::NODE_OK);
  } else {
    this->setNodeStatus(common::NODE_ERROR);
  }
} catch (std::exception &e) {
  RCLCPP_ERROR(this->get_logger(), e.what());
  this->setNodeStatus(common::NODE_FATAL_ERROR);
} catch (...) {
  RCLCPP_ERROR(
      this->get_logger(),
      "Unknown exception thrown in WaypointManagerMissionSelectiveNode.");
  this->setNodeStatus(common::NODE_FATAL_ERROR);
}