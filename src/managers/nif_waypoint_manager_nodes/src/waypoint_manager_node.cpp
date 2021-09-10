//
// Created by usrg on 7/10/21.
//

#include "nif_waypoint_manager_nodes/waypoint_manager_node.h"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

nif::managers::WaypointManagerNode::WaypointManagerNode(
    const std::string& node_name_)
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
  package_share_directory = package_share_directory.append("/");
  std::vector<std::string> file_path_list_default = {"maps/map.csv"};
  double spline_interval = 1.0;
  int maptrack_size = 100;
  this->declare_parameter("file_path_list", file_path_list_default);
  this->declare_parameter("spline_interval", spline_interval);
  this->declare_parameter("maptrack_size", maptrack_size);

  this->file_path_list =
      this->get_parameter("file_path_list").as_string_array();
  spline_interval = this->get_parameter("spline_interval").as_double();
  maptrack_size = this->get_parameter("maptrack_size").as_int();

  for (auto& path : file_path_list) {
    path.insert(0, package_share_directory);
  }

  // Could also inherit from IBaseSynchronizedNode
//  TODO convert to parameter
  m_timer = this->create_wall_timer(
      10ms, std::bind(&WaypointManagerNode::timerCallback, this));

  m_map_track_global_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "wpt_manager/maptrack_path/global", nif::common::constants::QOS_PLANNING);

  m_map_track_body_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "wpt_manager/maptrack_path/body", nif::common::constants::QOS_PLANNING);
  this->setWaypointManager(
      std::make_shared<WaypointManagerMinimal>(file_path_list,
                                               this->getBodyFrameId(),
                                               this->getGlobalFrameId(),
                                               spline_interval));

  this->wpt_manager->setSizeOfMapTrack(maptrack_size); // index level
}

// TODO should pass node_name_ as a reference here
// TODO IBaseNode should be initialized first
nif::managers::WaypointManagerNode::WaypointManagerNode(
    const std::string& node_name_,
    const std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr)
  : WaypointManagerNode(node_name_) {
  this->setWaypointManager(wpt_manager_ptr);
}

void nif::managers::WaypointManagerNode::timerCallback() {
  RCLCPP_DEBUG(this->get_logger(), "WaypointManagerNode timer callback");
  nav_msgs::msg::Path& path_in_global =
      this->wpt_manager->getDesiredMapTrackInGlobal();
  nav_msgs::msg::Path& path_in_body =
      this->wpt_manager->getDesiredMapTrackInBody();

  path_in_body.header.stamp = this->now();
  path_in_body.header.frame_id = this->getBodyFrameId();
  path_in_global.header.stamp = this->now();
  path_in_global.header.frame_id = this->getGlobalFrameId();

  m_map_track_global_publisher->publish(path_in_global);
  m_map_track_body_publisher->publish(path_in_body);
  this->wpt_manager->setCurrentOdometry(this->getEgoOdometry());
}

void nif::managers::WaypointManagerNode::initParameters() {}
void nif::managers::WaypointManagerNode::getParameters() {}
