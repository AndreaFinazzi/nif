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
  : IBaseNode(node_name_) {
  std::string package_share_directory;

  try {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_waypoint_manager_nodes");
  } catch (std::exception e) {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }
  std::vector<std::string> file_path_list_defualt = {package_share_directory +
                                                     "/maps/map.csv"};
  this->declare_parameter("file_path_list", file_path_list_defualt);
  this->declare_parameter("body_frame_id", "base_link");
  this->declare_parameter("global_frame_id", "odom");

  this->file_path_list =
      this->get_parameter("file_path_list").as_string_array();
  this->body_frame_id = this->get_parameter("body_frame_id").as_string();
  this->global_frame_id = this->get_parameter("global_frame_id").as_string();

  // Could also inherit from IBaseSynchronizedNode
  m_timer = this->create_wall_timer(
      10ms, std::bind(&WaypointManagerNode::timer_callback, this));

  m_map_track_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "nif/wpt_manager/maptrack_path", 10);

  this->setWaypointManager(std::make_shared<WaypointManagerMinimal>(
      file_path_list, body_frame_id, global_frame_id));
}

// TODO should pass node_name_ as a reference here
// TODO IBaseNode should be initialized first
nif::managers::WaypointManagerNode::WaypointManagerNode(
    const std::string& node_name_,
    const std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr)
  : WaypointManagerNode(node_name_) {
  this->setWaypointManager(wpt_manager_ptr);
}

void nif::managers::WaypointManagerNode::timer_callback() {
  RCLCPP_DEBUG(this->get_logger(), "WaypointManagerNode timer callback");
  nav_msgs::msg::Path maptrack;

  this->wpt_manager->setCurrentPose(this->ego_odometry);
  maptrack = this->wpt_manager->getDesiredMapTrackInGlobal();
  m_map_track_publisher->publish(maptrack);
}

void nif::managers::WaypointManagerNode::initParameters() {}
void nif::managers::WaypointManagerNode::getParameters() {}
