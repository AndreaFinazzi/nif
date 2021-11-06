//
// Created by usrg on 7/10/21.
//

#ifndef ROS2MASTER_WAYPOINT_MANAGER_NODE_MISSION_SELECTIVE_H
#define ROS2MASTER_WAYPOINT_MANAGER_NODE_MISSION_SELECTIVE_H

#include "nif_common_nodes/i_base_node.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"
#include "nif_waypoint_manager_mission_selective/waypoint_manager_mission_selective.h"

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nif {
namespace managers {

class WaypointManagerMissionSelectiveNode : public nif::common::IBaseNode {
public:
  /**
   *
   * Using default WaypointManager -> WaypointManagerMission
   *
   **/

  // TODO  wpt_file_path_list_, body_frame_id_ and global_frame_id_ could be
  // passed as rosparams
  explicit WaypointManagerMissionSelectiveNode(const std::string &node_name_);

  WaypointManagerMissionSelectiveNode(
      const std::string &node_name_,
      const std::shared_ptr<WaypointManagerMissionSelective> wpt_manager_ptr);

private:
  WaypointManagerMissionSelectiveNode();
  void timerCallback();
  void obstacleCallback(
      const visualization_msgs::msg::MarkerArray::SharedPtr obj_array);

  void setWaypointManager(
      const std::shared_ptr<WaypointManagerMissionSelective> wpt_manager_ptr) {
    this->wpt_manager = wpt_manager_ptr;
  }

  std::string race_wpt_file_path = "";
  std::string pit_wpt_file_path = "";
  double spline_interval = 1.0;
  int maptrack_size = 100;

  std::shared_ptr<WaypointManagerMissionSelective> wpt_manager;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      m_map_track_global_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_map_track_body_publisher;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr
      m_obstacle_subscriber;

  unsigned short int maptrack_size_safety_threshold = 1;
};

} // namespace managers
} // namespace nif
#endif // ROS2MASTER_WAYPOINT_MANAGER_NODE_MISSION_SELECTIVE_H
