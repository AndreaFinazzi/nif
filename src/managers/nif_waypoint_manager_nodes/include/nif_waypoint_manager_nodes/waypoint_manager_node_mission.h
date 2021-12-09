//
// Created by usrg on 7/10/21.
//

#ifndef ROS2MASTER_WAYPOINT_MANAGER_NODE_MISSION_H
#define ROS2MASTER_WAYPOINT_MANAGER_NODE_MISSION_H

#include "nif_common_nodes/i_base_node.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"
#include "nif_waypoint_manager_mission/waypoint_manager_mission.h"
#include "nif_waypoint_manager_mission/waypoint_manager_mission_v2.h"

#include "rclcpp/rclcpp.hpp"

namespace nif {
namespace managers {

class WaypointManagerMissionNode : public nif::common::IBaseNode {
public:
  /**
   *
   * Using default WaypointManager -> WaypointManagerMissionV2
   *
   **/

  // TODO  wpt_file_path_list_, body_frame_id_ and global_frame_id_ could be
  // passed as rosparams
  explicit WaypointManagerMissionNode(const std::string &node_name_);

  WaypointManagerMissionNode(
      const std::string &node_name_,
      const std::shared_ptr<WaypointManagerMissionV2> wpt_manager_ptr);

  void occupancyCallback(
      const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map_msg);
  void graphCallback(const nav_msgs::msg::Path::SharedPtr graph_path_msg);

private:
  WaypointManagerMissionNode();
  void timerCallback();

  void missionStatusCallback(const nif_msgs::msg::SystemStatus &sys_status);

  void setWaypointManager(
      const std::shared_ptr<WaypointManagerMissionV2> wpt_manager_ptr) {
    this->wpt_manager = wpt_manager_ptr;
  }

  std::string race_wpt_file_path = "";
  std::string warmup_wpt_file_path = "";
  std::string pit_wpt_file_path = "";
  double spline_interval = 1.0;
  int maptrack_size = 100;

  std::shared_ptr<WaypointManagerMissionV2> wpt_manager;

  rclcpp::TimerBase::SharedPtr m_timer;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      m_map_track_global_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_map_track_body_publisher;

  // frenet path candidates visualization
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      m_frenet_candidates_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_frenet_min_cost_publisher;

  // Potential map subscribe
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr
      m_occu_map_subscriber;

  // graph planner
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_graph_planner;

  unsigned short int maptrack_size_safety_threshold = 1;

  nav_msgs::msg::Path m_graph_path_in_global;
};

} // namespace managers
} // namespace nif
#endif // ROS2MASTER_WAYPOINT_MANAGER_NODE_MISSION_H
