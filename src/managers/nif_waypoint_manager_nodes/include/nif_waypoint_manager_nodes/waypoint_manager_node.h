//
// Created by usrg on 7/10/21.
//

#ifndef ROS2MASTER_WAYPOINT_MANAGER_NODE_H
#define ROS2MASTER_WAYPOINT_MANAGER_NODE_H

#include "nif_common_nodes/i_base_node.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"
#include "nif_waypoint_manager_minimal/waypoint_manager_minimal.h"

#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

class WaypointManagerNode : public nif::common::IBaseNode {
public:
  /**
   *
   * Using default WaypointManager -> WaypointManagerMinimal
   *
   **/
  WaypointManagerNode(std::string& node_name_,
                      vector<string>& wpt_file_path_list_,
                      string& body_frame_id_,
                      string& global_frame_id_);

  WaypointManagerNode(std::string& node_name_,
                      std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr);

private:
  WaypointManagerNode();
  void timer_callback();
  void initParameters();
  void getParameters();

  std::shared_ptr<WaypointManagerMinimal> wpt_manager;

  rclcpp::TimerBase::SharedPtr timer_;
  nif::common::msgs::Odometry m_current_pose;
};

#endif // ROS2MASTER_WAYPOINT_MANAGER_NODE_H
