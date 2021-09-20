//
// Created by usrg on 7/10/21.
//

#ifndef ROS2MASTER_WAYPOINT_MANAGER_NODE_H
#define ROS2MASTER_WAYPOINT_MANAGER_NODE_H

#include "nif_common_nodes/i_base_node.h"
#include "nif_waypoint_manager_common/i_waypoint_manager.h"
#include "nif_waypoint_manager_minimal/waypoint_manager_minimal.h"

#include "rclcpp/rclcpp.hpp"

namespace nif
{
  namespace managers
  {

    class WaypointManagerNode : public nif::common::IBaseNode
    {
    public:
      /**
   *
   * Using default WaypointManager -> WaypointManagerMinimal
   *
   **/

      // TODO  wpt_file_path_list_, body_frame_id_ and global_frame_id_ could be passed as rosparams
      explicit WaypointManagerNode(const std::string &node_name_);

      WaypointManagerNode(const std::string &node_name_,
                          const std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr);

    private:
      WaypointManagerNode();
      void timerCallback();

      void setWaypointManager(const std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr)
      {
        this->wpt_manager = wpt_manager_ptr;
      }

      std::vector<std::string> file_path_list{};

      std::shared_ptr<WaypointManagerMinimal> wpt_manager;

      rclcpp::TimerBase::SharedPtr m_timer;

      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_map_track_global_publisher;
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_map_track_body_publisher;

      unsigned short int maptrack_size_safety_threshold = 1;
    };

} // namespace managers
} // namespace nif
#endif // ROS2MASTER_WAYPOINT_MANAGER_NODE_H
