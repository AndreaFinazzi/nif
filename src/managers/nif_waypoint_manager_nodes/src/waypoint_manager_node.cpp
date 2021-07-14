//
// Created by usrg on 7/10/21.
//

#include "nif_waypoint_manager_nodes/waypoint_manager_node.h"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"


WaypointManagerNode::WaypointManagerNode(
    std::string& node_name_,
    std::vector<string>& wpt_file_path_list_,
    std::string& body_frame_id_,
    std::string& global_frame_id_)
  : WaypointManagerNode(
        node_name_,
        std::make_shared<WaypointManagerMinimal>(
            wpt_file_path_list_, body_frame_id_, global_frame_id_)) {}

// TODO should pass node_name_ as a reference here
// TODO IBaseNode should be initialized first
WaypointManagerNode::WaypointManagerNode(
    std::string& node_name_,
    std::shared_ptr<WaypointManagerMinimal> wpt_manager_ptr)
  : wpt_manager(wpt_manager_ptr), IBaseNode(node_name_) {
  m_timer = this->create_wall_timer(
      10ms, std::bind(&WaypointManagerNode::timer_callback, this));
  m_map_track_publisher = this->create_publisher<nav_msgs::msg::Path>(
      "nif/wpt_manager/maptrack_path", 10);
}

void WaypointManagerNode::timer_callback() {
  RCLCPP_INFO(this->get_logger(), "WaypointManagerNode timer callback");
  nav_msgs::msg::Path maptrack;
  this->wpt_manager->setCurrentPose(this->ego_odometry);
  maptrack = this->wpt_manager->getDesiredMapTrackInGlobal();
  m_map_track_publisher->publish(maptrack);
}

void WaypointManagerNode::initParameters() {}
void WaypointManagerNode::getParameters() {}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  //   using nif::control::ControlSafetyLayerNode;
  using namespace nif::common::constants;

  string node_name = "wappoint_manager";

  vector<string> file_path_list;
  string body_frame_id_test = "";
  string global_frame_id_test = "";
  const std::chrono::microseconds sync_period(10000); //  10ms

  rclcpp::Node::SharedPtr nd;

  try {
    RCLCPP_INFO(
        rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
        "Instantiating WaypointManagerNode with name: %s; sync_period: %d",
        node_name,
        sync_period);
    nd = std::make_shared<WaypointManagerNode>(
        node_name, file_path_list, body_frame_id_test, global_frame_id_test);

  } catch (std::range_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "Bad initialization of node: %s. Initializing with "
                 "SYNC_PERIOD_DEFAULT...\n%s",
                 e.what());

    //  Initialize with default period.
    //  TODO should we abort in these circumstances?
    nd = std::make_shared<WaypointManagerNode>(
        node_name, file_path_list, body_frame_id_test, global_frame_id_test);

  } catch (std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
                 "FATAL ERROR during node initialization: ABORTING.\n%s",
                 e.what());
    return -1;
  }

  rclcpp::spin(nd);
  rclcpp::shutdown();

  RCLCPP_INFO(rclcpp::get_logger(LOG_MAIN_LOGGER_NAME),
              "Shutting down %s [WaypointManagerNode]",
              node_name);

  return 0;
}