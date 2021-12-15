//
// Created by usrg on 7/10/21.
//

#include "../include/nif_objects_tracker_nodes/objects_tracker_nodes.h"

IMMObjectTrackerNode::IMMObjectTrackerNode(const std::string &node_name_)
    : Node(node_name_) {
  std::string package_share_directory;
  try {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_objects_tracker_nodes");
  } catch (std::exception e) {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }
  package_share_directory = package_share_directory.append("/");
  this->declare_parameter("tracking_config_file_path",
                          tracker_config_file_path);
  tracker_config_file_path =
      this->get_parameter("tracking_config_file_path").as_string();
  tracker_config_file_path.insert(0, package_share_directory);

  tracked_output_publisher =
      this->create_publisher<nif_msgs::msg::Perception3DArray>(
          "output_topic_name", 10);
  detection_result_subscription =
      this->create_subscription<nif_msgs::msg::DetectedObjectArray>(
          "input_topic_name", 10,
          std::bind(&IMMObjectTrackerNode::detectionCallback, this,
                    std::placeholders::_1));
  ego_odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "ego_odom_topic_name", 10,
      std::bind(&IMMObjectTrackerNode::egoOdomCallback, this,
                std::placeholders::_1));

  tracker_ptr = std::make_shared<ImmUkfPda>(tracker_config_file_path);
  // success to initialize the tracker
}

IMMObjectTrackerNode::IMMObjectTrackerNode(
    const std::string &node_name_, const std::string &tracker_config_file_path_)
    : Node(node_name_), tracker_config_file_path(tracker_config_file_path_) {
  tracked_output_publisher =
      this->create_publisher<nif_msgs::msg::Perception3DArray>(
          "output_topic_name", 10);
  detection_result_subscription =
      this->create_subscription<nif_msgs::msg::DetectedObjectArray>(
          "input_topic_name", 10,
          std::bind(&IMMObjectTrackerNode::detectionCallback, this,
                    std::placeholders::_1));
  ego_odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
      "ego_odom_topic_name", 10,
      std::bind(&IMMObjectTrackerNode::egoOdomCallback, this,
                std::placeholders::_1));

  tracker_ptr = std::make_shared<ImmUkfPda>(tracker_config_file_path);
  // success to initialize the tracker
}

void IMMObjectTrackerNode::detectionCallback(
    const nif_msgs::msg::Perception3DArray::SharedPtr msg) {
  // TODO : Set the latest ego position to the tracker
  tracker_ptr->setEgoOdom(ego_odom);
  // TODO : Set the detection result to the tracker
  // TODO : TOPIC TYPE CONVERSION : Perception3DArray to DetectionArray
  nif_msgs::msg::DetectedObjectArray tmp;

  tmp.header = msg->header;
  for (int det_idx = 0; det_idx < msg->perception_list.size(); det_idx++) {
    tmp.objects[det_idx].pose.position.x =
        msg->perception_list[det_idx].detection_result_3d.center.position.x;
    tmp.objects[det_idx].pose.position.y =
        msg->perception_list[det_idx].detection_result_3d.center.position.y;
    tmp.objects[det_idx].pose.position.z =
        msg->perception_list[det_idx].detection_result_3d.center.position.z;
  }

  tracked_objects = tracker_ptr->getTrackedResult(tmp);
  tracked_output_publisher->publish(tracked_objects);
}

void IMMObjectTrackerNode::egoOdomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  ego_odom = *msg;
}