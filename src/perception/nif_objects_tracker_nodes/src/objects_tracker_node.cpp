//
// Created by usrg on 7/10/21.
//

#include "../include/nif_objects_tracker_nodes/objects_tracker_nodes.h"

IMMObjectTrackerNode::IMMObjectTrackerNode(const std::string &node_name_)
    : Node(node_name_)
{
    std::string package_share_directory;
    try
    {
        // This value shouldn't be used, it's as a backup if a config param is
        // missing.
        package_share_directory = ament_index_cpp::get_package_share_directory(
            "nif_objects_tracker_nodes");
    }
    catch (std::exception e)
    {
        RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
    }
    package_share_directory = package_share_directory.append("/");
    this->declare_parameter("tracking_config_file_path", tracker_config_file_path);
    tracker_config_file_path = this->get_parameter("tracking_config_file_path").as_string();
    tracker_config_file_path.insert(0, package_share_directory);

    // std::cout << package_share_directory << std::endl;
    // std::cout << tracker_config_file_path << std::endl;

    tracked_output_publisher = this->create_publisher<nif_msgs::msg::DetectedObjectArray>("output_topic_name", 10);
    detection_result_subscription = this->create_subscription<nif_msgs::msg::DetectedObjectArray>(
        "input_topic_name", 10, std::bind(&IMMObjectTrackerNode::detectionCallback, this, std::placeholders::_1));
    ego_odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_odom_topic_name", 10, std::bind(&IMMObjectTrackerNode::egoOdomCallback, this, std::placeholders::_1));

    tracker_ptr = std::make_shared<ImmUkfPda>(tracker_config_file_path);

    // success to initialize the tracker
}

IMMObjectTrackerNode::IMMObjectTrackerNode(const std::string &node_name_, const std::string &tracker_config_file_path_)
    : Node(node_name_), tracker_config_file_path(tracker_config_file_path_)
{
    tracked_output_publisher = this->create_publisher<nif_msgs::msg::DetectedObjectArray>("output_topic_name", 10);
    detection_result_subscription = this->create_subscription<nif_msgs::msg::DetectedObjectArray>(
        "input_topic_name", 10, std::bind(&IMMObjectTrackerNode::detectionCallback, this, std::placeholders::_1));
    ego_odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "ego_odom_topic_name", 10, std::bind(&IMMObjectTrackerNode::egoOdomCallback, this, std::placeholders::_1));

    std::cout << "here 0" << std::endl;

    tracker_ptr = std::make_shared<ImmUkfPda>(tracker_config_file_path);

    std::cout << "here 1" << std::endl;
    // success to initialize the tracker
}

void IMMObjectTrackerNode::detectionCallback(const nif_msgs::msg::DetectedObjectArray::SharedPtr msg)
{
    // TODO 0 : set the latest ego position to the tracker
    tracker_ptr->setEgoOdom(ego_odom);
    // TODO 1 : set the detection result to the tracker
    // tracker_ptr->setDetectionResult(*msg);
    // TODO 2 : set the detection result to the tracker
    tracked_objects = tracker_ptr->getTrackedResult(*msg);
}

void IMMObjectTrackerNode::egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ego_odom = *msg;
}