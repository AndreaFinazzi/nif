#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include "imm/imm_ukf_pda.h"
#include "imm/ukf.h"
#include "rclcpp/rclcpp.hpp"
#include "nif_msgs/msg/detected_object.hpp"
#include "nif_msgs/msg/detected_object_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include "rcutils/error_handling.h"
#include <stdlib.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

class IMMObjectTrackerNode : public rclcpp::Node
{
public:
    IMMObjectTrackerNode(const std::string &node_name_);
    IMMObjectTrackerNode(const std::string &node_name_,
                         const std::string &tracker_config_file_path_);

private:
    void detectionCallback(const nif_msgs::msg::DetectedObjectArray::SharedPtr msg);
    void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<nif_msgs::msg::DetectedObjectArray>::SharedPtr tracked_output_publisher;
    rclcpp::Subscription<nif_msgs::msg::DetectedObjectArray>::SharedPtr detection_result_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_subscription;

    // tracker related
    std::string tracker_config_file_path;
    std::shared_ptr<ImmUkfPda> tracker_ptr;

    nif_msgs::msg::DetectedObjectArray tracked_objects;

    nav_msgs::msg::Odometry ego_odom;
};

#endif
