#ifndef OBJECT_TRACKER_H
#define OBJECT_TRACKER_H

#include "ament_index_cpp/get_package_prefix.hpp"
#include "imm/imm_ukf_pda.h"
#include "imm/ukf.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_msgs/msg/detected_object.hpp"
#include "nif_msgs/msg/detected_object_array.hpp"
#include "nif_msgs/msg/perception3_d.hpp"
#include "nif_msgs/msg/perception3_d_array.hpp"
#include "nif_utils/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <stdlib.h>
#include <string>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace nif {
namespace perception {

class IMMObjectTrackerNode : public rclcpp::Node {
public:
  IMMObjectTrackerNode(const std::string &node_name_);
  IMMObjectTrackerNode(const std::string &node_name_,
                       const std::string &tracker_config_file_path_);

private:
  // void
  // detectionCallback(const nif_msgs::msg::DetectedObjectArray::SharedPtr msg);

  void detectionCallback(const nif_msgs::msg::Perception3DArray::UniquePtr msg);
  void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  rclcpp::Publisher<nif_msgs::msg::Perception3DArray>::SharedPtr
      tracked_output_publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      tracked_output_vis_publisher;

  rclcpp::Subscription<nif_msgs::msg::Perception3DArray>::SharedPtr
      detection_result_subscription;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      ego_odom_subscription;

  // tracker related
  std::string tracker_config_file_path;
  std::shared_ptr<ImmUkfPda> tracker_ptr;

  nif_msgs::msg::Perception3DArray tracked_objects;
  nif_msgs::msg::DetectedObjectArray tracked_objects_det;

  nav_msgs::msg::Odometry ego_odom;
};

} // namespace perception
} // namespace nif
#endif //OBJECT_TRACKER_H
