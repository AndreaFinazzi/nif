//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 09/08/21.
//

#ifndef RESILIENT_LOCALIZE_H
#define RESILIENT_LOCALIZE_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"

// inlcude ROS library
#include "rclcpp/clock.hpp"
#include <chrono>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// inlcude PCL library
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>

#include <pcl/registration/correspondence_rejection_poly.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

namespace nif {
namespace localization {
namespace resilient {

class ResilientLocalization : public rclcpp::Node {
public:
  ResilientLocalization(const std::string &node_name_);
  ~ResilientLocalization();

  void EKFOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void
  InnerGeofenceDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void
  OuterGeofenceDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void
  InnerDetectedDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void
  OuterDetectedDistanceCallback(const std_msgs::msg::Float32::SharedPtr msg);

  void respond();

private:
  ResilientLocalization();
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubOuterError;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubOuterErrorFlag;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subInnerDistance;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subOuterDistance;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subInnerWallDetection;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subOuterWallDetection;

  rclcpp::TimerBase::SharedPtr timer_;

  double m_veh_x;
  double m_veh_y;
  double m_veh_roll;
  double m_veh_pitch;
  double m_veh_yaw;

  double m_geofence_inner_distance;
  double m_geofence_outer_distance;
  double m_detected_inner_distance;
  double m_detected_outer_distance;

  double m_ThresForDistanceErrorFlag;

};
} // namespace resilient
} // namespace localization
} // namespace nif

#endif // RESILIENT_LOCALIZE_H