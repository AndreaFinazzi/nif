//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 09/02/21.
//

#ifndef GLOBALMAP_LOADER_H
#define GLOBALMAP_LOADER_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_localization_minimal/localization_minimal.h"

// inlcude ROS library
#include "rclcpp/clock.hpp"
#include <chrono>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
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

#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl_conversions/pcl_conversions.h>


namespace nif {
namespace localization {
namespace maploader {

class GlobalmapLoader : public rclcpp::Node {
public:
  GlobalmapLoader(const std::string &node_name);
  ~GlobalmapLoader();

  void pcdFileIO();
  void TrajectorypcdFileIO();
  void Publisher();
  void respond();
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalmap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubTrajectory;

  rclcpp::TimerBase::SharedPtr timer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_globalmap_ptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_trajectory_ptr;

  std::string m_glbalmap_file_name;
  std::string m_trajectory_file_name;
  bool bUseTrajectory;
  bool bMapReady = false;
  bool bTrajectoryReady = false;
  bool bPublishOnce;
  double m_voxel_size;
};

} // namespace maploader
} // namespace localization
} // namespace nif

#endif // GLOBALMAP_LOADER_H
