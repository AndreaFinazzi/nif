//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegty Lee

//
// Created by usrg on 09/08/21.
//

#ifndef GEOFENCE_NODE_H
#define GEOFENCE_NODE_H

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
namespace geofence {

class GeoFenceLoader : public rclcpp::Node {
public:
  GeoFenceLoader(const std::string &node_name_);
  ~GeoFenceLoader();

  void OuterFencePCDFileIO();
  void InnerFencePCDFileIO();
  void Projector(const std::vector<std::pair<double, double>> &array_in,
                 const double &veh_x_, const double &veh_y_, double &distance,
                 nav_msgs::msg::Path &SegmentOut);
  void GeoFenceOnBody(const std::vector<std::pair<double, double>> &array_in,
                      const double &veh_x_, const double &veh_y_,
                      const double &veh_yaw_, nav_msgs::msg::Path &PathOut);

  void EKFOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

private:
  GeoFenceLoader();
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubOuterGeofence;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubInnerGeofence;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubInnerGeofencePath;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOuterGeofencePath;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubInnerSegmentPath;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubOuterSegmentPath;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubInnerDistance;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubOuterDistance;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pubOnTheTrack;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdometry;

  rclcpp::TimerBase::SharedPtr timer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_OuterFenceCloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_InnerFenceCloud;

  std::string m_OuterGeoFenceFileName;
  std::string m_InnerGeoFenceFileName;

  bool bOuterLoaded;
  bool bInnerLoaded;

  double m_voxel_size;
  double m_veh_x;
  double m_veh_y;
  double m_veh_roll, m_veh_pitch, m_veh_yaw;

  std::vector<std::pair<double, double>> m_OuterGeoFence;
  std::vector<std::pair<double, double>> m_InnerGeoFence;
};
} // namespace geofence
} // namespace localization
} // namespace nif

#endif // GEOFENCE_NODE_H3
