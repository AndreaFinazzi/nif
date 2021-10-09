//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 10/05/21.
// 

#ifndef LIDAR_CLUSTERING_H
#define LIDAR_CLUSTERING_H

// ROS
#include "rclcpp/clock.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_frame_id/frame_id.h"


// PCL library
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <mutex>
#include <thread>
#include <algorithm>

class PointsClustering : public rclcpp::Node {
public:
  PointsClustering();
  ~PointsClustering();
  void clusterAndColorGpu(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                          double in_max_cluster_distance);
  void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timer_callback();

private:
  void SetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud_ptr,
                const std::vector<int> &in_cluster_indices, int ind);
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
             double resolution);
  int m_cluster_size_min;
  int m_cluster_size_max;
  double m_max_cluster_distance;
  double m_height_filter_thres;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
          pubClusterPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSimpleheightMap;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
          subInputPoints;
  rclcpp::TimerBase::SharedPtr sub_timer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_inPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_simpleHeightmapPoints;
  bool bPoints = false;

  std::mutex sensor_mtx;
};

#endif // LIDAR_CLUSTERING_H
