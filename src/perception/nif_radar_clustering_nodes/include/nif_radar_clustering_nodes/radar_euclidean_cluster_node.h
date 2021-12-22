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

#include "radar_msgs/msg/"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <bits/stdc++.h>
#include <math.h>
#include <mutex>
#include <thread>
#include <unordered_map>

struct AnalyticalFunctions {
  std::function<double(double, double)> f_;
};

/**
 * @brief Radar clustering node. It computes cluster on a flatten polar plane (alpha, r).
 * 
 */
class RadarCluseteringNode : public rclcpp::Node {
public:
  RadarCluseteringNode(const std::string& node_name);
  ~RadarCluseteringNode();
  void
  clusterAndColorGpu(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                     visualization_msgs::msg::MarkerArray &out_clustered_array,
                     double in_max_cluster_distance);
  void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timer_callback();
  void LeftPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void RightPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  void SetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud_ptr,
                const std::vector<int> &in_cluster_indices, 
                visualization_msgs::msg::MarkerArray &out_clustered_array,
                int ind);
  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
             double resolution);
  void createGaussianWorld(visualization_msgs::msg::MarkerArray& marker_array_in, double inflation_x,
                           double inflation_y, pcl::PointCloud<pcl::PointXYZI>::Ptr &points_out) ;

  int m_cluster_size_min;
  int m_cluster_size_max;
  double m_max_cluster_distance;
  double m_height_filter_thres;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
          pubClusterPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSimpleheightMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubInflationPoints;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      pubClusteredArray;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubClusteredCenterPoints;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subInputPoints;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLeftPoints;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subRightPoints;
  
  rclcpp::TimerBase::SharedPtr sub_timer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_inPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_LeftPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_RightPoints;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_simpleHeightmapPoints;

  bool bPoints = false;
  bool bRightPoints = false;
  bool bLeftPoints = false;


  std::mutex sensor_mtx;
};

#endif // LIDAR_CLUSTERING_H
