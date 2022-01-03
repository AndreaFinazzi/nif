//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 10/05/21.
// 

#ifndef LIDAR_CLUSTERING_H
#define LIDAR_CLUSTERING_H

// ROS
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_frame_id/frame_id.h"
#include "nif_points_preprocessor_nodes/tools.h"

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
 * 2-D grid map size for wall detection
 * MAP_WIDTH : longitudinal direction
 * MAP_HEIGHT : longitudinal direction
 */
const unsigned long MAP_WIDTH =
    400; //(front_upper_distance + rear_upper_distance) / resolution
const unsigned long MAP_HEIGHT =
    150; // (right_upper_distance + left_upper_distance) / resolution


class PointsClustering : public rclcpp::Node {
public:
  PointsClustering();
  ~PointsClustering();

private:

  void
  clusterAndColorGpu(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                     visualization_msgs::msg::MarkerArray &out_clustered_array,
                     double in_max_cluster_distance);

  void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timer_callback();
  void LeftPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void RightPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void inverseMapPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void SetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud_ptr,
                const std::vector<int> &in_cluster_indices, 
                visualization_msgs::msg::MarkerArray &out_clustered_array,
                int ind);
  void
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
             double resolution);
  void createGaussianWorld(visualization_msgs::msg::MarkerArray& marker_array_in, double inflation_x,
                           double inflation_y, pcl::PointCloud<pcl::PointXYZI>::Ptr &points_out) ;

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> &vector);

  /**
   * @brief Fills cloudOut with inverse map points
   * 
   * @param cloudIn 
   * @param cloudOut 
   * @param min_x 
   * @param min_y 
   * @param in_resolution 
   */
  void InverseMap(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut,
      const float min_x, const float min_y, const float in_resolution);

  void RegisterPointToGrid(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_resolution,
      float min_x, float min_y);

void EgoShape(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
    double in_left_lower_threshold, double in_right_lower_threshold,
    double in_front_lower_threshold,
    double in_rear_lower_threshold, // lower limit
    double in_left_upper_threshold, double in_right_upper_threshold,
    double in_front_upper_threshold,
    double in_rear_upper_threshold, // upper limit
    double in_height_lower_threshold, double in_height_upper_threshold);
  
  int m_cluster_size_min;
  int m_cluster_size_max;
  double m_max_cluster_distance;
  double m_height_filter_thres;
  bool m_use_inverse_map = false;

  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubClusterPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubSimpleheightMap;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubInflationPoints;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubClusteredArray;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubClusteredCenterPoints;
  rclcpp::Publisher<nif::common::msgs::PerceptionResultList>::SharedPtr pubPerceptionList;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubInverseMappedPoints;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subInputPoints;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLeftPoints;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subRightPoints;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subInverseMapPoints;

  rclcpp::TimerBase::SharedPtr sub_timer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_inPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_LeftPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_RightPoints;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_simpleHeightmapPoints;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_inverseMapPoints;

  bool bFrontPoints = false;
  bool bRightPoints = false;
  bool bLeftPoints = false;
  bool bInverseMapPoints = false;

  rclcpp::Time m_front_points_last_update;
  rclcpp::Time m_right_points_last_update;
  rclcpp::Time m_left_points_last_update;
  rclcpp::Time m_inverse_map_points_last_update;

  std::mutex sensor_mtx_f;
  std::mutex sensor_mtx_r;
  std::mutex sensor_mtx_l;
  std::mutex sensor_mtx_im;

  std::vector<tf2::Transform> transfrom_list = 
    nif::perception::tools::get_av21_lidar_transform_list();

  std::array<std::array<float, (size_t)(MAP_WIDTH + 1)>,
                                  (size_t)(MAP_HEIGHT + 1)>
      map;
  std::array<std::array<std::vector<double>, (size_t)(MAP_WIDTH + 1)>,
             (size_t)(MAP_HEIGHT + 1)>
      points_map;

  std::array<std::array<float, (size_t)(MAP_WIDTH + 1)>,
             (size_t)(MAP_HEIGHT + 1)>
      count_map;

  std::array<std::array<float, (size_t)(MAP_WIDTH + 1)>,
             (size_t)(MAP_HEIGHT + 1)>
      mean_map;

  std::array<std::array<float, (size_t)(MAP_WIDTH + 1)>,
             (size_t)(MAP_HEIGHT + 1)>
      cov_map;

  double left_lower_distance_;
  double right_lower_distance_;
  double rear_lower_distance_;
  double front_lower_distance_;
  double left_upper_distance_;
  double right_upper_distance_;
  double rear_upper_distance_;
  double front_upper_distance_;

  double height_upper_distance_;
  double height_lower_distance_;
  double resolution_;
  double count_threshold_;

};

#endif // LIDAR_CLUSTERING_H
