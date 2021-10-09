//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  points_concat_async_node.h
//   Created on: Oct 7, 2021
//       Author: Daegyu Lee
//
//
// Created by usrg on 6/24/21.
//

#ifndef POINTS_CONCAT_ASYNC_NODE_H
#define POINTS_CONCAT_ASYNC_NODE_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_sensor_fusion/sensor_fusion_manager.h"
#include "nif_tracking/tracking_manager.h"

#include "message_filters/connection.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/rclcpp.hpp"

#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_listener.h>

// PCL library
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

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

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter.h>

// #include <pcl_ros/transforms.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// headers in STL
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>

#include <yaml-cpp/yaml.h>

namespace nif {
namespace perception {

class PointsConcatAsyncFilterNode : public rclcpp::Node {

public:
  PointsConcatAsyncFilterNode(const std::string &node_name_);
  ~PointsConcatAsyncFilterNode();

  void FrontCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void LeftCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void RightCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void timerCallback();

protected:
private:
  PointsConcatAsyncFilterNode();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_front_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_left_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_right_;
  rclcpp::TimerBase::SharedPtr
      timer_; // !< @brief time for ekf calculation callback

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
          pub_concat_points_;


  rclcpp::Time front_last_update;
  rclcpp::Time left_last_update;
  rclcpp::Time right_last_update;
  rclcpp::Duration timeout = rclcpp::Duration(1, 0);

  size_t input_topics_size_;
  std::string input_topics_;
  std::string output_topic_;
  std::string output_frame_id_;
  bool use_new_luminar_driver_;

  bool bFront = false;
  bool bLeft = false;
  bool bRight = false;
  bool bTransfromListGenerated = false;

      pcl::PointCloud<pcl::PointXYZI>::Ptr m_front_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_left_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_right_points;

  void make_transform_list();

  std::vector<tf2::Transform> transfrom_list;

  void transformPointCloudCustom(const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
                            pcl::PointCloud<pcl::PointXYZI> &cloud_out,
                            const tf2::Transform &transform);

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);
};

} // namespace perception
} // namespace nif

#endif // POINTS_CONCAT_ASYNC_NODE_H
