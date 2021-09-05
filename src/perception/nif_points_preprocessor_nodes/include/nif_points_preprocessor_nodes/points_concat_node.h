//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  points_concat_filter.cpp
//   Created on: Aug 10, 2021
//       Author: Daegyu Lee
//
//
// Created by usrg on 6/24/21.
//

#ifndef POINTS_CONCAT_NODE_H
#define POINTS_CONCAT_NODE_H

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

// #include <pcl_ros/transforms.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// headers in STL
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>

#include <yaml-cpp/yaml.h>

struct LuminarPointXYZIRT {
  PCL_ADD_POINT4D;
  PCL_ADD_INTENSITY;
  float ring;
  float time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(LuminarPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(float, ring, ring)(float, time,
                                                                    time));

namespace nif {
namespace perception {

class PointsConcatFilterNode : public rclcpp::Node {

public:
  PointsConcatFilterNode(const std::string &node_name_);
  ~PointsConcatFilterNode();

protected:
private:
  PointsConcatFilterNode();

  typedef pcl::PointXYZI PointT;
  // typedef PointXYZIRT PointT;
  typedef pcl::PointCloud<PointT> PointCloudT;
  typedef sensor_msgs::msg::PointCloud2 PointCloudMsgT;
  typedef message_filters::sync_policies::ApproximateTime<
      PointCloudMsgT, PointCloudMsgT, PointCloudMsgT>
      SyncPolicyT;

  message_filters::Subscriber<PointCloudMsgT> *cloud_subscribers_[3];
  message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;

  size_t input_topics_size_;
  std::string input_topics_;
  std::string output_topic_;
  std::string output_frame_id_;

  void respond();
  void make_transform_list();
  void pointcloud_callback(const PointCloudMsgT::ConstPtr &msg1,
                           const PointCloudMsgT::ConstPtr &msg2,
                           const PointCloudMsgT::ConstPtr &msg3);

  std::vector<tf2::Transform> transfrom_list;

  void
  transformPointCloudCustom(const pcl::PointCloud<LuminarPointXYZIRT> &cloud_in,
                            pcl::PointCloud<LuminarPointXYZIRT> &cloud_out,
                            const tf2::Transform &transform);
};

} // namespace perception
} // namespace nif

#endif // POINTS_CONCAT_NODE_H
