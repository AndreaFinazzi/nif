/*
 * nif_dk_fake_obstacle_node.h
 *
 *  Created on: Oct 11, 2021
 *      Author: Daegyu Lee
 */
#ifndef DK_FAKE_OBS_H
#define DK_FAKE_OBS_H

// headers in ROS
#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// headers in STL
#include <memory>
#include <cmath>
#include <type_traits>
#include <stdio.h>
#include <float.h>
#include <vector>
#include <queue>
#include <deque>
#include <algorithm>
#include <unordered_map>
#include <bits/stdc++.h>
#include <mutex>
#include <thread>

#include <fstream>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <utility> // std::pair

#include <pugixml.hpp>


// //User defined messages
#include "nif_dk_graph_planner_msgs/msg/osm_parcer.hpp"
#include "nif_dk_graph_planner_msgs/msg/way.hpp"
#include "nif_dk_graph_planner_msgs/msg/node.hpp"
#include "nif_dk_graph_planner_msgs/msg/final_path_id_array.hpp"

#include "utils/geodetic_conv.h"


// PCL Libraries
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>

#include <pcl/features/don.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

struct AnalyticalFunctions {
  std::function<double(double, double)> f_;
};

namespace nif{
namespace planning{

class FakeObsNode : public rclcpp::Node {
public:
  FakeObsNode(const std::string &node_name_);
  ~FakeObsNode();

  void OsmParcing();

  void MessagePublisher();
  void timer_callback();
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void TransformPointsToBody(const pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn,
                             pcl::PointCloud<pcl::PointXYZI>::Ptr CloudOut,
                             const double &veh_x_, const double &veh_y_,
                             const double &veh_yaw_);
  void TransformPointsToGlobal(const pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudIn,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr &CloudOut, 
                               const double &veh_x_, const double &veh_y_, 
                               const double &veh_yaw_);
  void createGaussianWorld(pcl::PointCloud<pcl::PointXYZI>::Ptr &points_in,
                           double inflation_x, double inflation_y,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr &points_out);

private:
  FakeObsNode();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFakeObsPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pubFakeInflatedPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr 
      pubFakeCenterPoints;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalFakeInflatedPoints;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubGlobalFakeCenterPoints;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_FakeObsPoints;
  rclcpp::TimerBase::SharedPtr sub_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;

  bool bParcingComplete = false;
  std::string m_OsmFileName, m_FullFilePath;
  nif_dk_graph_planner_msgs::msg::OsmParcer m_OsmParcer;
  nif::localization::utils::GeodeticConverter conv_;
  double m_originLat;
  double m_originLon;

  double m_veh_x;
  double m_veh_y;
  double m_veh_roll, m_veh_pitch, m_veh_yaw;
  bool bOdometry = false;
  };
} // namespace planning
} // namespace nif

#endif  // DK_FAKE_OBS_H
