/*
 * wall_detection_node.h
 *
 *  Created on: Oct 6, 2021
 *      Author: Daegyu Lee
 */
#ifndef WALL_DETECTION_NODE_H
#define WALL_DETECTION_NODE_H

#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_sensor_fusion/sensor_fusion_manager.h"
#include "nif_tracking/tracking_manager.h"

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
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>

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

#include <mutex>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <thread>

// #include <pcl_ros/transforms.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// headers in STL
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>

// OpenCV Libraries
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

// Kin-Controller
#include "nif_wall_following_controller/kin_control_node.hpp" 


namespace nif {
namespace perception {

class WallDetectionNode : public rclcpp::Node {
public:
  WallDetectionNode(const std::string &node_name_);
  ~WallDetectionNode();
  void InverseLeftCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void InverseRightCallback(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void WheelSpeedCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
  void RadarMarkerCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
  void timer_callback();

private:
  WallDetectionNode();

  void respond();
  void SetControllerParams();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_inverse_left_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      sub_inverse_right_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr sub_wheel_speed_;
  rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_radar_marker_;
  rclcpp::TimerBase::SharedPtr timer_;

   rclcpp::Publisher<
      sensor_msgs::msg::PointCloud2>::SharedPtr pub_left_ransac_filtered_points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_right_ransac_filtered_points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_both_ransac_filtered_points;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_left_wall_line;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_right_wall_line;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_wall_following_path;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_inner_wall_distance;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_outer_wall_distance;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_wall_following_steer_cmd;

  double rear_upper_distance_;
  double front_upper_distance_;

  double normal_angle_thres_;
  int ransac_pts_thresh_;
  double m_ransacDistanceThres;

  double inner_bound_distance;
  double outer_bound_distance;
  double prev_inner_bound_distance;
  double prev_outer_bound_distance;
  double distance_low_fass_filter;

  int m_cluster_size_min;
  int m_cluster_size_max;
  double m_max_cluster_distance;

  double extract_distance_x_roi; 
  double extract_distance_thres;
  double m_target_space_to_wall;
  double m_margin_to_wall;
  int m_average_filter_size;
  std::deque<double> control_out_que;

  std::mutex mtx_right;
  std::mutex mtx_left;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_InverseLeftPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_InverseRightPoints;
  bool bInverseLeftPoints = false;
  bool bInverseRightPoints = false;

  nav_msgs::msg::Path final_wall_following_path_msg;
  nif::control::KinControl m_KinController;
  double m_vel_speed_x;
  int average_filter_size;

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

  void
  ExtractDistanceInCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIn,
                         const double &x_roi_, const double &dist_thres_,
                         double &distance_out);

  void CubicSpliner(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                    const bool wall_detected, double in_front_upper_threshold,
                    double in_rear_upper_threshold,
                    nav_msgs::msg::Path &path_msg_out,
                    cv::Mat &PolyCoefficient);

  cv::Mat polyfit(std::vector<cv::Point2f> &in_point, int n);

  void EstimatePredictivePath(
      const bool wall_detected,
      const cv::Mat &PolyCoefficient, nav_msgs::msg::Path &path_msg_out,
      const double &target_space_to_wall);

  void
  wall_detect_with_clustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                     pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                     double in_max_cluster_distance);
  void SetCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_origin_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZI>::Ptr register_cloud_ptr,
                const std::vector<int> &in_cluster_indices, int ind,
                double &cluster_length);
};
} // namespace perception
} // namespace nif

#endif // WALL_DETECTION_NODE_H
