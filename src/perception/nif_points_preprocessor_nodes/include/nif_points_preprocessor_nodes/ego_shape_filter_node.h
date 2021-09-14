/*
 * ego_shape_filter_node.h
 *
 *  Created on: Aug 28, 2021
 *      Author: Daegyu Lee
 */
#ifndef EGO_SHAPE_FILTER_NODE_H
#define EGO_SHAPE_FILTER_NODE_H

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
#include <pcl/filters/voxel_grid.h>

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
/**
 * 2-D grid map size for wall detection
 * MAP_WIDTH : longitudinal direction
 * MAP_HEIGHT : longitudinal direction
 */
const unsigned long MAP_WIDTH =
    400; //(front_upper_distance + rear_upper_distance) / resolution
const unsigned long MAP_HEIGHT =
    160; // (right_upper_distance + left_upper_distance) / resolution

namespace nif {
namespace perception {

class EgoShapeFilterNode : public rclcpp::Node {
public:
  EgoShapeFilterNode(const std::string &node_name_);
  ~EgoShapeFilterNode();
  void mergedPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  EgoShapeFilterNode();

  void respond();

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_filtered_points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_inverse_points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_weaker_thres_inverse_points;
   rclcpp::Publisher<
      sensor_msgs::msg::PointCloud2>::SharedPtr pub_left_ransac_filtered_points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_right_ransac_filtered_points;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      pub_both_ransac_filtered_points;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_oc_grid;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
      pub_forwarding_map_grid;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_left_wall_line;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_right_wall_line;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_inner_wall_distance;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_outer_wall_distance;

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
  double normal_angle_thres_;
  int ransac_pts_thresh_;

  double inner_bound_distance;
  double outer_bound_distance;
  double prev_inner_bound_distance;
  double prev_outer_bound_distance;
  double distance_low_fass_filter;


  double extract_distance_x_roi; 
  double extract_distance_thres;

      std::array<std::array<float, (size_t)(MAP_WIDTH + 1)>,
                 (size_t)(MAP_HEIGHT + 1)>
          map;

  void EgoShape(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                double in_left_lower_threshold, double in_right_lower_threshold,
                double in_front_lower_threshold,
                double in_rear_lower_threshold, // lower limit
                double in_left_upper_threshold, double in_right_upper_threshold,
                double in_front_upper_threshold,
                double in_rear_upper_threshold, // upper limit
                double in_height_lower_threshold,
                double in_height_upper_threshold); // height

  void RegisterPointToGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
                           double in_resolution, float min_x, float min_y);

  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

  void InverseMap(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr WeakerThrescloudOut,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLeftOut,
                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRightOut,
                  float min_x, float min_y, float in_resolution);
  boost::optional<Eigen::Vector4f>
  wall_detect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
              pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut);

  void
  ExtractDistanceInCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIn,
                         const double &x_roi_, const double &dist_thres_,
                         double &distance_out);

  nav_msgs::msg::Path
  LineVisualizer(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                 const boost::optional<Eigen::Vector4f> coeff,
                 double in_front_upper_threshold,
                 double in_rear_upper_threshold);
  cv::Mat polyfit(std::vector<cv::Point2f> &in_point, int n);


};
} // namespace perception
} // namespace nif

#endif // EGO_SHAPE_FILTER_NODE_H
