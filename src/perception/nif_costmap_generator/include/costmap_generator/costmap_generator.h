/*
 * costmap_generator.h
 *
 *  Created on: Aug 9, 2021
 *      Author: Daegyu Lee
 */

#ifndef COSTMAP_GENERATOR_H
#define COSTMAP_GENERATOR_H

// headers in ROS

// ROS
#include "rclcpp/clock.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/TypeDefs.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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

#include <Eigen/Dense>
#include <Eigen/Geometry>

// headers in STL
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>

// // headers in Autoware
// #include <costmap_generator/bboxes_to_costmap.h>
// #include <costmap_generator/lane_to_costmap.h>
// #include <costmap_generator/laserscan_to_costmap.h>
#include <costmap_generator/points_to_costmap.h>
// #include <costmap_generator/visual_to_costmap.h>

// #include <pcl_ros/point_cloud.h>

// #include <pcl/segmentation/conditional_euclidean_clustering.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/sac_segmentation.h>


// struct by_value {
//   bool operator()(geometry_msgs::Pose const &left,
//                   geometry_msgs::Pose const &right) {
//     return left.position.x < right.position.x;
//   }
// };

namespace nif {
namespace perception {
namespace costmap {

class CostmapGenerator : public rclcpp::Node {
public:
  CostmapGenerator();
  ~CostmapGenerator();

  void init();
  void run();
  void param();

private:
  void respond();

  bool use_points_;
  bool use_wayarea_;
  bool use_laserscan_;
  bool use_dynamic_trajectories_;

  bool bPoints, bBoundingBox, bRoadBoundary, bVisual;

  void timer_callback();
  void sensorPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void OdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void MessegefilteringCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &inner_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &outer_msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

  //   bool has_subscribed_wayarea_;

  //   bool loaded_lanelet_map_ = false;
  //   bool use_all_road_lanelets_ = true;

  std::string lidar_frame_;
  std::string map_frame_;
  std::string scan_topic_;
  std::string points_topic_name_;
  std::string output_map_name_;
  double visual_expand_size_;
  double grid_min_value_;
  double grid_max_value_;
  double grid_resolution_;
  double grid_length_x_;
  double grid_length_y_;
  double grid_position_x_;
  double grid_position_y_;

  double maximum_lidar_height_thres_;
  double minimum_lidar_height_thres_;
  double maximum_laserscan_distance_thres_;
  double minimum_laserscan_distance_thres_;

  double m_veh_x;
  double m_veh_y;
  double m_veh_roll, m_veh_pitch, m_veh_yaw;

  bool bGeoFence;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_inner_geofence_points;
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_outer_geofence_points;

  //   double expand_polygon_size_;
  //   int size_of_expansion_kernel_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::TimerBase::SharedPtr sub_timer_;

  using SyncPolicyT = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_InnerGeofence;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> sub_OuterGeofence;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicyT>> m_sync;

  //   ros::Subscriber sub_waypoint_;
  //   ros::Subscriber sub_points_;
  //   ros::Subscriber sub_BodyCoordPath_, sub_BoundingBoxes_;
  //   ros::Subscriber sub_Visual_;

  //   tf::TransformListener tf_listener_;

  //   std::vector<std::vector<geometry_msgs::Point>> area_points_;
  //   std::vector<std::pair<double, double>> obstacleArray;

  PointsToCostmap points2costmap_;
  //   LaneToCostmap lane2costmap_;
  //   LaserScanToCostmap laserscan2costmap_;
  //   BBoxesToCostmap bboxes2costmap_;
  //   VisualToCostmap visual2costmap_;
  std_msgs::msg::Header m_in_header;
  //   nav_msgs::Odometry m_odom;
  //   nav_msgs::Path m_LocalPathOnBody;
  //   geometry_msgs::PoseStamped pose_prev;

  grid_map::GridMap costmap_;

  //   pcl::PointCloud<pcl::PointXYZI>::Ptr m_points;

  const std::string SENSOR_POINTS_COSTMAP_LAYER_;
  //   const std::string LASER_2D_COSTMAP_LAYER_;
  const std::string COMBINED_COSTMAP_LAYER_;
  //   const std::string LANE_POINTS_COSTMAP_LAYER_;
  //   const std::string BOUNDING_BOX_COSTMAP_LAYER_;
  //   const std::string VISUAL_COSTMAP_LAYER_;
  //   const std::string INFLATION_COSTMAP_LAYER_;
  //   bool bEnablePotential_;

  //   void sensorPointsCallback(
  //       const sensor_msgs::PointCloud2::ConstPtr &in_sensor_points_msg);
  //   void
  //   LaserScanCallback(const sensor_msgs::LaserScanConstPtr
  //   &in_laser_scan_msg); void laneBoundaryCallback(
  //       const sensor_msgs::PointCloud2::ConstPtr &in_lane_points_msg);
  //   void OdometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
  //   void BoundingBoxesCallback(
  //       const jsk_recognition_msgs::BoundingBoxArray::ConstPtr &in_boxes);
  //   void VisualCallback(
  //       const detection_msgs::BoundingBoxArray::ConstPtr &msg);

  //   void LocalWaypointCallback(const nav_msgs::PathConstPtr &msg);

  //   // Added by cy : 21.01.26
  //   void RollPitchYawCallback(const geometry_msgs::Vector3::ConstPtr &msg);

  grid_map::GridMap initGridmap();
  void publishRosMsg(grid_map::GridMap *map);
  //   void publishRoadBoundaryMsg(grid_map::GridMap *map);

  grid_map::Matrix generateSensorPointsCostmap(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr &in_sensor_points);
  //   grid_map::Matrix
  //   generateLaserScanCostmap(const sensor_msgs::LaserScanConstPtr
  //   &in_laser_scan); grid_map::Matrix generateBBoxesCostmap(
  //       const jsk_recognition_msgs::BoundingBoxArrayConstPtr &in_boxes);

  //   grid_map::Matrix generateVisualCostmap(const
  //   detection_msgs::BoundingBoxArrayConstPtr &msg);

  void generateCombinedCostmap();
  //   void publishBoundaryPathMsg(nav_msgs::OccupancyGrid &gridmap);

  //   void MakeInflationWithPoints();
  //   grid_map::Matrix createGaussianWorld(grid_map::GridMap *map, const
  //   std::string layer_name,
  //                                           double inflation_x, double
  //                                           inflation_y, const
  //                                           std::vector<std::pair<double,
  //                                           double>>& pointArray);

  //   grid_map::Matrix fillGridMap(grid_map::GridMap *map, const std::string
  //   layer_name,
  //                                               const AnalyticalFunctions
  //                                               &functions);

  //   std::string m_ndtSearchMethod;
  //   pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;
};

} // namespace costmap
} // namespace perception
} // namespace nif

#endif // COSTMAP_GENERATOR_H
