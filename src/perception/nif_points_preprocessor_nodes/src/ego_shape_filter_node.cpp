
/*
 * ego_shape_filter_node.cpp
 *
 *  Created on: Aug 10, 2021
 *      Author: Daegyu Lee
 */

#include <nif_points_preprocessor_nodes/ego_shape_filter_node.h>
#include "nif_frame_id/frame_id.h"

using namespace nif::perception;
using namespace nif::common::frame_id::localization;

EgoShapeFilterNode::EgoShapeFilterNode(const std::string &node_name_)
    : Node(node_name_) {
  this->declare_parameter<double>("left_lower_distance", double(0.5));
  this->declare_parameter<double>("right_lower_distance", double(0.5));
  this->declare_parameter<double>("rear_lower_distance", double(2.2));
  this->declare_parameter<double>("front_lower_distance", double(1.5));

  this->declare_parameter<double>("left_upper_distance", double(20.0));
  this->declare_parameter<double>("right_upper_distance", double(20.0));
  this->declare_parameter<double>("rear_upper_distance", double(50.0));
  this->declare_parameter<double>("front_upper_distance", double(50.0));

  this->declare_parameter<double>("height_lower_distance", double(-0.5));
  this->declare_parameter<double>("height_upper_distance", double(1.0));
  this->declare_parameter<double>("resolution", double(0.25));
  this->declare_parameter<double>("count_threshold", double(3.));
  this->declare_parameter<double>("normal_angle_thres", double(50.));
  this->declare_parameter<int>("ransac_pts_thresh", int(200));
  this->declare_parameter<double>("ransac_distance_thres", double(0.2));

  this->declare_parameter<double>("x_roi", double(0.));
  this->declare_parameter<double>("distance_extract_thres", double(0.5));
  this->declare_parameter<double>("distance_low_fass_filter", double(0.5));
  this->declare_parameter<double>("target_space_to_wall", double(7.));

  // Controller Parameters
  this->declare_parameter<double>("min_lookahead", double(4.0));
  this->declare_parameter<double>("max_lookahead", double(30.0));
  this->declare_parameter<double>("lookahead_speed_ratio", double(0.75));
  this->declare_parameter<double>("proportional_gain", double(0.2));
  this->declare_parameter<double>("vehicle.wheelbase", double(2.97));
  this->declare_parameter<double>("max_steer_angle", double(30.0)); // 15 deg * 2 because ratio is wrong

  respond();

  RCLCPP_INFO(this->get_logger(), "left_lower_distance_: %f", left_lower_distance_);
  RCLCPP_INFO(this->get_logger(), "right_lower_distance_: %f", right_lower_distance_);
  RCLCPP_INFO(this->get_logger(), "rear_lower_distance_: %f", rear_lower_distance_);
  RCLCPP_INFO(this->get_logger(), "front_lower_distance_: %f", front_lower_distance_);

  RCLCPP_INFO(this->get_logger(), "left_upper_distance_: %f", left_upper_distance_);
  RCLCPP_INFO(this->get_logger(), "right_upper_distance_: %f", right_upper_distance_);
  RCLCPP_INFO(this->get_logger(), "rear_upper_distance_: %f", rear_upper_distance_);
  RCLCPP_INFO(this->get_logger(), "front_upper_distance_: %f", front_upper_distance_);

  RCLCPP_INFO(this->get_logger(), "height_lower_distance: %f", height_lower_distance_);
  RCLCPP_INFO(this->get_logger(), "height_upper_distance: %f", height_upper_distance_);
  RCLCPP_INFO(this->get_logger(), "resolution: %f", resolution_);
  RCLCPP_INFO(this->get_logger(), "normal_angle_thres: %f", normal_angle_thres_);
  RCLCPP_INFO(this->get_logger(), "ransac_pts_thresh: %f", ransac_pts_thresh_);

  RCLCPP_INFO(this->get_logger(), "x_roi : %f", extract_distance_x_roi);
  RCLCPP_INFO(this->get_logger(), "distance_extract_thres : %f", extract_distance_thres );
  RCLCPP_INFO(this->get_logger(), "distance_low_fass_filter : %f", distance_low_fass_filter );
  RCLCPP_INFO(this->get_logger(), "target_space_to_wall : %f", m_target_space_to_wall );

  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/merged/lidar", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&EgoShapeFilterNode::mergedPointsCallback, this,
                std::placeholders::_1));
  sub_wheel_speed_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "raptor_dbw_interface/wheel_speed_report",
      nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&EgoShapeFilterNode::WheelSpeedCallback, this,
                std::placeholders::_1));

  using namespace std::chrono_literals; // NOLINT
  // TODO convert period to paramter
  timer_ = this->create_wall_timer(
      10ms, std::bind(&EgoShapeFilterNode::timer_callback, this));

  pub_filtered_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/merged/ego_filtered", nif::common::constants::QOS_SENSOR_DATA);
  pub_oc_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/OccupancyGridMap/map", nif::common::constants::QOS_SENSOR_DATA);
  pub_forwarding_map_grid =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/OccupancyGridMap/forwarding_map", nif::common::constants::QOS_SENSOR_DATA);

  pub_inverse_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/inverse_mapped_points", nif::common::constants::QOS_SENSOR_DATA);
  pub_weaker_thres_inverse_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/weaker_thres_inverse_mapped_points", nif::common::constants::QOS_SENSOR_DATA);
  pub_left_ransac_filtered_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/ransac_filtered_points/left", nif::common::constants::QOS_SENSOR_DATA);
  pub_right_ransac_filtered_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/ransac_filtered_points/right", nif::common::constants::QOS_SENSOR_DATA);
  pub_both_ransac_filtered_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/ransac_filtered_points/both", nif::common::constants::QOS_SENSOR_DATA);

  pub_left_wall_line =
      this->create_publisher<nav_msgs::msg::Path>("/wall_left", nif::common::constants::QOS_SENSOR_DATA);
  pub_right_wall_line =
      this->create_publisher<nav_msgs::msg::Path>("/wall_right", nif::common::constants::QOS_SENSOR_DATA);
  pub_wall_following_path = this->create_publisher<nav_msgs::msg::Path>(
      "/wall_based_predictive_path", nif::common::constants::QOS_SENSOR_DATA);

  pub_inner_wall_distance = this->create_publisher<std_msgs::msg::Float32>(
      "/detected_inner_distance", nif::common::constants::QOS_SENSOR_DATA);
  pub_outer_wall_distance = this->create_publisher<std_msgs::msg::Float32>(
      "/detected_outer_distance", nif::common::constants::QOS_SENSOR_DATA);

  pub_wall_following_steer_cmd = this->create_publisher<std_msgs::msg::Float32>(
      "/wall_following_steering_cmd",
      nif::common::constants::QOS_CONTROL_CMD); // TODO

  lidar_timeout = rclcpp::Duration(1, 0);

  // kin controller
  SetControllerParams();
  m_KinController.setCmdsToZeros();
}

EgoShapeFilterNode::~EgoShapeFilterNode() {}

void EgoShapeFilterNode::respond() {
  this->get_parameter("left_lower_distance", left_lower_distance_);
  this->get_parameter("right_lower_distance", right_lower_distance_);
  this->get_parameter("rear_lower_distance", rear_lower_distance_);
  this->get_parameter("front_lower_distance", front_lower_distance_);

  this->get_parameter("left_upper_distance", left_upper_distance_);
  this->get_parameter("right_upper_distance", right_upper_distance_);
  this->get_parameter("rear_upper_distance", rear_upper_distance_);
  this->get_parameter("front_upper_distance", front_upper_distance_);

  this->get_parameter("height_lower_distance", height_lower_distance_);
  this->get_parameter("height_upper_distance", height_upper_distance_);

  this->get_parameter("resolution", resolution_);
  this->get_parameter("count_threshold", count_threshold_);
  this->get_parameter("normal_angle_thres", normal_angle_thres_);
  this->get_parameter("ransac_pts_thresh", ransac_pts_thresh_);
  this->get_parameter("ransac_distance_thres",m_ransacDistanceThres);

  this->get_parameter("x_roi", extract_distance_x_roi);
  this->get_parameter("distance_extract_thres", extract_distance_thres);
  this->get_parameter("distance_low_fass_filter", distance_low_fass_filter);
  this->get_parameter("target_space_to_wall", m_target_space_to_wall);

}

void EgoShapeFilterNode::SetControllerParams() {
  m_KinController.min_la = this->get_parameter("min_lookahead").as_double();
  m_KinController.max_la = this->get_parameter("max_lookahead").as_double();
  m_KinController.la_ratio = this->get_parameter("lookahead_speed_ratio").as_double();
  m_KinController.Kp = this->get_parameter("proportional_gain").as_double();
  m_KinController.L = this->get_parameter("vehicle.wheelbase").as_double();
  m_KinController.ms = this->get_parameter("max_steer_angle").as_double();
}

void EgoShapeFilterNode::EgoShape(
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr,
    pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
    double in_left_lower_threshold, double in_right_lower_threshold,
    double in_front_lower_threshold,
    double in_rear_lower_threshold, // lower limit
    double in_left_upper_threshold, double in_right_upper_threshold,
    double in_front_upper_threshold,
    double in_rear_upper_threshold, // upper limit
    double in_height_lower_threshold, double in_height_upper_threshold) {
  pcl::PointIndices::Ptr far_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
    pcl::PointXYZI current_point;
    current_point.x = in_cloud_ptr->points[i].x;
    current_point.y = in_cloud_ptr->points[i].y;
    current_point.z = in_cloud_ptr->points[i].z;

    if (current_point.y > (in_left_upper_threshold)) {
      far_indices->indices.push_back(i);
      continue;
    }
    if (current_point.y < (-1.0 * in_right_upper_threshold)) {
      far_indices->indices.push_back(i);
      continue;
    }

    if (current_point.x > in_front_upper_threshold ||
        current_point.x < -in_rear_upper_threshold) {
      far_indices->indices.push_back(i);
      continue;
    }

    if (current_point.z < (in_height_lower_threshold)) {
      far_indices->indices.push_back(i);
      continue;
    }
    if (current_point.z > (in_height_upper_threshold)) {
      far_indices->indices.push_back(i);
      continue;
    }

    if (current_point.y < (in_left_lower_threshold) &&
        current_point.y > -1.0 * in_right_lower_threshold) {
      if (current_point.x < (in_front_lower_threshold) &&
          current_point.x > -1.0 * in_rear_lower_threshold) {
        far_indices->indices.push_back(i);
      }
    }
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(far_indices);
  extract.setNegative(
      true); // true removes the indices, false leaves only the indices
  extract.filter(*out_cloud_ptr);
}

void EgoShapeFilterNode::timer_callback() {
  std::lock_guard<std::mutex> sensor_lock(sensor_mtx);

  if (!bMergedLidar)
    return;

  if ((this->now() - lidar_time_last_update) >= this->lidar_timeout) {
    // Set error, but keep going
    // node_status = common::NODE_ERROR;
    RCLCPP_WARN_ONCE(this->get_logger(), "No lidar update");
    RCLCPP_DEBUG(this->get_logger(), "No lidar update");
    return;
  } else {
    // node_status = common::NODE_OK;
  }

  float min_x = -(front_upper_distance_ + rear_upper_distance_) / 2;
  float min_y = -(left_upper_distance_ + right_upper_distance_) / 2;

  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseBoth(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseLeft(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseRight(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseWeakerThres(
      new pcl::PointCloud<pcl::PointXYZI>);

  /* REGISTER POINTS TO GRID 
  1. Register points on the 2-d grid map for ground-filtering
  2. Visualize the occupancy grid map
    - input : ego-shape & voxelized points, grid resolution, origin point of grid
    - output : 2-D grid map
  */
  RegisterPointToGrid(m_CloudShapeFiltered, resolution_, min_x, min_y);

  /* INVERSE MAP
  1. Find the ground filtered points
  2. Find the left/right filtered points
    - input : ego-shape & voxelized points, grid resolution, origin point of grid
    - output : Inverse mapped filtered points (Both, Left, Right)
  */
  InverseMap(m_CloudShapeFiltered, CloudInverseBoth, CloudInverseWeakerThres,
             CloudInverseLeft, CloudInverseRight, min_x, min_y, resolution_);

  sensor_msgs::msg::PointCloud2 cloud_inverse_msg;
  pcl::toROSMsg(*CloudInverseBoth, cloud_inverse_msg);
  cloud_inverse_msg.header.frame_id = BASE_LINK;
  cloud_inverse_msg.header.stamp = this->now();
  pub_inverse_points->publish(cloud_inverse_msg);

  sensor_msgs::msg::PointCloud2 cloud_weaker_thres_inverse_msg;
  pcl::toROSMsg(*CloudInverseWeakerThres, cloud_weaker_thres_inverse_msg);
  cloud_weaker_thres_inverse_msg.header.frame_id = BASE_LINK;
  cloud_weaker_thres_inverse_msg.header.stamp = this->now();
  pub_weaker_thres_inverse_points->publish(cloud_weaker_thres_inverse_msg);

  /* LEFT RANSAC FILTER
  1. Remove outlier points
  2. Implement left/right points separately.
    - input : left ground filtered points
    - output : distance to the wall, ransac filtered left wall points
  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACLeft(
      new pcl::PointCloud<pcl::PointXYZI>);
  // left wall detection
  boost::optional<Eigen::Vector4f> left_wall_plane_coeff =
      wall_detect(CloudInverseLeft, CloudRANSACLeft);

  inner_bound_distance = 0;
  outer_bound_distance = 0;
  // detected left_wall_plane_coeff coefficients
  if (left_wall_plane_coeff && !CloudRANSACLeft->points.empty()) {
    // for (int i = 0; i < 4; i++) {
    //   std::cout << "left wall" << (*left_wall_plane_coeff)[i] << std::endl;
    // }
    // inner_bound_distance = (*left_wall_plane_coeff)[3];
    RCLCPP_DEBUG(this->get_logger(), "Left margin : %f", inner_bound_distance);

    ExtractDistanceInCloud(CloudRANSACLeft, extract_distance_x_roi,
                           extract_distance_thres, inner_bound_distance);
  }
  sensor_msgs::msg::PointCloud2 cloud_left_ransac_filtered_msg;
  pcl::toROSMsg(*CloudRANSACLeft, cloud_left_ransac_filtered_msg);
  cloud_left_ransac_filtered_msg.header.frame_id = BASE_LINK;
  cloud_left_ransac_filtered_msg.header.stamp = this->now();
  pub_left_ransac_filtered_points->publish(cloud_left_ransac_filtered_msg);
  std_msgs::msg::Float32 inner_bound_distance_msg;
  inner_bound_distance_msg.data =
      -1 * ((distance_low_fass_filter)*inner_bound_distance +
            (1 - distance_low_fass_filter) * prev_inner_bound_distance);
  pub_inner_wall_distance->publish(inner_bound_distance_msg);

  prev_inner_bound_distance = inner_bound_distance;

  /* RIGHT WALL BASED
  1. Remove outlier points
  2. Implement left/right points separately.
    - input : right ground filtered points
    - output : distance to the wall, ransac filtered right wall points
  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACRight(
      new pcl::PointCloud<pcl::PointXYZI>);
  // right wall detection
  boost::optional<Eigen::Vector4f> right_wall_plane_coeff =
      wall_detect(CloudInverseRight, CloudRANSACRight);

  // detected right_wall_plane_coeff coefficients
  if (right_wall_plane_coeff && !CloudRANSACRight->points.empty()) {
    // for (int i = 0; i < 4; i++) {
    //   std::cout << "right wall" << (*right_wall_plane_coeff)[i] << std::endl;
    // }
    // outer_bound_distance = (*right_wall_plane_coeff)[3];
    RCLCPP_DEBUG(this->get_logger(), "Right margin : %f",
                 -1 * outer_bound_distance);
    ExtractDistanceInCloud(CloudRANSACRight, extract_distance_x_roi,
                           extract_distance_thres, outer_bound_distance);
  }
  sensor_msgs::msg::PointCloud2 cloud_right_ransac_filtered_msg;
  pcl::toROSMsg(*CloudRANSACRight, cloud_right_ransac_filtered_msg);
  cloud_right_ransac_filtered_msg.header.frame_id = BASE_LINK;
  cloud_right_ransac_filtered_msg.header.stamp = this->now();
  pub_right_ransac_filtered_points->publish(cloud_right_ransac_filtered_msg);

  std_msgs::msg::Float32 outer_bound_distance_msg;
  // outer_bound_distance_msg.data =
  //     -1 * ((distance_low_fass_filter)*outer_bound_distance +
  //           (1 - distance_low_fass_filter) * prev_outer_bound_distance);
  outer_bound_distance_msg.data = -1 * outer_bound_distance;

  pub_outer_wall_distance->publish(outer_bound_distance_msg);

  prev_outer_bound_distance = outer_bound_distance;

  /* BOTH OF RANSAC-FILTERED POINTS PUBLISHER
  1. Publish ransac filtered both of wall points
    - input : left/right ransac filtered points
    - output : both of ransac filtered points
  */
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACBoth(
      new pcl::PointCloud<pcl::PointXYZI>);
  *CloudRANSACBoth += *CloudRANSACRight;
  *CloudRANSACBoth += *CloudRANSACLeft;

  sensor_msgs::msg::PointCloud2 cloud_both_ransac_filtered_msg;
  pcl::toROSMsg(*CloudRANSACBoth, cloud_both_ransac_filtered_msg);
  cloud_both_ransac_filtered_msg.header.frame_id = BASE_LINK;
  cloud_both_ransac_filtered_msg.header.stamp = this->now();
  pub_both_ransac_filtered_points->publish(cloud_both_ransac_filtered_msg);

  /* CUBIC SPLINER & WALL PATH PUBLISHER
  1. Publish left/right wall detected path on the BASE_LINK frame
    - input : left/right ransac-filtered points
    - output : ransac-filtered wall path
  */
  nav_msgs::msg::Path left_path_msg;
  left_path_msg.header.frame_id = BASE_LINK;
  left_path_msg.header.stamp = this->now();
  cv::Mat LeftPolyCoefficient;
  CubicSpliner(CloudRANSACLeft, left_wall_plane_coeff, front_upper_distance_,
               rear_upper_distance_, left_path_msg, LeftPolyCoefficient);
  pub_left_wall_line->publish(left_path_msg);

  nav_msgs::msg::Path right_path_msg;
  right_path_msg.header.frame_id = BASE_LINK;
  right_path_msg.header.stamp = this->now();
  cv::Mat RightPolyCoefficient;
  CubicSpliner(CloudRANSACRight, right_wall_plane_coeff, front_upper_distance_,
               rear_upper_distance_, right_path_msg, RightPolyCoefficient);
  pub_right_wall_line->publish(right_path_msg);

  /* WALL CURVATURE BASED CONTROLLER 
  1. Left/Right wall based lateral control output
  2. Estimated curvature / predictive path based on the kinematic model
    - input : coefficient of cubic splined wall
    - input : feasibility of wall detection result(named as left_wall_plane_coeff/right_wall_plane_coeff)
    - output : wall following reference path on the base_link frame
  */
  nav_msgs::msg::Path wall_folllowing_path_msg;
  wall_folllowing_path_msg.header.frame_id = BASE_LINK;
  wall_folllowing_path_msg.header.stamp = this->now();

  if (right_wall_plane_coeff && outer_bound_distance != 0.0) {
    if (fabs(outer_bound_distance) < m_target_space_to_wall)
    {
      m_margin_to_wall = outer_bound_distance + m_target_space_to_wall;
    }
    else {
      m_margin_to_wall = 0.0;
    }
  }
  final_wall_following_path_msg.header.frame_id = BASE_LINK;
  final_wall_following_path_msg.header.stamp = this->now();

  EstimatePredictivePath(right_wall_plane_coeff, RightPolyCoefficient,
                         wall_folllowing_path_msg, m_margin_to_wall);

  if (right_wall_plane_coeff && !wall_folllowing_path_msg.poses.empty()) {
    final_wall_following_path_msg = wall_folllowing_path_msg;
  }
  pub_wall_following_path->publish(final_wall_following_path_msg);

  /* KIN-CONTROLLER 
  1. Calculate Control output using KinController
  2. Calculate predictive path from the vehicle
    - params : getparam using SetControllerParams()
    - input : wall_folllowing_path_msg
    - output : control output
    - output : predictive path on the base_link frame
  */
  m_KinController.setPath(final_wall_following_path_msg);
  m_KinController.setVelocity(m_vel_speed_x);
  m_KinController.run();
  double wall_following_steering_cmd;
  m_KinController.getSteering(wall_following_steering_cmd);
  std_msgs::msg::Float32 wall_following_steering_cmd_msg;
  wall_following_steering_cmd_msg.data = wall_following_steering_cmd;
  pub_wall_following_steer_cmd->publish(wall_following_steering_cmd_msg);
}

void EgoShapeFilterNode::mergedPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::lock_guard<std::mutex> sensor_lock(sensor_mtx);

  lidar_time_last_update = this->now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn(
          new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudVoxelized(
      new pcl::PointCloud<pcl::PointXYZI>);
  m_CloudShapeFiltered.reset(new pcl::PointCloud<pcl::PointXYZI>());

  RCLCPP_DEBUG(this->get_logger(), "-------------");

  pcl::fromROSMsg(*msg, *CloudIn);
  CloudVoxelized = downsample(CloudIn, resolution_);

  EgoShape(CloudVoxelized, m_CloudShapeFiltered, left_lower_distance_,
           right_lower_distance_, front_lower_distance_, rear_lower_distance_,
           left_upper_distance_, right_upper_distance_, front_upper_distance_,
           rear_upper_distance_, height_lower_distance_,
           height_upper_distance_);
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*m_CloudShapeFiltered, cloud_msg);
  cloud_msg.header = msg->header;
  cloud_msg.header.frame_id = BASE_LINK;
  pub_filtered_points->publish(cloud_msg);

  bMergedLidar = true;
}

void EgoShapeFilterNode::WheelSpeedCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
{
  m_vel_speed_x = (msg->front_right + msg->front_left) / 2 * nif::common::constants::KPH2MS;
}


void EgoShapeFilterNode::RegisterPointToGrid(
    pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, double in_resolution,
    float min_x, float min_y) {
  nav_msgs::msg::OccupancyGrid oc_grid_msg;
  oc_grid_msg.header.frame_id = BASE_LINK;
  oc_grid_msg.header.stamp = this->now();
  oc_grid_msg.info.origin.position.x = min_x;
  oc_grid_msg.info.origin.position.y = min_y;
  oc_grid_msg.info.origin.position.z = 0.;
  oc_grid_msg.info.origin.orientation.x = 0.;
  oc_grid_msg.info.origin.orientation.y = 0.;
  oc_grid_msg.info.origin.orientation.z = 0.;
  oc_grid_msg.info.origin.orientation.w = 1.;
  oc_grid_msg.info.width = MAP_WIDTH;
  oc_grid_msg.info.height = MAP_HEIGHT;
  oc_grid_msg.info.resolution = in_resolution;

  nav_msgs::msg::OccupancyGrid forwarding_map_grid_msg = oc_grid_msg;

  for (int i = 0; i < MAP_HEIGHT + 1; i++) {
    this->map[i].fill(0.0);
  }

  int x, y;
  for (auto point_buf : in_cloud_ptr->points) {
    x = std::floor((point_buf.x - min_x) / in_resolution);
    y = std::floor((point_buf.y - min_y) / in_resolution);

    if (x > MAP_WIDTH || x < 0 || y > MAP_HEIGHT || y < 0) {
      // std::cout << x << ", " << y << std::endl;
      continue;
    }
    map[y][x] = map[y][x] + 1.f; // count hit
  }

  for (int i = 0; i < MAP_HEIGHT; i++) {
    for (int j = 0; j < MAP_WIDTH; j++) {
      if (map[i][j] > count_threshold_) {
        // oc_grid_msg.data.push_back((int8_t)(map[i][j]));
        oc_grid_msg.data.push_back((int8_t)(80.));
      } else {
        oc_grid_msg.data.push_back((int8_t)(0.0));
      }

      // if (i > 1 && i < MAP_HEIGHT - 1 && j < MAP_WIDTH - 3) {
      //   if (map[i][j] > 0 && map[i][j + 3] > 0) {
      //     forwarding_map_grid_msg.data.push_back((int8_t)(80.));
      //   } else {
      //     forwarding_map_grid_msg.data.push_back((int8_t)(0.));
      //   }
      // } else {
      //   forwarding_map_grid_msg.data.push_back((int8_t)(0.));
      // }
    }
  }
  pub_oc_grid->publish(oc_grid_msg);
  // pub_forwarding_map_grid->publish(forwarding_map_grid_msg);
}

void EgoShapeFilterNode::InverseMap(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut,
    pcl::PointCloud<pcl::PointXYZI>::Ptr WeakerThrescloudOut,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudLeftOut,
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRightOut, float min_x,
    float min_y, float in_resolution) {
  int x, y;
  for (auto point_buf : cloudIn->points) {
    x = std::floor((point_buf.x - min_x) / in_resolution);
    y = std::floor((point_buf.y - min_y) / in_resolution);
    if (map[y][x] > count_threshold_) {
      cloudOut->points.push_back(point_buf);
      if (point_buf.y > 0) {
        cloudLeftOut->points.push_back(point_buf);
      } else {
        cloudRightOut->points.push_back(point_buf);
      }
    }
    if (map[y][x] > count_threshold_ - 1.) {
      WeakerThrescloudOut->points.push_back(point_buf);
    }
  }
}
pcl::PointCloud<pcl::PointXYZI>::Ptr
EgoShapeFilterNode::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, 0.05);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}

/**
 * @brief wall_detect the floor plane from a point cloud
 * @param cloud  input cloud
 * @return detected floor plane coefficients
 */
boost::optional<Eigen::Vector4f>
EgoShapeFilterNode::wall_detect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut) {

  // too few points for RANSAC
  if (cloud->size() < 50) {
    return boost::none;
  }

  // RANSAC
  pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
  ransac.setDistanceThreshold(0.2);
  ransac.computeModel();

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  ransac.getInliers(inliers->indices);

  // too few inliers
  if (inliers->indices.size() < ransac_pts_thresh_) {
    return boost::none;
  }

  // verticality check of the detected wall's normal
  Eigen::Vector4f reference = Eigen::Vector4f::UnitY();

  Eigen::VectorXf coeffs;
  ransac.getModelCoefficients(coeffs);

  double dot = coeffs.head<3>().dot(reference.head<3>());

  if (std::fabs(dot) < std::cos(normal_angle_thres_ * M_PI / 180.0)) {
    // the normal is not vertical
    return boost::none;
  }

  // make the normal sideward
  if (coeffs.head<3>().dot(Eigen::Vector3f::UnitY()) < 0.0f) {
    coeffs *= -1.0f;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr inlier_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.filter(*inlier_cloud);

  *cloudOut = *inlier_cloud;

  return Eigen::Vector4f(coeffs);
}

void EgoShapeFilterNode::ExtractDistanceInCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudIn, const double& x_roi_, const double& dist_thres_, double& distance_out)
    {
      double sum_y = 0;
      double count = 0;
      for(auto point : cloudIn->points)
      {
        if (fabs(point.x - x_roi_) < dist_thres_) {
          sum_y += point.y;
          count ++;
        }
      }
      if(count != 0)
      {
        distance_out = sum_y / count;
      }
      else
      {
        distance_out = 0;
      }

    }

void EgoShapeFilterNode::CubicSpliner(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
        const boost::optional<Eigen::Vector4f> wall_plane_coeff,
        double in_front_upper_threshold, double in_rear_upper_threshold,
        nav_msgs::msg::Path& path_msg_out, cv::Mat& PolyCoefficientOut) {
        
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudDownSampled(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (auto point_buf : cloudIn->points) {
    pcl::PointXYZI ground_points;
    ground_points.x = point_buf.x;
    ground_points.y = point_buf.y;
    cloudDownSampled->points.push_back(ground_points);
  }
  cloudDownSampled = downsample(cloudDownSampled, 0.25);

  std::vector<cv::Point2f> ToBeFit;
  ToBeFit.clear();

  for (auto point_buf : cloudDownSampled->points) {
    cv::Point2f pointTmp;
    pointTmp.x = point_buf.x;
    pointTmp.y = point_buf.y;
    ToBeFit.push_back(pointTmp);
  }

  // For polynomial fitting
  int poly_order = 2;
  PolyCoefficientOut = polyfit(ToBeFit, poly_order);

  if (!wall_plane_coeff) {
    return;
  }

  for (double x = -30; x < 50; x = x + 0.5) {
    geometry_msgs::msg::PoseStamped pose_buf;
    pose_buf.pose.position.x = x;
    pose_buf.pose.position.y =
        PolyCoefficientOut.at<double>(poly_order, 0) * pow(x, poly_order) +
        PolyCoefficientOut.at<double>(poly_order - 1, 0) * pow(x, poly_order - 1) +
        // PolyCoefficientOut.at<double>(poly_order - 2, 0) * pow(x, poly_order - 2) +
        PolyCoefficientOut.at<double>(poly_order - 2, 0); // Cubic
    pose_buf.pose.orientation.w = 1;
    pose_buf.pose.orientation.x = 0;
    pose_buf.pose.orientation.y = 0;
    pose_buf.pose.orientation.z = 0;
    path_msg_out.poses.push_back(pose_buf);
  }

}

cv::Mat EgoShapeFilterNode::polyfit(std::vector<cv::Point2f> &in_point, int n) {
  int size = in_point.size();

  int x_num = n + 1;

  cv::Mat mat_u(size, x_num, CV_64F);
  cv::Mat mat_y(size, 1, CV_64F);
  cv::Mat mat_k(x_num, 1, CV_64F);

  if (size == 0) {
    for (int i = 0; i < mat_k.rows; ++i) {
      mat_k.at<double>(i, 0) = 0;
    }
    return mat_k;
  } else {
    for (int i = 0; i < mat_u.rows; ++i)
      for (int j = 0; j < mat_u.cols; ++j) {
        mat_u.at<double>(i, j) = pow(in_point[i].x, j);
      }

    for (int i = 0; i < mat_y.rows; ++i) {
      mat_y.at<double>(i, 0) = in_point[i].y;
    }

    mat_k = (mat_u.t() * mat_u).inv() * mat_u.t() * mat_y;

    return mat_k;
  }
}

void EgoShapeFilterNode::EstimatePredictivePath(
    const boost::optional<Eigen::Vector4f> wall_plane_coeff,
    const cv::Mat &PolyCoefficientIn, nav_msgs::msg::Path &path_msg_out,
    const double &target_space_to_wall) {
  if (!wall_plane_coeff) {
    return;
  }
  
  int poly_order = 2;
  for (double x = -10; x < 30; x = x + 0.5) {
    geometry_msgs::msg::PoseStamped pose_buf;
    pose_buf.pose.position.x = x;
    pose_buf.pose.position.y =
        PolyCoefficientIn.at<double>(poly_order, 0) * pow(x, poly_order) +
        PolyCoefficientIn.at<double>(poly_order - 1, 0) * pow(x, poly_order - 1) +
        // PolyCoefficientIn.at<double>(poly_order - 2, 0) * pow(x, poly_order - 2) +
        target_space_to_wall; // Cubic

    pose_buf.pose.orientation.w = 1;
    pose_buf.pose.orientation.x = 0;
    pose_buf.pose.orientation.y = 0;
    pose_buf.pose.orientation.z = 0;
    path_msg_out.poses.push_back(pose_buf);
  }

  // 
}