
/*
 * ego_shape_filter_node.cpp
 *
 *  Created on: Aug 10, 2021
 *      Author: Daegyu Lee
 */

#include <nif_points_preprocessor_nodes/ego_shape_filter_node.h>
#include "nif_frame_id/frame_id.h"
#include <numeric>

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

  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/merged/lidar", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&EgoShapeFilterNode::mergedPointsCallback, this,
                std::placeholders::_1));

  using namespace std::chrono_literals; // NOLINT
  // TODO convert period to paramter
  timer_ = this->create_wall_timer(
      30ms, std::bind(&EgoShapeFilterNode::timer_callback, this));

  pub_filtered_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/merged/ego_filtered", nif::common::constants::QOS_SENSOR_DATA);
  pub_oc_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/OccupancyGridMap/map", nif::common::constants::QOS_SENSOR_DATA);

  pub_inverse_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/inverse_mapped_points", nif::common::constants::QOS_SENSOR_DATA);

  pub_inverse_left_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/inverse_left_mapped_points",
          nif::common::constants::QOS_SENSOR_DATA);
  pub_inverse_right_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/inverse_right_mapped_points",
          nif::common::constants::QOS_SENSOR_DATA);

  pub_weaker_thres_inverse_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/inverse_weaker_thres_mapped_points",
          nif::common::constants::QOS_SENSOR_DATA);

  lidar_timeout = rclcpp::Duration(1, 0);

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
  
  pcl::PointIndices::Ptr out_indices_list(new pcl::PointIndices);
  for (unsigned int i = 0; i < in_cloud_ptr->points.size(); i++) {
    auto& x = in_cloud_ptr->points[i].x;
    auto& y = in_cloud_ptr->points[i].y;
    auto& z = in_cloud_ptr->points[i].z;

    // outer side boundaries
    if (y > in_left_upper_threshold ||
        y < -in_right_upper_threshold) {
      out_indices_list->indices.push_back(i);
      continue;
    }

    // outer front/rear boundaries
    if (x > in_front_upper_threshold ||
        x < -in_rear_upper_threshold) {
      out_indices_list->indices.push_back(i);
      continue;
    }

    if (z < (in_height_lower_threshold)) {
      out_indices_list->indices.push_back(i);
      continue;
    }
    if (z > (in_height_upper_threshold)) {
      out_indices_list->indices.push_back(i);
      continue;
    }

    if ( y < in_left_lower_threshold  && y > -1.0 * in_right_lower_threshold &&
         x < in_front_lower_threshold && x > -1.0 * in_rear_lower_threshold )
      {
        out_indices_list->indices.push_back(i);
        continue;
      }
      
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_cloud_ptr);
  extract.setIndices(out_indices_list);
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

  //both
  sensor_msgs::msg::PointCloud2 cloud_inverse_msg;
  pcl::toROSMsg(*CloudInverseBoth, cloud_inverse_msg);
  cloud_inverse_msg.header.frame_id = BASE_LINK;
  cloud_inverse_msg.header.stamp = this->now();
  pub_inverse_points->publish(cloud_inverse_msg);

  // // left
  // sensor_msgs::msg::PointCloud2 left_cloud_inverse_msg;
  // pcl::toROSMsg(*CloudInverseLeft, left_cloud_inverse_msg);
  // left_cloud_inverse_msg.header.frame_id = BASE_LINK;
  // left_cloud_inverse_msg.header.stamp = this->now();
  // pub_inverse_left_points->publish(left_cloud_inverse_msg);

  // // right
  // sensor_msgs::msg::PointCloud2 right_cloud_inverse_msg;
  // pcl::toROSMsg(*CloudInverseRight, right_cloud_inverse_msg);
  // right_cloud_inverse_msg.header.frame_id = BASE_LINK;
  // right_cloud_inverse_msg.header.stamp = this->now();
  // pub_inverse_right_points->publish(right_cloud_inverse_msg);

  sensor_msgs::msg::PointCloud2 cloud_weaker_thres_inverse_msg;
  pcl::toROSMsg(*CloudInverseWeakerThres, cloud_weaker_thres_inverse_msg);
  cloud_weaker_thres_inverse_msg.header.frame_id = BASE_LINK;
  cloud_weaker_thres_inverse_msg.header.stamp = this->now();
  pub_weaker_thres_inverse_points->publish(cloud_weaker_thres_inverse_msg);

}

void EgoShapeFilterNode::mergedPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

  std::lock_guard<std::mutex> sensor_lock(sensor_mtx);

  lidar_time_last_update = this->now();

  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudVoxelized(
      new pcl::PointCloud<pcl::PointXYZI>);
  m_CloudShapeFiltered.reset(new pcl::PointCloud<pcl::PointXYZI>());

  RCLCPP_DEBUG(this->get_logger(), "-------------");

  pcl::fromROSMsg(*msg, *CloudVoxelized);
  downsample(CloudVoxelized, resolution_);

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

  for (int i = 0; i < MAP_HEIGHT + 1; i++) {
    this->map[i].fill(0.0);
    this->count_map[i].fill(0.0);
    this->mean_map[i].fill(0.0);
    this->cov_map[i].fill(0.0);
    this->points_map[i].fill({});
  }

  int x, y;
  for (auto point_buf : in_cloud_ptr->points) {
    x = std::floor((point_buf.x - min_x) / in_resolution);
    y = std::floor((point_buf.y - min_y) / in_resolution);

    if (x > MAP_WIDTH || x < 0 || y > MAP_HEIGHT || y < 0) {
      // std::cout << x << ", " << y << std::endl;
      continue;
    }
    // count_map[y][x] = count_map[y][x] + 1.f; // count hit

    // std::vector<double> points_map_buf = points_map[y][x];
    // points_map_buf.push_back(point_buf.z);
    count_map[y][x] = count_map[y][x] + 1.f; // count hit
    // points_map[y][x] = points_map_buf;
    // map[y][x] = map[y][x] + point_buf.z + 0.5; // accumulate height
    // double sum_of_elems =
    //     std::accumulate(points_map_buf.begin(), points_map_buf.end(),
    //                     decltype(points_map_buf)::value_type(0));

    // if(count_map[y][x] != 0.)
    // {
    //   mean_map[y][x] = sum_of_elems / count_map[y][x]; // calculate mean map
    //   for(auto z : points_map_buf)
    //   {
    //     cov_map[y][x] = cov_map[y][x] * count_map[y][x]; 
    //     cov_map[y][x] = (cov_map[y][x] + pow((z - mean_map[y][x]),2)) / count_map[y][x];
    //   }
    // }
  }

  for (int i = 0; i < MAP_HEIGHT; i++) {
    for (int j = 0; j < MAP_WIDTH; j++) {
      if (count_map[i][j] > count_threshold_) {
        // oc_grid_msg.data.push_back((int8_t)(map[i][j]));
        oc_grid_msg.data.push_back((int8_t)(80.));
      } else {
        oc_grid_msg.data.push_back((int8_t)(0.0));
      }
    }
  }
  pub_oc_grid->publish(oc_grid_msg);
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
    if (count_map[y][x] > count_threshold_) {
      cloudOut->points.push_back(point_buf);
      if (point_buf.y > 0) {
        cloudLeftOut->points.push_back(point_buf);
      } else {
        cloudRightOut->points.push_back(point_buf);
      }
    }
    if (count_map[y][x] > count_threshold_ - 1.) {
      WeakerThrescloudOut->points.push_back(point_buf);
    }
  }
}

void
EgoShapeFilterNode::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                               double resolution) {

  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, 0.02);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*cloud);
}
