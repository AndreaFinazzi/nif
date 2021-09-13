
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

  respond();

  std::cout << "left_lower_distance_: " << left_lower_distance_ << std::endl;
  std::cout << "right_lower_distance_: " << right_lower_distance_ << std::endl;
  std::cout << "rear_lower_distance_: " << rear_lower_distance_ << std::endl;
  std::cout << "front_lower_distance_: " << front_lower_distance_ << std::endl;

  std::cout << "left_upper_distance_: " << left_upper_distance_ << std::endl;
  std::cout << "right_upper_distance_: " << right_upper_distance_ << std::endl;
  std::cout << "rear_upper_distance_: " << rear_upper_distance_ << std::endl;
  std::cout << "front_upper_distance_: " << front_upper_distance_ << std::endl;

  std::cout << "height_lower_distance: " << height_lower_distance_ << std::endl;
  std::cout << "height_upper_distance: " << height_upper_distance_ << std::endl;
  std::cout << "resolution: " << resolution_ << std::endl;
  std::cout << "normal_angle_thres: " << normal_angle_thres_ << std::endl;
  std::cout << "ransac_pts_thresh: " << ransac_pts_thresh_ << std::endl;

  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();
  auto rmw_qos_profile = qos.get_rmw_qos_profile();

  sub_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/merged/lidar", qos,
      std::bind(&EgoShapeFilterNode::mergedPointsCallback, this,
                std::placeholders::_1));

  pub_filtered_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/merged/ego_filtered", qos);
  pub_oc_grid = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/OccupancyGridMap/map", qos);
  pub_forwarding_map_grid =
      this->create_publisher<nav_msgs::msg::OccupancyGrid>(
          "/OccupancyGridMap/forwarding_map", qos);

  pub_inverse_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/inverse_mapped_points", qos);
  pub_weaker_thres_inverse_points = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/weaker_thres_inverse_mapped_points", qos);
  pub_left_ransac_filtered_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/ransac_filtered_points/left", qos);
  pub_right_ransac_filtered_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/ransac_filtered_points/right", qos);
  pub_both_ransac_filtered_points =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(
          "/ransac_filtered_points/both", qos);

  pub_left_wall_line =
      this->create_publisher<nav_msgs::msg::Path>("/wall_left", qos);
  pub_right_wall_line =
      this->create_publisher<nav_msgs::msg::Path>("/wall_right", qos);

  pub_inner_wall_distance = this->create_publisher<std_msgs::msg::Float32>(
      "/detected_inner_distance", qos);
  pub_outer_wall_distance = this->create_publisher<std_msgs::msg::Float32>(
      "/detected_outer_distance", qos);
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

void EgoShapeFilterNode::mergedPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudVoxelized(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudShapeFiltered(
      new pcl::PointCloud<pcl::PointXYZI>);

  RCLCPP_DEBUG(this->get_logger(), "-------------");

  pcl::fromROSMsg(*msg, *CloudIn);
  CloudVoxelized = downsample(CloudIn, resolution_);

  EgoShape(CloudVoxelized, CloudShapeFiltered, left_lower_distance_,
           right_lower_distance_, front_lower_distance_, rear_lower_distance_,
           left_upper_distance_, right_upper_distance_, front_upper_distance_,
           rear_upper_distance_, height_lower_distance_,
           height_upper_distance_);
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*CloudShapeFiltered, cloud_msg);
  cloud_msg.header = msg->header;
  cloud_msg.header.frame_id = BASE_LINK;
  pub_filtered_points->publish(cloud_msg);

  float min_x = -(front_upper_distance_ + rear_upper_distance_) / 2;
  float min_y = -(left_upper_distance_ + right_upper_distance_) / 2;

  RegisterPointToGrid(CloudShapeFiltered, resolution_, min_x, min_y);

  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseBoth(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseLeft(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseRight(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudInverseWeakerThres(
      new pcl::PointCloud<pcl::PointXYZI>);

  InverseMap(CloudShapeFiltered, CloudInverseBoth,
             CloudInverseWeakerThres, CloudInverseLeft, CloudInverseRight, min_x,
             min_y, resolution_);

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

  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACLeft(
      new pcl::PointCloud<pcl::PointXYZI>);
  // left wall detection
  boost::optional<Eigen::Vector4f> left_wall =
      wall_detect(CloudInverseLeft, CloudRANSACLeft);

  inner_bound_distance = 0;
  outer_bound_distance = 0;
  // detected left_wall coefficients
  if (left_wall) {
    // for (int i = 0; i < 4; i++) {
    //   std::cout << "left wall" << (*left_wall)[i] << std::endl;
    // }
    inner_bound_distance = (*left_wall)[3];
    RCLCPP_DEBUG(this->get_logger(), "Left margin : %f", inner_bound_distance);
  }
  sensor_msgs::msg::PointCloud2 cloud_left_ransac_filtered_msg;
  pcl::toROSMsg(*CloudRANSACLeft, cloud_left_ransac_filtered_msg);
  cloud_left_ransac_filtered_msg.header.frame_id = BASE_LINK;
  cloud_left_ransac_filtered_msg.header.stamp = this->now();
  pub_left_ransac_filtered_points->publish(cloud_left_ransac_filtered_msg);
  std_msgs::msg::Float32 inner_bound_distance_msg;
  inner_bound_distance_msg.data = inner_bound_distance;
  pub_inner_wall_distance->publish(inner_bound_distance_msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACRight(
      new pcl::PointCloud<pcl::PointXYZI>);
  // left wall detection
  boost::optional<Eigen::Vector4f> right_wall =
      wall_detect(CloudInverseRight, CloudRANSACRight);

  // detected right_wall coefficients
  if (right_wall) {
    // for (int i = 0; i < 4; i++) {
    //   std::cout << "right wall" << (*right_wall)[i] << std::endl;
    // }
    outer_bound_distance = (*right_wall)[3];
    RCLCPP_DEBUG(this->get_logger(), "Right margin : %f", outer_bound_distance);
  }
  sensor_msgs::msg::PointCloud2 cloud_right_ransac_filtered_msg;
  pcl::toROSMsg(*CloudRANSACRight, cloud_right_ransac_filtered_msg);
  cloud_right_ransac_filtered_msg.header.frame_id = BASE_LINK;
  cloud_right_ransac_filtered_msg.header.stamp = this->now();
  pub_right_ransac_filtered_points->publish(cloud_right_ransac_filtered_msg);

  std_msgs::msg::Float32 outer_bound_distance_msg;
  outer_bound_distance_msg.data = outer_bound_distance;
  pub_outer_wall_distance->publish(outer_bound_distance_msg);

  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACBoth(
      new pcl::PointCloud<pcl::PointXYZI>);
  *CloudRANSACBoth += *CloudRANSACRight;
  *CloudRANSACBoth += *CloudRANSACLeft;

  sensor_msgs::msg::PointCloud2 cloud_both_ransac_filtered_msg;
  pcl::toROSMsg(*CloudRANSACBoth, cloud_both_ransac_filtered_msg);
  cloud_both_ransac_filtered_msg.header.frame_id = BASE_LINK;
  cloud_both_ransac_filtered_msg.header.stamp = this->now();
  pub_both_ransac_filtered_points->publish(cloud_both_ransac_filtered_msg);

  pub_left_wall_line->publish(LineVisualizer(
      CloudRANSACLeft, left_wall, front_upper_distance_, rear_upper_distance_));
  pub_right_wall_line->publish(LineVisualizer(CloudRANSACRight, right_wall,
                                              front_upper_distance_,
                                              rear_upper_distance_));

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

nav_msgs::msg::Path
EgoShapeFilterNode::LineVisualizer(pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
                                   const boost::optional<Eigen::Vector4f> coeff,
                                   double in_front_upper_threshold,
                                   double in_rear_upper_threshold) {

  std::vector<cv::Point2f> ToBeFit;
  ToBeFit.clear();

  for (auto point_buf : cloudIn->points) {
    cv::Point2f pointTmp;
    pointTmp.x = point_buf.x;
    pointTmp.y = point_buf.y;
    ToBeFit.push_back(pointTmp);
  }

  // For polynomial fitting
  int poly_order = 3;
  cv::Mat PolyCoefficient = polyfit(ToBeFit, poly_order);

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = BASE_LINK;
  path_msg.header.stamp = this->now();

  if (!coeff) {
    return path_msg;
  }

  for (double x = -10; x < 20; x = x + 0.5) {
    geometry_msgs::msg::PoseStamped pose_buf;
    pose_buf.pose.position.x = x;
    pose_buf.pose.position.y =
        PolyCoefficient.at<double>(poly_order, 0) * pow(x, poly_order) +
        PolyCoefficient.at<double>(poly_order - 1, 0) * pow(x, poly_order - 1) +
        PolyCoefficient.at<double>(poly_order - 2, 0) * pow(x, poly_order - 2) +
        PolyCoefficient.at<double>(poly_order - 3, 0); // Cubic
    pose_buf.pose.orientation.w = 1;
    pose_buf.pose.orientation.x = 0;
    pose_buf.pose.orientation.y = 0;
    pose_buf.pose.orientation.z = 0;
    path_msg.poses.push_back(pose_buf);
  }

  return path_msg;
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
