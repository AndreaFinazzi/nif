
/*
 * wall_detection_node.cpp
 *
 *  Created on: Oct 6, 2021
 *      Author: Daegyu Lee
 */
#include <nif_points_preprocessor_nodes/wall_detection_node.h>
#include "nif_frame_id/frame_id.h"
#include <numeric>

using namespace nif::perception;
using namespace nif::common::frame_id::localization;

WallDetectionNode::WallDetectionNode(const std::string &node_name_)
    : Node(node_name_) {
  this->declare_parameter<double>("rear_upper_distance", double(50.0));
  this->declare_parameter<double>("front_upper_distance", double(50.0));

  this->declare_parameter<double>("normal_angle_thres", double(50.));
  this->declare_parameter<int>("ransac_pts_thresh", int(200));
  this->declare_parameter<double>("ransac_distance_thres", double(0.2));

  this->declare_parameter<double>("x_roi", double(0.));
  this->declare_parameter<double>("distance_extract_thres", double(0.5));
  this->declare_parameter<double>("distance_low_fass_filter", double(0.5));
  this->declare_parameter<double>("target_space_to_wall", double(7.));

  // Controller Parameters
  this->declare_parameter<int>("average_filter_size", int(10));
  this->declare_parameter<double>("min_lookahead", double(4.0));
  this->declare_parameter<double>("max_lookahead", double(30.0));
  this->declare_parameter<double>("lookahead_speed_ratio", double(0.75));
  this->declare_parameter<double>("proportional_gain", double(0.2));
  this->declare_parameter<double>("vehicle.wheelbase", double(2.97));
  this->declare_parameter<double>("max_steer_angle", double(30.0)); // 15 deg * 2 because ratio is wrong

  respond();


  RCLCPP_INFO(this->get_logger(), "normal_angle_thres: %f", normal_angle_thres_);
  RCLCPP_INFO(this->get_logger(), "ransac_pts_thresh: %f", ransac_pts_thresh_);
  RCLCPP_INFO(this->get_logger(), "x_roi : %f", extract_distance_x_roi);
  RCLCPP_INFO(this->get_logger(), "distance_extract_thres : %f", extract_distance_thres );
  RCLCPP_INFO(this->get_logger(), "distance_low_fass_filter : %f", distance_low_fass_filter );
  RCLCPP_INFO(this->get_logger(), "target_space_to_wall : %f", m_target_space_to_wall );

  sub_inverse_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/inverse_left_mapped_points",
      nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&WallDetectionNode::InverseLeftCallback, this,
                std::placeholders::_1));

  sub_inverse_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/inverse_right_mapped_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&WallDetectionNode::InverseRightCallback, this,
                std::placeholders::_1));

  sub_wheel_speed_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
      "raptor_dbw_interface/wheel_speed_report",
      nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&WallDetectionNode::WheelSpeedCallback, this,
                std::placeholders::_1));
  sub_radar_marker_ =
      this->create_subscription<visualization_msgs::msg::Marker>(
          "/radar_front/radar_visz_moving",
          nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&WallDetectionNode::RadarMarkerCallback, this,
                    std::placeholders::_1));

  using namespace std::chrono_literals; // NOLINT
  // TODO convert period to paramter
  timer_ = this->create_wall_timer(
      50ms, std::bind(&WallDetectionNode::timer_callback, this));

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


  // kin controller
  SetControllerParams();
  m_KinController.setCmdsToZeros();
}

WallDetectionNode::~WallDetectionNode() {}

void WallDetectionNode::respond() {
  this->get_parameter("rear_upper_distance", rear_upper_distance_);
  this->get_parameter("front_upper_distance", front_upper_distance_);
  
  this->get_parameter("normal_angle_thres", normal_angle_thres_);
  this->get_parameter("ransac_pts_thresh", ransac_pts_thresh_);
  this->get_parameter("ransac_distance_thres",m_ransacDistanceThres);

  this->get_parameter("x_roi", extract_distance_x_roi);
  this->get_parameter("distance_extract_thres", extract_distance_thres);
  this->get_parameter("distance_low_fass_filter", distance_low_fass_filter);
  this->get_parameter("target_space_to_wall", m_target_space_to_wall);

  m_average_filter_size = this->get_parameter("average_filter_size").as_int();
}

void WallDetectionNode::SetControllerParams() {
  m_KinController.min_la = this->get_parameter("min_lookahead").as_double();
  m_KinController.max_la = this->get_parameter("max_lookahead").as_double();
  m_KinController.la_ratio = this->get_parameter("lookahead_speed_ratio").as_double();
  m_KinController.Kp = this->get_parameter("proportional_gain").as_double();
  m_KinController.L = this->get_parameter("vehicle.wheelbase").as_double();
  m_KinController.ms = this->get_parameter("max_steer_angle").as_double();
}


void WallDetectionNode::timer_callback() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACLeft(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudRANSACRight(
      new pcl::PointCloud<pcl::PointXYZI>);

  boost::optional<Eigen::Vector4f> left_wall_plane_coeff;
  boost::optional<Eigen::Vector4f> right_wall_plane_coeff;

  if (bInverseLeftPoints) {
    /* LEFT RANSAC FILTER
    1. Remove outlier points
    2. Implement left/right points separately.
      - input : left ground filtered points
      - output : distance to the wall, ransac filtered left wall points
    */
    // left wall detection
    left_wall_plane_coeff =
        wall_detect(m_InverseLeftPoints, CloudRANSACLeft);

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
  }

  if (bInverseRightPoints)
  {
    /* RIGHT WALL BASED
    1. Remove outlier points
    2. Implement left/right points separately.
      - input : right ground filtered points
      - output : distance to the wall, ransac filtered right wall points
    */
    // right wall detection
    right_wall_plane_coeff =
        wall_detect(m_InverseRightPoints, CloudRANSACRight);

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
  }

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

//  CubicSpliner(CloudRANSACLeft, left_wall_plane_coeff, front_upper_distance_,
//               rear_upper_distance_, left_path_msg, LeftPolyCoefficient);

  // changed
  int left_polynorm_order = -1;
  CubicSpliner(CloudRANSACLeft, left_wall_plane_coeff, front_upper_distance_,
               rear_upper_distance_, left_path_msg, LeftPolyCoefficient, left_polynorm_order);
  pub_left_wall_line->publish(left_path_msg);

  nav_msgs::msg::Path right_path_msg;
  right_path_msg.header.frame_id = BASE_LINK;
  right_path_msg.header.stamp = this->now();
  cv::Mat RightPolyCoefficient;

//  CubicSpliner(CloudRANSACRight, right_wall_plane_coeff, front_upper_distance_,
//               rear_upper_distance_, right_path_msg, RightPolyCoefficient);
  // changed
  int right_polynorm_order = -1;
  CubicSpliner(CloudRANSACRight, right_wall_plane_coeff, front_upper_distance_,
               rear_upper_distance_, right_path_msg, RightPolyCoefficient, right_polynorm_order);
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
  bool close_to_left = false;
  if (right_wall_plane_coeff && outer_bound_distance != 0.0) {
    if (fabs(outer_bound_distance) < m_target_space_to_wall)
    {
      m_margin_to_wall = outer_bound_distance + m_target_space_to_wall;
    }
    else {
      m_margin_to_wall = 0.0;
    }
  }

  if (left_wall_plane_coeff && inner_bound_distance != 0.0) {
    if (fabs(inner_bound_distance) < m_target_space_to_wall) {
      m_margin_to_wall = inner_bound_distance - m_target_space_to_wall;
      close_to_left = true;
    } else {
      m_margin_to_wall = 0.0;
    }
  }

  // if vehicle goes close to the left wall, calculate predictive path from left wall, not from the right wall. 
  if(close_to_left)
  {
    EstimatePredictivePath(left_wall_plane_coeff, LeftPolyCoefficient,
                           wall_folllowing_path_msg, m_margin_to_wall);
  }
  else
  {
    EstimatePredictivePath(right_wall_plane_coeff, RightPolyCoefficient,
                           wall_folllowing_path_msg, m_margin_to_wall);
  }

  final_wall_following_path_msg.header.frame_id = BASE_LINK;
  final_wall_following_path_msg.header.stamp = this->now();
  if (right_wall_plane_coeff && !wall_folllowing_path_msg.poses.empty()) {
     final_wall_following_path_msg = wall_folllowing_path_msg;
  }
  if (left_wall_plane_coeff && !wall_folllowing_path_msg.poses.empty() && close_to_left)
  {
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


  double wall_following_steering_cmd = 0.0;
  m_KinController.getSteering(wall_following_steering_cmd);
  
  control_out_que.push_back(wall_following_steering_cmd);
  if(control_out_que.size() > m_average_filter_size)
  {
    control_out_que.pop_front();
  }
  size_t filter_que_size = control_out_que.size();

  double filter_que_sum = 0.0;
  for(auto cmd : control_out_que)
  {
    filter_que_sum += cmd;
  }
  double filtered_steering_output = filter_que_sum / filter_que_size;
  // if vehicle is not close to the outer wall, but steering output is negative, cut steering cmd as zero.
  // normally false negative output was calculated when vehicle proceeds from straight to curvy road.
  // However, if vehicle is close to the left wall, this saturator does not work.    
  if (!close_to_left && filtered_steering_output < 0.)
  {
    filtered_steering_output = 0.0 ;
  }

  std_msgs::msg::Float32 wall_following_steering_cmd_msg;
  wall_following_steering_cmd_msg.data = filtered_steering_output;
  pub_wall_following_steer_cmd->publish(wall_following_steering_cmd_msg);

}

void WallDetectionNode::InverseLeftCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  m_InverseLeftPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_InverseLeftPoints);
  bInverseLeftPoints = true;
}
void WallDetectionNode::InverseRightCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  m_InverseRightPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_InverseRightPoints);
  bInverseRightPoints = true;
}

void WallDetectionNode::WheelSpeedCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
{
  m_vel_speed_x = (msg->front_right + msg->front_left) / 2 * nif::common::constants::KPH2MS;
}

void WallDetectionNode::RadarMarkerCallback(
    const visualization_msgs::msg::Marker::SharedPtr msg) {

}

pcl::PointCloud<pcl::PointXYZI>::Ptr
WallDetectionNode::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
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
WallDetectionNode::wall_detect(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut) {

  // too few points for RANSAC
  if (cloud->size() < 50) {
    return boost::none;
  }

  // RANSAC
  pcl::SampleConsensusModelPlane<pcl::PointXYZI>::Ptr model_p(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZI>(cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZI> ransac(model_p);
  ransac.setDistanceThreshold(m_ransacDistanceThres);
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

void WallDetectionNode::ExtractDistanceInCloud(
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

void WallDetectionNode::CubicSpliner(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
        const boost::optional<Eigen::Vector4f> wall_plane_coeff,
        double in_front_upper_threshold, double in_rear_upper_threshold,
        nav_msgs::msg::Path& path_msg_out, cv::Mat& PolyCoefficientOut) {
  if (cloudIn->points.empty())
    return;

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

void WallDetectionNode::CubicSpliner(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn,
    const boost::optional<Eigen::Vector4f> wall_plane_coeff,
    double in_front_upper_threshold, double in_rear_upper_threshold,
    nav_msgs::msg::Path& path_msg_out, cv::Mat& PolyCoefficientOut, int& poly_order) {
  if (cloudIn->points.empty())
    return;

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
  int first_order = 1;
  cv::Mat first_order_poly_coeff = polyfit(ToBeFit, first_order);
  double first_order_error = 0.0;

  // For polynomial fitting
  int second_order = 2;
  cv::Mat second_order_poly_coeff = polyfit(ToBeFit, second_order);
  double second_order_error = 0.0;



  if (!wall_plane_coeff) {
    return;
  }

  // check which polynorm model is more fitted
  for(int i =0 ; i < ToBeFit.size(); i++) {
    first_order_error += pow(ToBeFit[i].y -
                                       (first_order_poly_coeff.at<double>(first_order, 0) * pow(ToBeFit[i].x, first_order) +
                                       first_order_poly_coeff.at<double>(first_order - 1, 0)),2);
    second_order_error += pow(ToBeFit[i].y -
                                       (second_order_poly_coeff.at<double>(second_order, 0) * pow(ToBeFit[i].x, second_order) +
                                        second_order_poly_coeff.at<double>(second_order - 1, 0) * pow(ToBeFit[i].x, second_order - 1) +
                                        second_order_poly_coeff.at<double>(second_order - 2, 0)),2);
  }


  if(second_order_error > first_order_error){
    poly_order = first_order;
    PolyCoefficientOut = first_order_poly_coeff;
  }
  else{
    poly_order = second_order;
    PolyCoefficientOut = second_order_poly_coeff;
  }

  for (double x = -30; x < 50; x = x + 2.0) {
    geometry_msgs::msg::PoseStamped pose_buf;
    pose_buf.pose.position.x = x;
    if (poly_order == second_order) {
      pose_buf.pose.position.y =
          second_order_poly_coeff.at<double>(poly_order, 0) * pow(x, poly_order) +
          second_order_poly_coeff.at<double>(poly_order - 1, 0) * pow(x, poly_order - 1) +
          second_order_poly_coeff.at<double>(poly_order - 2, 0);
    } else {
      pose_buf.pose.position.y =
          first_order_poly_coeff.at<double>(poly_order, 0) * pow(x, poly_order) +
          first_order_poly_coeff.at<double>(poly_order - 1, 0);
    }
    pose_buf.pose.orientation.w = 1;
    pose_buf.pose.orientation.x = 0;
    pose_buf.pose.orientation.y = 0;
    pose_buf.pose.orientation.z = 0;
    path_msg_out.poses.push_back(pose_buf);
  }

}

cv::Mat WallDetectionNode::polyfit(std::vector<cv::Point2f> &in_point, int n) {
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

void WallDetectionNode::EstimatePredictivePath(
    const boost::optional<Eigen::Vector4f> wall_plane_coeff,
    const cv::Mat &PolyCoefficientIn, nav_msgs::msg::Path &path_msg_out,
    const double &target_space_to_wall) {
  if (!wall_plane_coeff) {
    return;
  }
  
  int poly_order = 2;
  for (double x = -10; x < 50; x = x + 0.5) {
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

void WallDetectionNode::EstimatePredictivePath(
    const boost::optional<Eigen::Vector4f> wall_plane_coeff,
    const cv::Mat &PolyCoefficientIn, const int& poly_order,  nav_msgs::msg::Path &path_msg_out,
    const double &target_space_to_wall) {
  if (!wall_plane_coeff) {
    return;
  }

  for (double x = -10; x < 30; x = x + 0.5) {
    geometry_msgs::msg::PoseStamped pose_buf;
    pose_buf.pose.position.x = x;

    if (poly_order == 2) {
      pose_buf.pose.position.y =
          PolyCoefficientIn.at<double>(2, 0) * pow(x, 2) +
          PolyCoefficientIn.at<double>(1, 0) * pow(x, 1) +
          target_space_to_wall; // Cubic
    } else {
      pose_buf.pose.position.y =
          PolyCoefficientIn.at<double>(1, 0) * pow(x, 1) +
          target_space_to_wall;
    }

    pose_buf.pose.orientation.w = 1;
    pose_buf.pose.orientation.x = 0;
    pose_buf.pose.orientation.y = 0;
    pose_buf.pose.orientation.z = 0;
    path_msg_out.poses.push_back(pose_buf);
  }

  //
}