//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 09/08/21.
//
#include "localization_ekf_nodes/load_geofence_node.h"
#include "nif_frame_id/frame_id.h"

using namespace nif::localization::geofence;
using namespace nif::common::frame_id::localization;

// Constructor
GeoFenceLoader::GeoFenceLoader(const std::string &node_name_)
    : Node(node_name_) {
  this->declare_parameter<std::string>(
      "outer_geofence_filename", "");
  this->declare_parameter<std::string>(
      "inner_geofence_filename", "");

  this->m_OuterGeoFenceFileName = this->get_parameter("outer_geofence_filename").as_string();     
  this->m_InnerGeoFenceFileName = this->get_parameter("inner_geofence_filename").as_string();    

  if (this->m_OuterGeoFenceFileName.length() <= 0 || 
      this->m_InnerGeoFenceFileName.length() <= 0) {
        throw std::runtime_error("Invalid pointcloud paths.");
      }

  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.best_effort();

  pubOuterGeofence = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/geofence_outer", qos);
  pubInnerGeofence = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/geofence_inner", qos);

  pubInnerGeofencePath =
      this->create_publisher<nav_msgs::msg::Path>("/geofence_path_inner", qos);
  pubOuterGeofencePath =
      this->create_publisher<nav_msgs::msg::Path>("/geofence_path_outer", qos);
  pubInnerSegmentPath =
      this->create_publisher<nav_msgs::msg::Path>("/segment_path_inner", qos);
  pubOuterSegmentPath =
      this->create_publisher<nav_msgs::msg::Path>("/segment_path_outer", qos);

  pubInnerDistance = this->create_publisher<std_msgs::msg::Float32>(
      "/geofence_inner_distance", qos);
  pubOuterDistance = this->create_publisher<std_msgs::msg::Float32>(
      "/geofence_outer_distance", qos);

  // TODO : use this "ON_THE_TRACK" flag in the system state manager
  pubOnTheTrack =
      this->create_publisher<std_msgs::msg::Bool>("/bool/on_the_track", qos);

  subOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      "/out_odometry_ekf_estimated", qos,
      std::bind(&GeoFenceLoader::EKFOdometryCallback, this,
                std::placeholders::_1));


  std::cout << "outer_geofence_filename: " << m_OuterGeoFenceFileName
            << std::endl;
  std::cout << "inner_geofence_filename: " << m_InnerGeoFenceFileName
            << std::endl;

  OuterFencePCDFileIO();
  InnerFencePCDFileIO();

  using namespace std::chrono_literals; // NOLINT
  timer_ = this->create_wall_timer(10ms, [this]() {
    if (!bOuterLoaded) {
      OuterFencePCDFileIO();
    }
    if (!bInnerLoaded) {
      InnerFencePCDFileIO();
    }

    double minDistToInner = DBL_MAX;
    double minDistToOuter = DBL_MAX;
    nav_msgs::msg::Path GeofenceInnerMsg;
    nav_msgs::msg::Path GeofenceOuterMsg;
    nav_msgs::msg::Path SegmentInnerMsg;
    nav_msgs::msg::Path SegmentOuterMsg;

    GeofenceInnerMsg.header.frame_id = BASE_LINK;
    GeofenceInnerMsg.header.stamp = this->now();
    GeofenceOuterMsg.header.frame_id = BASE_LINK;
    GeofenceOuterMsg.header.stamp = this->now();

    SegmentInnerMsg.header.frame_id = ODOM;
    SegmentInnerMsg.header.stamp = this->now();
    SegmentOuterMsg.header.frame_id = ODOM;
    SegmentOuterMsg.header.stamp = this->now();
    std_msgs::msg::Bool OnTheTrackMsg;
    OnTheTrackMsg.data = false;

    if (bInnerLoaded) {
      Projector(m_InnerGeoFence, m_veh_x, m_veh_y, minDistToInner,
                SegmentInnerMsg);
      GeoFenceOnBody(m_InnerGeoFence, m_veh_x, m_veh_y, m_veh_yaw,
                     GeofenceInnerMsg);
      pubInnerGeofencePath->publish(GeofenceInnerMsg);
      pubInnerSegmentPath->publish(SegmentInnerMsg);
      std_msgs::msg::Float32 minDistToInnerMsg;
      minDistToInnerMsg.data = minDistToInner;
      if (fabs(minDistToInner) < 50.0)
        pubInnerDistance->publish(minDistToInnerMsg);

      if (minDistToInner > 0.0)
        OnTheTrackMsg.data = true;

      sensor_msgs::msg::PointCloud2 InnerGeoFenceCloudMsg;
      pcl::toROSMsg(*m_InnerFenceCloud, InnerGeoFenceCloudMsg);
      InnerGeoFenceCloudMsg.header.frame_id = ODOM;
      InnerGeoFenceCloudMsg.header.stamp = this->now();
      pubInnerGeofence->publish(InnerGeoFenceCloudMsg);
    }

    pubOnTheTrack->publish(OnTheTrackMsg);

    if (bOuterLoaded) {
      Projector(m_OuterGeoFence, m_veh_x, m_veh_y, minDistToOuter,
                SegmentOuterMsg);
      GeoFenceOnBody(m_OuterGeoFence, m_veh_x, m_veh_y, m_veh_yaw,
                     GeofenceOuterMsg);
      pubOuterGeofencePath->publish(GeofenceOuterMsg);
      pubOuterSegmentPath->publish(SegmentOuterMsg);
      std_msgs::msg::Float32 minDistToOuterMsg;
      minDistToOuterMsg.data = minDistToOuter;
      if (fabs(minDistToOuter) < 50.0)
        pubOuterDistance->publish(minDistToOuterMsg);

      sensor_msgs::msg::PointCloud2 OuterGeoFenceCloudMsg;
      pcl::toROSMsg(*m_OuterFenceCloud, OuterGeoFenceCloudMsg);
      OuterGeoFenceCloudMsg.header.frame_id = ODOM;
      OuterGeoFenceCloudMsg.header.stamp = this->now();
      pubOuterGeofence->publish(OuterGeoFenceCloudMsg);
    }

    // std::cout << "minDistToInner: " << minDistToInner << std::endl;
    // std::cout << "minDistToOuter: " << minDistToOuter << std::endl;
    // std::cout << "------------" << std::endl;
  });
}

GeoFenceLoader::~GeoFenceLoader() {}

void GeoFenceLoader::OuterFencePCDFileIO() {
  m_OuterFenceCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn(
      new pcl::PointCloud<pcl::PointXYZI>);

  //  read m_OuterFenceCloud from a pcd file
  RCLCPP_INFO(this->get_logger(), "LODAING OUTER GEO-FENCE...");

  pcl::io::loadPCDFile(m_OuterGeoFenceFileName, *CloudIn);
  if (CloudIn->size() > 0) {
    RCLCPP_INFO(this->get_logger(), "OUTER GEO-FENCE IS LOADED!");

    for (auto point : CloudIn->points) {
      std::pair<double, double> xy_buf;
      xy_buf.first = point.x;
      xy_buf.second = point.y;
      m_OuterGeoFence.push_back(xy_buf);
    }
    std::pair<double, double> first_point;
    first_point.first = CloudIn->points[0].x;
    first_point.second = CloudIn->points[0].y;
    m_OuterGeoFence.push_back(first_point);

  } else {
    RCLCPP_INFO(this->get_logger(), "...");
    return;
  }
  *m_OuterFenceCloud = *CloudIn;

  m_OuterFenceCloud->header.frame_id = ODOM;
  bOuterLoaded = true;
}

void GeoFenceLoader::InnerFencePCDFileIO() {
  m_InnerFenceCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn(
      new pcl::PointCloud<pcl::PointXYZI>);
  //  read m_InnerFenceCloud from a pcd file
  RCLCPP_INFO(this->get_logger(), "LODAING INNER GEO-FENCE...");
  pcl::io::loadPCDFile(m_InnerGeoFenceFileName, *CloudIn);
  if (CloudIn->size() > 0) {
    RCLCPP_INFO(this->get_logger(), "INNER GEO-FENCE IS LOADED!");
    for (auto point : CloudIn->points) {
      std::pair<double, double> xy_buf;
      xy_buf.first = point.x;
      xy_buf.second = point.y;
      m_InnerGeoFence.push_back(xy_buf);
    }
    std::pair<double, double> first_point;
    first_point.first = CloudIn->points[0].x;
    first_point.second = CloudIn->points[0].y;
    m_InnerGeoFence.push_back(first_point);

  } else {
    RCLCPP_INFO(this->get_logger(), "...");
    return;
  }
  *m_InnerFenceCloud = *CloudIn;

  m_InnerFenceCloud->header.frame_id = ODOM;
  bInnerLoaded = true;
}

void GeoFenceLoader::EKFOdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  m_veh_x = msg->pose.pose.position.x;
  m_veh_y = msg->pose.pose.position.y;

  tf2::Quaternion tf_quat;
  tf2::convert(msg->pose.pose.orientation, tf_quat);
  tf2::Matrix3x3 mat(tf_quat);
  mat.getRPY(m_veh_roll, m_veh_pitch, m_veh_yaw);
}

void GeoFenceLoader::GeoFenceOnBody(
    const std::vector<std::pair<double, double>> &array_in,
    const double &veh_x_, const double &veh_y_, const double &veh_yaw_,
    nav_msgs::msg::Path &PathOut) {

  for (auto point_buf : array_in) {
    double veh_x_body = (point_buf.first - veh_x_) * cos(veh_yaw_) +
                        (point_buf.second - veh_y_) * sin(veh_yaw_);
    double veh_y_body = -(point_buf.first - veh_x_) * sin(veh_yaw_) +
                        (point_buf.second - veh_y_) * cos(veh_yaw_);
    geometry_msgs::msg::PoseStamped poseBuf;
    poseBuf.pose.position.x = veh_x_body;
    poseBuf.pose.position.y = veh_y_body;
    poseBuf.pose.orientation.w = 1;
    PathOut.poses.push_back(poseBuf);
  }
}

void GeoFenceLoader::Projector(
    const std::vector<std::pair<double, double>> &array_in,
    const double &veh_x_, const double &veh_y_, double &distance,
    nav_msgs::msg::Path &SegmentOut) {
  double bias, normal_distance;
  double min_distance = DBL_MAX;
  int minWayIndex = 0;
  int minNodeIndex = 0;

  bool first_point = true;
  double prev_x, prev_y;
  int idx = 0;
  double sign;
  for (auto point_buf : array_in) {

    if (first_point) {
      prev_x = point_buf.first;
      prev_y = point_buf.second;
      first_point = false;
      idx = idx + 1;
      continue;
    }

    double slope;
    if (point_buf.first - prev_x == 0) {
      slope = 0;
    } else {
      slope = (point_buf.second - prev_y) / (point_buf.first - prev_x);
    }
    bias = point_buf.second - slope * point_buf.first;
    normal_distance =
        fabs((slope * veh_x_ - veh_y_ + bias) / sqrt(pow(slope, 2) + 1));

    double prod1 = (veh_x_ - prev_x) * (point_buf.first - prev_x) +
                   (veh_y_ - prev_y) * (point_buf.second - prev_y);

    double prod2 = (veh_x_ - point_buf.first) * (prev_x - point_buf.first) +
                   (veh_y_ - point_buf.second) * (prev_y - point_buf.second);

    /* element of z-directional cross product (i, j, k)
      i = v_A[1] * v_B[2] - v_A[2] * v_B[1];
      j = -(v_A[0] * v_B[2] - v_A[2] * v_B[0]);
      k = v_A[0] * v_B[1] - v_A[1] * v_B[0];
      v_A = { prev_x - veh_x_, prev_y - veh_y_}
          --> v_A[0] = prev_x - veh_x_
          --> v_A[1] = prev_y - veh_y_
      v_B = { point_buf.first - veh_x_, point_buf.second - veh_y_}
          --> v_B[0] = point_buf.first - veh_x_
          --> v_B[1] = point_buf.second - veh_y_
    */

    if (normal_distance < min_distance && prod1 > 0 && prod2 > 0) {
      min_distance = normal_distance;

      SegmentOut.poses.clear();
      geometry_msgs::msg::PoseStamped pose_buf;
      pose_buf.pose.position.x = prev_x;
      pose_buf.pose.position.y = prev_y;
      pose_buf.pose.orientation.w = 1;
      SegmentOut.poses.push_back(pose_buf);

      pose_buf.pose.position.x = point_buf.first;
      pose_buf.pose.position.y = point_buf.second;
      pose_buf.pose.orientation.w = 1;
      SegmentOut.poses.push_back(pose_buf);

      double cross_prod_z = ((prev_x - veh_x_) * (point_buf.second - veh_y_) -
                             (prev_y - veh_y_) * (point_buf.first - veh_x_));
      // std::cout << cross_prod_z << std::endl;

      if (cross_prod_z > 0.) {
        sign = 1.0;
      } else {
        sign = -1.0;
      }
    }
    idx = idx + 1;

    prev_x = point_buf.first;
    prev_y = point_buf.second;
  }

  // std::cout << idx << std::endl;
  // std::cout << array_in.size() << std::endl;

  distance = min_distance * sign;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
GeoFenceLoader::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                           double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}
