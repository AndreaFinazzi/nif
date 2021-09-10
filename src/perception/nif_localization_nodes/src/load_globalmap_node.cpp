//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Daegyu Lee

//
// Created by usrg on 09/09/21.
//

#include "localization_ekf_nodes/load_globalmap_node.h"
#include "nif_frame_id/frame_id.h"

using namespace nif::localization::maploader;
using namespace nif::common::frame_id::localization;

// Constructor
GlobalmapLoader::GlobalmapLoader(const std::string &node_name)
    : Node(node_name) {
  this->declare_parameter<std::string>("globalmap_file_name", std::string(""));
  this->declare_parameter<std::string>("trajectory_pcd_file", std::string(""));
  this->declare_parameter<bool>("use_trajectory", bool(false));
  this->declare_parameter<double>("voxel_size", double(3.0));

  this->m_glbalmap_file_name = this->get_parameter("globalmap_file_name").as_string();
  this->m_trajectory_file_name = this->get_parameter("trajectory_pcd_file").as_string();
  this->bUseTrajectory = this->get_parameter("use_trajectory").as_bool();
  this->m_voxel_size = this->get_parameter("voxel_size").as_double();

  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
          rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 10));
  qos.best_effort();

  pubGlobalmap =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", nif::common::constants::QOS_PLANNING);
  pubTrajectory = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/pcd_trajectory", nif::common::constants::QOS_PLANNING);

  RCLCPP_INFO(this->get_logger(), "GLOBAL MAP FILE : ", m_glbalmap_file_name);

  pcdFileIO();
  // TrajectorypcdFileIO();

  using namespace std::chrono_literals; // NOLINT
  timer_ = this->create_wall_timer(10000ms, [this]() {
    if (!bMapReady) {
      pcdFileIO();
      // TrajectorypcdFileIO();
    } else {
      // if(bPublishOnce)
      //   return;

      Publisher();
    }
  });
}

GlobalmapLoader::~GlobalmapLoader() {}

void GlobalmapLoader::pcdFileIO() {
  m_globalmap_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr CloudIn(
      new pcl::PointCloud<pcl::PointXYZI>);
  //  read m_globalmap_ptr from a pcd file
  RCLCPP_INFO(this->get_logger(), "LODAING GLOBAL MAP...");

  pcl::io::loadPCDFile(m_glbalmap_file_name, *CloudIn);
  if (CloudIn->size() > 0) {
    RCLCPP_INFO(this->get_logger(), "GLOBAL MAP IS LOADED!");
    RCLCPP_INFO(this->get_logger(), "POINT SIZE: %d", CloudIn->points.size());
  } else {
    RCLCPP_INFO(this->get_logger(), "...");
  }
  std::cout << "m_voxel_size: " << m_voxel_size << std::endl;

  m_globalmap_ptr->header.frame_id = ODOM;
  m_globalmap_ptr = downsample(CloudIn, m_voxel_size);
  bMapReady = true;
}

void GlobalmapLoader::TrajectorypcdFileIO() {
  if (!bUseTrajectory)
    return;

  m_trajectory_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  //  read m_trajectory_ptr from a pcd file
  RCLCPP_INFO(this->get_logger(), "LODAING TRAJECTORY MAP...");
  pcl::io::loadPCDFile(m_trajectory_file_name, *m_trajectory_ptr);
  if (m_trajectory_ptr->size() > 0) {
    RCLCPP_INFO(this->get_logger(), "TRAJECTORY IS LOADED!");
  } else {
    RCLCPP_INFO(this->get_logger(), "...");
  }
  m_trajectory_ptr->header.frame_id = ODOM;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
GlobalmapLoader::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                            double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::VoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}

void GlobalmapLoader::Publisher() {
  sensor_msgs::msg::PointCloud2 GlobalMapCloudMsg;
  pcl::toROSMsg(*m_globalmap_ptr, GlobalMapCloudMsg);
  GlobalMapCloudMsg.header.frame_id = ODOM;
  GlobalMapCloudMsg.header.stamp = this->now();
  pubGlobalmap->publish(GlobalMapCloudMsg);

  // sensor_msgs::msg::PointCloud2 TrajectoryCloudMsg;
  // pcl::toROSMsg(*m_trajectory_ptr, TrajectoryCloudMsg);
  // TrajectoryCloudMsg.header.frame_id = ODOM;
  // TrajectoryCloudMsg.header.stamp = this->now();
  // pubTrajectory->publish(TrajectoryCloudMsg);

  bPublishOnce = true;
}
