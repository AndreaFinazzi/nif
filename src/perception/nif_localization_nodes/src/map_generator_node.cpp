// inlcude ROS library
#include "novatel_oem7_msgs/msg/inspva.hpp"
#include "rclcpp/clock.hpp"
#include <chrono>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// inlcude PCL library
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
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
#include <pcl_ros/transforms.hpp>

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_frame_id/frame_id.h"
#include "nif_localization_minimal/localization_minimal.h"

class MapGenerator : public rclcpp::Node {
public:
  MapGenerator();
  ~MapGenerator();

  void
  SubkeyFramePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void
  SurfaceFeatureCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void
  VoxelizedPointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  void INSPVAOdometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void MapSaverCallback(const std_msgs::msg::Bool::SharedPtr msg);

  void mapping_timer_callback();

private:
  void respond();
  void run();


  pcl::PointCloud<pcl::PointXYZI>::Ptr
  downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution);

  rclcpp::TimerBase::SharedPtr mapping_timer_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      subSubkeyFramePoints;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      subVoxelizedPoints;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subINSPVAOdometry;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subSaveMap;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFinalMapCloud;

  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> subKeyFrameQue;

  int count;
  int m_SizeOfSubKeyFrame;
  double m_score;
  Eigen::Matrix4f prev_trans;
  pcl::PointCloud<pcl::PointXYZI>::Ptr SubKeyFramePoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr VoxelizedPoints;
  pcl::PointCloud<pcl::PointXYZI>::Ptr FinalMapCloud;

  bool bSubKeyFramePoints;
  bool bPrevSubKeyFramePoints;
  bool bRegisteredSubKeyFramePoints;
  bool bVoxelizedPoints;
  bool bFirstCall;
  bool bFinalMapInit;

  std_msgs::msg::Header m_header;
  double m_VoxelSize;
  bool bFirstMatched;
  nav_msgs::msg::Odometry m_Odometry;
};

MapGenerator::MapGenerator() : Node("scan_matching") {
  this->declare_parameter<double>("voxel_size", double(3.0));

  // this->declare_parameter<std::string>("output_frame_id",
  // std::string("center_of_gravity")); std::chrono::microseconds
  // SYNC_PERIOD_TMP(1000);
  using namespace std::chrono_literals; // NOLINT
  mapping_timer_ = this->create_wall_timer(
      50ms, std::bind(&MapGenerator::mapping_timer_callback, this));
  // "/ransac_filtered_points/both"
  subSubkeyFramePoints =
      this->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/inverse_mapped_points", nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&MapGenerator::SubkeyFramePointsCallback, this,
                    std::placeholders::_1));
  subVoxelizedPoints = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/merged/ego_filtered", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&MapGenerator::VoxelizedPointsCallback, this,
                std::placeholders::_1));
  subINSPVAOdometry = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aw_localization/ekf/odom", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&MapGenerator::INSPVAOdometryCallback, this,
                std::placeholders::_1));
  subSaveMap = this->create_subscription<std_msgs::msg::Bool>(
      "/map_saver", nif::common::constants::QOS_EGO_ODOMETRY,
      std::bind(&MapGenerator::MapSaverCallback, this, std::placeholders::_1));

  pubFinalMapCloud =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap_generated", nif::common::constants::QOS_EGO_ODOMETRY);

  respond();
  // make_transform_list();

  // std::cout << "input_topics : " << input_topics_ << std::endl;
  // std::cout << "output_topic : " << output_topic_ << std::endl;
  // std::cout << "output_frame_id : "<< output_frame_id_ << std::endl;
}

MapGenerator::~MapGenerator() {}

void MapGenerator::respond() { this->get_parameter("voxel_size", m_VoxelSize); }

// mapping mode
void MapGenerator::mapping_timer_callback() {
  if (bVoxelizedPoints && bSubKeyFramePoints && bFinalMapInit) {



    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudToBeStack(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*SubKeyFramePoints, *cloudToBeStack, prev_trans);

    *FinalMapCloud += *cloudToBeStack;

    FinalMapCloud = downsample(FinalMapCloud, m_VoxelSize);

    sensor_msgs::msg::PointCloud2 FinalMapCloudMsg;
    pcl::toROSMsg(*FinalMapCloud, FinalMapCloudMsg);
    FinalMapCloudMsg.header.frame_id = nif::common::frame_id::localization::ODOM;
    FinalMapCloudMsg.header.stamp = m_header.stamp;
    pubFinalMapCloud->publish(FinalMapCloudMsg);
  }
}

void MapGenerator::MapSaverCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  pcl::io::savePCDFileASCII("map.pcd", *FinalMapCloud);
  std::cout << "save map" << std::endl;
  // std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd."
  // << std::endl;
}

void MapGenerator::SubkeyFramePointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  SubKeyFramePoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIn(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloudIn);
  pcl::PointIndices::Ptr outlier_indices(new pcl::PointIndices);
  for (unsigned int i = 0; i < cloudIn->points.size(); i++) {
    pcl::PointXYZI points_raw;
    points_raw.x = cloudIn->points[i].x;
    points_raw.y = cloudIn->points[i].y;
    points_raw.z = cloudIn->points[i].z;

    double distance2point = sqrt(pow(points_raw.x, 2) + pow(points_raw.y, 2));
    if (distance2point > 50 || points_raw.x < 0.0) {
      continue;
    }

    outlier_indices->indices.push_back(i);
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloudIn);
  extract.setIndices(outlier_indices);
  extract.setNegative(
      false); // true removes the indices, false leaves only the indices
  extract.filter(*SubKeyFramePoints);

  bSubKeyFramePoints = true;

  if (!bFirstCall) {
    bFirstCall = true;
    FinalMapCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    bFinalMapInit = true;
  }
}


void MapGenerator::VoxelizedPointsCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  VoxelizedPoints.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *VoxelizedPoints);
  bVoxelizedPoints = true;
}

void MapGenerator::INSPVAOdometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  m_Odometry = *msg;
  // if(!bFirstMatched)
  // {
  Eigen::Matrix3f mat3 = Eigen::Quaternionf(msg->pose.pose.orientation.w,
                                            msg->pose.pose.orientation.x,
                                            msg->pose.pose.orientation.y,
                                            msg->pose.pose.orientation.z)
                             .toRotationMatrix();
  prev_trans.block(0, 0, 3, 3) = mat3;
  prev_trans(0, 3) = msg->pose.pose.position.x;
  prev_trans(1, 3) = msg->pose.pose.position.y;
  // }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
MapGenerator::downsample(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                         double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MapGenerator>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}