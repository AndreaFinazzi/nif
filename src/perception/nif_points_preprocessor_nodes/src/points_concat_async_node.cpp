/*
 * points_concat_async_node.cpp
 *
 *  Created on: Oct 8, 2021
 *      Author: Daegyu Lee
 */

#include "nif_points_preprocessor_nodes/points_concat_async_node.h"
#include "nif_frame_id/frame_id.h"

using namespace nif::perception;
using namespace nif::common::frame_id::localization;

PointsConcatAsyncFilterNode::PointsConcatAsyncFilterNode(const std::string &node_name_)
    : Node(node_name_) {

  make_transform_list();

  RCLCPP_INFO(this->get_logger(), "input_topics : '%s'", input_topics_.c_str());
  RCLCPP_INFO(this->get_logger(), "output_topic : '%s'", output_topic_.c_str());

  sub_points_front_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/luminar_front_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&PointsConcatAsyncFilterNode::FrontCallback, this,
                std::placeholders::_1));
  sub_points_left_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/luminar_left_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&PointsConcatAsyncFilterNode::LeftCallback, this,
                std::placeholders::_1));
  sub_points_right_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/luminar_right_points", nif::common::constants::QOS_SENSOR_DATA,
      std::bind(&PointsConcatAsyncFilterNode::RightCallback, this,
                std::placeholders::_1));

  pub_concat_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/merged/lidar", nif::common::constants::QOS_SENSOR_DATA);

  using namespace std::chrono_literals; // NOLINT
  timer_ = this->create_wall_timer(
      20ms, std::bind(&PointsConcatAsyncFilterNode::timerCallback, this));

  timeout = rclcpp::Duration(1, 0);
}

PointsConcatAsyncFilterNode::~PointsConcatAsyncFilterNode() {}

void PointsConcatAsyncFilterNode::timerCallback() {
  if (!bTransfromListGenerated)
    return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ReceivedFrontPoints(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ReceivedLeftPoints(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ReceivedRightPoints(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr ConcatenatedPoints(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr VoxelfilteredPoints(
      new pcl::PointCloud<pcl::PointXYZI>);

  //front concat
  if (bFront)
  {
    if ((this->now() - front_last_update) <= this->timeout) {
      *ReceivedFrontPoints = *m_front_points;
      transformPointCloudCustom(*ReceivedFrontPoints, *ReceivedFrontPoints,
                                transfrom_list[0]);
      *ConcatenatedPoints += *ReceivedFrontPoints;
    }
  }
  // Left concat
  if (bLeft)
  {
    if ((this->now() - left_last_update) <= this->timeout) {
      *ReceivedLeftPoints = *m_left_points;
      transformPointCloudCustom(*ReceivedLeftPoints, *ReceivedLeftPoints,
                                transfrom_list[1]);
      *ConcatenatedPoints += *ReceivedLeftPoints;
    }
  }
  // Right concat
  if(bRight)
  {
    if ((this->now() - right_last_update) <= this->timeout) {
      *ReceivedRightPoints = *m_right_points;
      transformPointCloudCustom(*ReceivedRightPoints, *ReceivedRightPoints,
                                transfrom_list[2]);
      *ConcatenatedPoints += *ReceivedRightPoints;    
    }
  }

  // publsh points
  sensor_msgs::msg::PointCloud2 MergedCloudMsg;
  // if(!ConcatenatedPoints->points.empty())
  //   ConcatenatedPoints = downsample(ConcatenatedPoints, 0.05);
  pcl::toROSMsg(*ConcatenatedPoints, MergedCloudMsg);

  MergedCloudMsg.header.frame_id =
      nif::common::frame_id::localization::BASE_LINK;
  pub_concat_points_->publish(MergedCloudMsg);
}

void PointsConcatAsyncFilterNode::FrontCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  this->front_last_update = this->now();
  bFront = true;
  m_front_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_front_points);
}
void PointsConcatAsyncFilterNode::LeftCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  this->left_last_update = this->now();
  bLeft = true;
  m_left_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_left_points);
}
void PointsConcatAsyncFilterNode::RightCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  this->right_last_update = this->now();
  bRight = true;
  m_right_points.reset(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*msg, *m_right_points);
}

void PointsConcatAsyncFilterNode::make_transform_list() {
  tf2::Transform transform_to_center_of_gravity;
  transform_to_center_of_gravity.setOrigin(
      tf2::Vector3(-1.3206, 0.030188, -0.23598));
  tf2::Quaternion quat_to_center_of_gravity;
  quat_to_center_of_gravity.setRPY(0.0, 0.0, 0.0);
  transform_to_center_of_gravity.setRotation(quat_to_center_of_gravity);

  // Lidar Rotate
  tf2::Transform transform_lidar_rotate;
  transform_lidar_rotate.setOrigin(tf2::Vector3(0, 0, 0));
  tf2::Quaternion quat_lidar_rotate;
  // quat_lidar_rotate.setRPY(0.0, 0.0, -M_PI / 2); //origin data was
  // rotated but our sensory data was okay
  quat_lidar_rotate.setRPY(0.0, 0.0, 0.0);
  transform_lidar_rotate.setRotation(quat_lidar_rotate);

  //  FRONT LIDAR
  tf2::Transform transform_front;
  transform_front.setOrigin(tf2::Vector3(2.242, 0, 0.448));
  tf2::Quaternion quat_front;
  quat_front.setRPY(0.0, 0.0, 0.0);
  transform_front.setRotation(quat_front);
  transfrom_list.push_back(transform_to_center_of_gravity *
                            transform_front * transform_lidar_rotate);

  //  LEFT LIDAR
  tf2::Transform transform_left;
  transform_left.setOrigin(tf2::Vector3(1.549, 0.267, 0.543));
  tf2::Quaternion quat_left;
  quat_left.setRPY(0.0, 0.0, 2.0943951024);
  transform_left.setRotation(quat_left);
  transfrom_list.push_back(transform_to_center_of_gravity * transform_left *
                            transform_lidar_rotate);

  //  RIGHT LIDAR
  tf2::Transform transform_right;
  transform_right.setOrigin(tf2::Vector3(1.549, -0.267, 0.543));
  tf2::Quaternion quat_right;
  quat_right.setRPY(0.0, 0.0, -2.0943951024);
  transform_right.setRotation(quat_right);
  transfrom_list.push_back(transform_to_center_of_gravity *
                            transform_right * transform_lidar_rotate);

  bTransfromListGenerated = true;
}

void PointsConcatAsyncFilterNode::transformPointCloudCustom(
    const pcl::PointCloud<pcl::PointXYZI> &cloud_in,
    pcl::PointCloud<pcl::PointXYZI> &cloud_out,
    const tf2::Transform &transform) {
  // Bullet (used by tf) and Eigen both store quaternions in x,y,z,w order,
  // despite the ordering of arguments in Eigen's constructor. We could use
  // an Eigen Map to convert without copy, but this only works if Bullet
  // uses floats, that is if BT_USE_DOUBLE_PRECISION is not defined. Rather
  // that risking a mistake, we copy the quaternion, which is a small cost
  // compared to the conversion of the point cloud anyway. Idem for the
  // origin.
  tf2::Quaternion q = transform.getRotation();
  Eigen::Quaternionf rotation(q.w(), q.x(), q.y(),
                              q.z()); // internally stored as (x,y,z,w)
  tf2::Vector3 v = transform.getOrigin();
  Eigen::Vector3f origin(v.x(), v.y(), v.z());
  //    Eigen::Translation3f translation(v);
  // Assemble an Eigen Transform
  // Eigen::Transform3f t;
  // t = translation * rotation;
  transformPointCloud(cloud_in, cloud_out, origin, rotation);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointsConcatAsyncFilterNode::downsample(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, double resolution) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxelgrid;
  voxelgrid.setLeafSize(resolution, resolution, resolution);
  voxelgrid.setInputCloud(cloud);
  voxelgrid.filter(*filtered);
  return filtered;
}