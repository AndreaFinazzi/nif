//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/22/21.
//

#include "nif_utils/utils.h"
#include <math.h>

/**
 * Time in seconds
 **/
double nif::common::utils::time::secs(rclcpp::Time t) {
  return static_cast<double>(t.nanoseconds()) * 1e-9;
}

/**
 * Duration in seconds
 **/
double nif::common::utils::time::secs(rclcpp::Duration t) {
  return static_cast<double>(t.nanoseconds()) * 1e-9;
}

double nif::common::utils::geometry::calEuclideanDistance(
    const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
}

double nif::common::utils::geometry::calEuclideanDistance(
    const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b) {
  return calEuclideanDistance(a.position, b.position);
}

double nif::common::utils::geometry::calEuclideanDistance(
    const geometry_msgs::msg::PoseStamped& a,
    const geometry_msgs::msg::PoseStamped& b) {
  return calEuclideanDistance(a.pose, b.pose);
}

double nif::common::utils::geometry::mph2kph(const double mph) {
  return MPH_TO_KPH_FACTOR * mph;
}

double nif::common::utils::geometry::kph2mph(const double kph) {
  return KPH_TO_MPH_FACTOR * kph;
}

double nif::common::utils::geometry::mph2mps(const double mph) {
  return 0.44704 * mph;
}

inline double nif::common::utils::coordination::quat2yaw(
    const geometry_msgs::msg::Quaternion& data) {
  return atan2(2 * (data.w * data.z + data.x * data.y),
               1 - 2 * (data.y * data.y + data.z * data.z));
}

inline geometry_msgs::msg::Quaternion nif::common::utils::coordination::eular2quat(
  double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
{
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

inline double nif::common::utils::coordination::angle_wrap(double diff) {
  diff = fmod(diff + nif::common::constants::numeric::PI,
              2 * nif::common::constants::numeric::PI);
  if (diff < 0)
    diff += 2 * nif::common::constants::numeric::PI;
  diff -= nif::common::constants::numeric::PI;
  return diff;
}

// ================================ TO GLOBAL ================================
geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtBodytoGlobal(
    const nav_msgs::msg::Odometry& current_odom_,
    const geometry_msgs::msg::PoseStamped& point_in_body_) 
{
      geometry_msgs::msg::PoseStamped point_in_global_stamped{};

      point_in_global_stamped = getPtBodytoGlobal(current_odom_, point_in_body_.pose);
      point_in_global_stamped.header.stamp  = point_in_body_.header.stamp;

      return point_in_global_stamped;

}

geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtBodytoGlobal(
    const nav_msgs::msg::Odometry& current_odom_,
    const geometry_msgs::msg::Pose& point_in_body_) 
{
      geometry_msgs::msg::PoseStamped point_in_global_stamped{};

      point_in_global_stamped.pose    = getPtBodytoGlobal(current_odom_.pose.pose, point_in_body_);
      point_in_global_stamped.header.frame_id  = current_odom_.header.frame_id;

      return point_in_global_stamped;
}

geometry_msgs::msg::Pose
nif::common::utils::coordination::getPtBodytoGlobal(
    const geometry_msgs::msg::Pose& current_pose_,
    const geometry_msgs::msg::Pose& point_in_body_) 
{
  tf2::Vector3 tf_body_in_global_origin(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
  tf2::Quaternion tf_body_in_global_quat(current_pose_.orientation.x, current_pose_.orientation.y, current_pose_.orientation.z, current_pose_.orientation.w);
  tf2::Transform tf_body_in_global(
    tf_body_in_global_quat,
    tf_body_in_global_origin
  );
      

  tf2::Vector3 tf_point_in_body_origin(point_in_body_.position.x, point_in_body_.position.y, point_in_body_.position.z);
  tf2::Quaternion tf_point_in_body_quat(point_in_body_.orientation.x, point_in_body_.orientation.y, point_in_body_.orientation.z, point_in_body_.orientation.w);
  tf2::Transform tf_point_in_body(
    tf_point_in_body_quat,
    tf_point_in_body_origin
  );

  tf2::Transform tf_point_in_global = tf_body_in_global * tf_point_in_body;

  geometry_msgs::msg::Pose point_in_global{};

  point_in_global.position.x = tf_point_in_global.getOrigin().x();
  point_in_global.position.y = tf_point_in_global.getOrigin().y();
  point_in_global.position.z = tf_point_in_global.getOrigin().z();

  point_in_global.orientation.x = tf_point_in_global.getRotation().x();
  point_in_global.orientation.y = tf_point_in_global.getRotation().y();
  point_in_global.orientation.z = tf_point_in_global.getRotation().z();
  point_in_global.orientation.w = tf_point_in_global.getRotation().w();

  return point_in_global;
}

// ================================ TO BODY ================================


geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtGlobaltoBody(
    const nav_msgs::msg::Odometry& current_odom_,
    const geometry_msgs::msg::PoseStamped& point_in_global_) 
{
  geometry_msgs::msg::PoseStamped point_in_body_stamped{};

  point_in_body_stamped = getPtGlobaltoBody(current_odom_, point_in_global_.pose);
  point_in_body_stamped.header.stamp = point_in_global_.header.stamp;

  return point_in_body_stamped;
}


geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtGlobaltoBody(
    const nav_msgs::msg::Odometry& current_odom_,
    const geometry_msgs::msg::Pose& point_in_global_) 
{
  geometry_msgs::msg::PoseStamped point_in_body_stamped{};

  point_in_body_stamped.pose = getPtGlobaltoBody(current_odom_.pose.pose, point_in_global_);

  return point_in_body_stamped;
}

__attribute_deprecated__
geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtGlobaltoBody(
    const nav_msgs::msg::Odometry& current_odom_,
    const double& global_x_,
    const double& global_y_) 
{
  geometry_msgs::msg::Pose pose_in_global{};
  pose_in_global.position.x = global_x_;
  pose_in_global.position.y = global_y_;

  geometry_msgs::msg::PoseStamped point_in_body_stamped{};

  point_in_body_stamped.pose = getPtGlobaltoBody(current_odom_.pose.pose, pose_in_global);

  return point_in_body_stamped;
}

geometry_msgs::msg::Pose
nif::common::utils::coordination::getPtGlobaltoBody(
  const geometry_msgs::msg::Pose& current_pose_,
  const geometry_msgs::msg::Pose& point_in_global_)
{
  tf2::Vector3 tf_body_in_global_origin(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
  tf2::Quaternion tf_body_in_global_quat(current_pose_.orientation.x, current_pose_.orientation.y, current_pose_.orientation.z, current_pose_.orientation.w);
  tf2::Transform tf_body_in_global(
    tf_body_in_global_quat,
    tf_body_in_global_origin
  );
      
  tf2::Vector3 tf_point_in_global_origin(point_in_global_.position.x, point_in_global_.position.y, point_in_global_.position.z);
  tf2::Quaternion tf_point_in_global_quat(point_in_global_.orientation.x, point_in_global_.orientation.y, point_in_global_.orientation.z, point_in_global_.orientation.w);
  tf2::Transform tf_point_in_global(
    tf_point_in_global_quat,
    tf_point_in_global_origin
  );

  tf2::Transform tf_point_in_body = tf_body_in_global.inverse() * tf_point_in_global;

  geometry_msgs::msg::Pose point_in_body{};

  point_in_body.position.x = tf_point_in_body.getOrigin().x();
  point_in_body.position.y = tf_point_in_body.getOrigin().y();
  point_in_body.position.z = tf_point_in_body.getOrigin().z();

  point_in_body.orientation.x = tf_point_in_body.getRotation().x();
  point_in_body.orientation.y = tf_point_in_body.getRotation().y();
  point_in_body.orientation.z = tf_point_in_body.getRotation().z();
  point_in_body.orientation.w = tf_point_in_body.getRotation().w();

  return point_in_body;
}


nav_msgs::msg::Path nif::common::utils::coordination::getPathGlobaltoBody(
    const nav_msgs::msg::Odometry& current_pose_,
    const nav_msgs::msg::Path& path_in_global_) 
{
  nav_msgs::msg::Path path_in_body;
 
  for (int i = 0; i < path_in_global_.poses.size(); i++) {
    geometry_msgs::msg::PoseStamped point_in_body = getPtGlobaltoBody(
      current_pose_,
      path_in_global_.poses[i]
    );
    path_in_body.poses.push_back(point_in_body);
  }

  return path_in_body;
}

