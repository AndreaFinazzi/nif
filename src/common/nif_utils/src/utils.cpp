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
    geometry_msgs::msg::Quaternion& data) {
  return atan2(2 * (data.w * data.z + data.x * data.y),
               1 - 2 * (data.y * data.y + data.z * data.z));
}

inline double nif::common::utils::coordination::angle_wrap(double diff) {
  diff = fmod(diff + nif::common::constants::numeric::PI,
              2 * nif::common::constants::numeric::PI);
  if (diff < 0)
    diff += 2 * nif::common::constants::numeric::PI;
  diff -= nif::common::constants::numeric::PI;
  return diff;
}

geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtBodytoGlobal(
    nav_msgs::msg::Odometry& current_pose_,
    geometry_msgs::msg::PoseStamped& point_in_body_) {
  double current_yaw_rad = nif::common::utils::coordination::quat2yaw(
      current_pose_.pose.pose.orientation);
  geometry_msgs::msg::PoseStamped point_in_global;
  point_in_global.pose.position.x = current_pose_.pose.pose.position.x +
      point_in_body_.pose.position.x * cos(current_yaw_rad) -
      point_in_body_.pose.position.y * sin(current_yaw_rad);
  point_in_global.pose.position.y = current_pose_.pose.pose.position.y +
      point_in_body_.pose.position.x * sin(current_yaw_rad) +
      point_in_body_.pose.position.y * cos(current_yaw_rad);
  point_in_global.pose.position.z =
      current_pose_.pose.pose.position.y + point_in_body_.pose.position.z;
  return point_in_global;
}

geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtGlobaltoBody(
    nav_msgs::msg::Odometry& current_pose_,
    geometry_msgs::msg::PoseStamped& point_in_global_) {
  double current_yaw_rad = nif::common::utils::coordination::quat2yaw(
      current_pose_.pose.pose.orientation);
  geometry_msgs::msg::PoseStamped point_in_body;
  point_in_body.pose.position.x = cos(-1 * current_yaw_rad) *
          (point_in_global_.pose.position.x -
           current_pose_.pose.pose.position.x) -
      sin(-1 * current_yaw_rad) *
          (point_in_global_.pose.position.y -
           current_pose_.pose.pose.position.y);
  point_in_body.pose.position.y = sin(-1 * current_yaw_rad) *
          (point_in_global_.pose.position.x -
           current_pose_.pose.pose.position.x) +
      cos(-1 * current_yaw_rad) *
          (point_in_global_.pose.position.y -
           current_pose_.pose.pose.position.y);
  point_in_body.pose.position.z = 0.0;

  point_in_body.pose.orientation.x = 0.0;
  point_in_body.pose.orientation.y = 0.0;
  point_in_body.pose.orientation.z = 0.0;
  point_in_body.pose.orientation.w = 1.0;
  return point_in_body;
}

geometry_msgs::msg::PoseStamped
nif::common::utils::coordination::getPtGlobaltoBody(
    nav_msgs::msg::Odometry& current_pose_,
    double& global_x_,
    double& global_y_) {
  double current_yaw_rad = nif::common::utils::coordination::quat2yaw(
      current_pose_.pose.pose.orientation);
  geometry_msgs::msg::PoseStamped point_in_body;
  point_in_body.pose.position.x = cos(-1 * current_yaw_rad) *
          (global_x_ - current_pose_.pose.pose.position.x) -
      sin(-1 * current_yaw_rad) *
          (global_y_ - current_pose_.pose.pose.position.y);
  point_in_body.pose.position.y = sin(-1 * current_yaw_rad) *
          (global_x_ - current_pose_.pose.pose.position.x) +
      cos(-1 * current_yaw_rad) *
          (global_y_ - current_pose_.pose.pose.position.y);
  point_in_body.pose.position.z =
      point_in_global_.pose.position.z - current_pose_.pose.pose.position.z;

  double target_yaw = nif::common::utils::coordination::quat2yaw(
      point_in_global_.pose.orientation);

  point_in_body.pose.orientation.x = 0.0;
  point_in_body.pose.orientation.y = 0.0;
  // TODO : should be tested
  point_in_body.pose.orientation.z = sin((target_yaw - current_yaw_rad) / 2.0);
  point_in_body.pose.orientation.w = cos((target_yaw - current_yaw_rad) / 2.0);
  return point_in_body;
}

nav_msgs::msg::Path nif::common::utils::coordination::getPathGlobaltoBody(
    nav_msgs::msg::Odometry& current_pose_,
    nav_msgs::msg::Path& path_in_global_) {
  double current_yaw_rad = nif::common::utils::coordination::quat2yaw(
      current_pose_.pose.pose.orientation);

  nav_msgs::msg::Path path_in_body;
  for (int i = 0; i < path_in_global_.poses.size(); i++) {
    geometry_msgs::msg::PoseStamped point_in_body;
    point_in_body.pose.position.x = cos(-1 * current_yaw_rad) *
            (path_in_global_.poses[i].pose.position.x -
             current_pose_.pose.pose.position.x) -
        sin(-1 * current_yaw_rad) *
            (path_in_global_.poses[i].pose.position.y -
             current_pose_.pose.pose.position.y);
    point_in_body.pose.position.y = sin(-1 * current_yaw_rad) *
            (path_in_global_.poses[i].pose.position.x -
             current_pose_.pose.pose.position.x) +
        cos(-1 * current_yaw_rad) *
            (path_in_global_.poses[i].pose.position.y -
             current_pose_.pose.pose.position.y);
    point_in_body.pose.position.z = path_in_global_.poses[i].pose.position.z -
        current_pose_.pose.pose.position.z;

    double target_yaw = nif::common::utils::coordination::quat2yaw(
        path_in_global_.poses[i].pose.orientation);

    point_in_body.pose.orientation.x = 0.0;
    point_in_body.pose.orientation.y = 0.0;
    point_in_body.pose.orientation.z =
        sin((target_yaw - current_yaw_rad) / 2.0);
    point_in_body.pose.orientation.w =
        cos((target_yaw - current_yaw_rad) / 2.0);
    path_in_body.poses.push_back(point_in_body);
  }

  return path_in_body;
}