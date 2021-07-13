//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/22/21.
//

#include "nif_utils/utils.h"
#include <math.h>

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
