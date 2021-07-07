//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/22/21.
//

#include "nif_utils/utils.h"
#include <cmath>

double nif::common::utils::geometry::calEuclideanDistance(
    const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
  return sqrt(pow((a.x - b.x), 2) + pow((a.y - b.y), 2) + pow((a.z - b.z), 2));
}

double nif::common::utils::geometry::calEuclideanDistance(
    const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b) {
  return calEuclideanDistance(a.position, b.position);
}

double nif::common::utils::geometry::calEuclideanDistance(
    const geometry_msgs::msg::PoseStamped &a,
    const geometry_msgs::msg::PoseStamped &b) {
  return calEuclideanDistance(a.pose, b.pose);
}

double nif::common::utils::geometry::mph2kph(const double mph) {
  return MPH_TO_KPH_FACTOR * mph;
}

double nif::common::utils::geometry::kph2mph(const double kph) {
  return KPH_TO_MPH_FACTOR * kph;
}