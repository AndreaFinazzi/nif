//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/22/21.
//

#ifndef NIF_COMMON_NODES_UTILS_GEOMETRY_H
#define NIF_COMMON_NODES_UTILS_GEOMETRY_H

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nif_common/constants.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/parameter.hpp>
#include <string>

namespace nif {
namespace common {
namespace utils {

namespace time {
  /**
   * Time in seconds
  **/
  double secs(rclcpp::Time t);
  
  /**
   * Duration in seconds
  **/
  double secs(rclcpp::Duration t);
}

namespace geometry {

const float MPH_TO_KPH_FACTOR = 1.60934;
const float KPH_TO_MPH_FACTOR = 0.621371;

/**
 * Calculate Euclidean distance between Point/Pose/PoseStamped
 * @param a Point/Pose/PoseStamped
 * @param b Point/Pose/PoseStamped
 * @return Euclidean distance between Point/Pose/PoseStamped
 */
double calEuclideanDistance(const geometry_msgs::msg::PoseStamped& a,
                            const geometry_msgs::msg::PoseStamped& b);

/**
 * Calculate Euclidean distance between Point/Pose/PoseStamped
 * @param a Point/Pose/PoseStamped
 * @param b Point/Pose/PoseStamped
 * @return Euclidean distance between Point/Pose/PoseStamped
 */
double calEuclideanDistance(const geometry_msgs::msg::Pose& a,
                            const geometry_msgs::msg::Pose& b);

/**
 * Calculate Euclidean distance between Point/Pose/PoseStamped
 * @param a Point/Pose/PoseStamped
 * @param b Point/Pose/PoseStamped
 * @return Euclidean distance between Point/Pose/PoseStamped
 */
double calEuclideanDistance(const geometry_msgs::msg::Point& a,
                            const geometry_msgs::msg::Point& b);

//    TODO: fix static override problems
//    double calEuclideanDistance(const std::vector<double,double>& a, const
//    std::vector<double,double>& b);

/**
 * Convert linear velocity from miles-per-hour to kilometers-per-hour
 * @param mph Linear velocity in miles-per-hour
 * @return Linear velocity in kilometers-per-hour
 */
double mph2kph(double mph);

/**
 * Convert linear velocity from kilometers-per-hour to miles-per-hour
 * @param mph Linear velocity in kilometers-per-hour
 * @return Linear velocity in miles-per-hour
 */
double kph2mph(double kph);

} // namespace geometry

namespace io {
//  TODO: define precisely (do we need this?)
std::vector<rclcpp::Parameter>& readConfig(std::string& file_name);

} // namespace io

// TODO: numeric is an awful name, needs something better.
namespace numeric {

//    TODO: provide exaustive description of these two functions, along wth teir
//    params' description.
//     std::tuple<min_value, min_value_idx>
//    findMinValueNIdx(std::vector<double>& vec); std::tuple<max_value,
//    max_value_idx> findMaxValueNIdx(std::vector<double>& vec);

/**
 * Clip the target value to min-max bounds.
 * @tparam T datatype must implement comparison operators
 * @param min Minimum value.
 * @param max Maximum value.
 * @param target Target value to be clipped.
 * @return Clipped value.
 */
template <typename T>
constexpr inline const T& clip(const T& min, const T& max, const T& target) {
  return std::max<T>(min, std::min<T>(target, max));
}

} // namespace numeric

namespace coordination {

inline double quat2yaw(geometry_msgs::msg::Quaternion& data);

inline double angle_wrap(double diff);

geometry_msgs::msg::PoseStamped
getPtBodytoGlobal(nav_msgs::msg::Odometry& current_pose_,
                  geometry_msgs::msg::PoseStamped& point_in_body_);
geometry_msgs::msg::PoseStamped
getPtGlobaltoBody(nav_msgs::msg::Odometry& current_pose_,
                  geometry_msgs::msg::PoseStamped& point_in_global_);

} // namespace coordination

namespace naming {

inline const std::string getGlobalParamName( const std::string & param_name )
{
  return 
    nif::common::constants::parameters::DELIMITER + 
    nif::common::constants::parameters::GLOBAL_PARAMETERS_NODE_NAME + 
    nif::common::constants::parameters::DELIMITER + 
    param_name;
}

} // namespace naming

} // namespace utils
} // namespace common
} // namespace nif

#endif // NIF_COMMON_NODES_UTILS_GEOMETRY_H
