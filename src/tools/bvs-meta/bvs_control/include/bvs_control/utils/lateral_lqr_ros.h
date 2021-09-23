/**
 * @brief utility functions to convert from ROS types to
 *  Eigen matrices for lateral_lqr
 **/
#ifndef BVS_CONTROL_UTILS_LATERAL_LQR_ROS_H_
#define BVS_CONTROL_UTILS_LATERAL_LQR_ROS_H_

#include "bvs_control/lqr/lateral_lqr.h"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace bvs_control {
namespace utils {

/**
 * @brief converts from odometry to LQR State
 **/
lqr::LateralLQR::StateMatrix
LQRState(nav_msgs::msg::Odometry odom);

/**
 * @brief converts from odometry to LQR Goal
 **/
lqr::LateralLQR::GoalMatrix
LQRGoal(nav_msgs::msg::Odometry odom);

/**
 * @brief converts from pose stamped to LQR Goal
 **/
lqr::LateralLQR::GoalMatrix
LQRGoal(geometry_msgs::msg::PoseStamped pose);

/**
 * @brief converts from LQR Error matrix to float32 multiarray
 **/
std_msgs::msg::Float32MultiArray
ROSError(lqr::LateralLQR::ErrorMatrix error);



} /* namespace utils */
} /* namespace bvs_control */

#endif