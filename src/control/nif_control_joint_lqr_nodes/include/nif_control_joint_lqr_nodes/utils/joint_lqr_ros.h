/**
 * @brief utility functions to convert from ROS types to
 *  Eigen matrices for joint_lqr
 **/
#ifndef UTILS_JOINT_LQR_ROS_H_
#define UTILS_JOINT_LQR_ROS_H_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nif_control_joint_lqr_nodes/lqr/joint_lqr.h"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace joint_lqr {
namespace utils {

/**
 * @brief converts from odometry to LQR State
 **/
lqr::JointLQR::StateMatrix LQRState(const nav_msgs::msg::Odometry &odom);

/**
 * @brief converts from odometry to LQR Goal
 **/
lqr::JointLQR::GoalMatrix LQRGoal(const nav_msgs::msg::Odometry &odom,
                                  double desired_velocity);

/**
 * @brief converts from pose stamped to LQR Goal
 **/
lqr::JointLQR::GoalMatrix LQRGoal(const geometry_msgs::msg::PoseStamped &pose,
                                  double desired_velocity);

/**
 * @brief converts from LQR Error matrix to float32 multiarray
 **/
std_msgs::msg::Float32MultiArray
ROSError(const lqr::JointLQR::ErrorMatrix &error);

} /* namespace utils */
} /* namespace joint_lqr */

#endif