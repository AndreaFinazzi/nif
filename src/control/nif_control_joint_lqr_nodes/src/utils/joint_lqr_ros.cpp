#include "nif_control_joint_lqr_nodes/utils/joint_lqr_ros.h"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace joint_lqr {
namespace utils {

lqr::JointLQR::StateMatrix LQRState(const nav_msgs::msg::Odometry &odom) {
  lqr::JointLQR::StateMatrix result;
  //! These should just be in the correct frame
  result(0, 0) = odom.pose.pose.position.x;
  result(1, 0) = odom.pose.pose.position.y;
  result(2, 0) = odom.twist.twist.linear.x;
  result(3, 0) = odom.twist.twist.linear.y;

  //! Handle yaw
  tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  result(4, 0) = yaw;
  // NOTE: This is probably not being estimated
  result(5, 0) = odom.twist.twist.angular.z;
  return result;
}

lqr::JointLQR::GoalMatrix LQRGoal(const nav_msgs::msg::Odometry &odom,
                                  double desired_velocity) {
  lqr::JointLQR::GoalMatrix result;
  //! These should just be in the correct frame
  result(0, 0) = odom.pose.pose.position.x;
  result(1, 0) = odom.pose.pose.position.y;

  //! Handle yaw
  tf2::Quaternion q(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
                    odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  result(2, 0) = yaw;
  result(3, 0) = desired_velocity;
  return result;
}

lqr::JointLQR::GoalMatrix LQRGoal(const geometry_msgs::msg::PoseStamped &pose,
                                  double desired_velocity) {
  lqr::JointLQR::GoalMatrix result;
  //! These should just be in the correct frame
  result(0, 0) = pose.pose.position.x;
  result(1, 0) = pose.pose.position.y;

  //! Handle yaw
  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y,
                    pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  result(2, 0) = yaw;
  result(3, 0) = desired_velocity;
  return result;
}

std_msgs::msg::Float32MultiArray
ROSError(const lqr::JointLQR::ErrorMatrix &error) {
  std_msgs::msg::Float32MultiArray result;
  result.data.push_back(error(0, 0));
  result.data.push_back(error(1, 0));
  result.data.push_back(error(2, 0));
  result.data.push_back(error(3, 0));
  result.data.push_back(error(4, 0)); // add velocity error
  return result;
}

} /* namespace utils */
} /* namespace joint_lqr */