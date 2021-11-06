#include "nif_control_joint_lqr_nodes/utils/pure_pursuit_tracker.h"

namespace joint_lqr {
namespace utils {

double pursuit_dist(const nav_msgs::msg::Odometry &point_a,
                    const nav_msgs::msg::Odometry &point_b) {
  return std::sqrt(
      std::pow(point_a.pose.pose.position.x - point_b.pose.pose.position.x, 2) +
      std::pow(point_a.pose.pose.position.y - point_b.pose.pose.position.y, 2));
}

double pursuit_dist(const geometry_msgs::msg::PoseStamped &point_a,
                    const nav_msgs::msg::Odometry &point_b) {
  return std::sqrt(
      std::pow(point_a.pose.position.x - point_b.pose.pose.position.x, 2) +
      std::pow(point_a.pose.position.y - point_b.pose.pose.position.y, 2));
}

double pursuit_dist(const geometry_msgs::msg::PoseStamped &point_a,
                    const geometry_msgs::msg::PoseStamped &point_b) {
  return std::sqrt(
      std::pow(point_a.pose.position.x - point_b.pose.position.x, 2) +
      std::pow(point_a.pose.position.y - point_b.pose.position.y, 2));
}

double pursuit_azimuth(const geometry_msgs::msg::PoseStamped &target_point,
                       const nav_msgs::msg::Odometry &ego_point) {
  //
  tf2::Quaternion q(
      ego_point.pose.pose.orientation.x, ego_point.pose.pose.orientation.y,
      ego_point.pose.pose.orientation.z, ego_point.pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  double azimuth =
      std::atan2(target_point.pose.position.y - ego_point.pose.pose.position.y,
                 target_point.pose.position.x -
                     ego_point.pose.pose.position.x) -
      yaw;

  return normalizeTheta(azimuth);
}

double smoothSignal(double current_signal, double target_signal,
                    double delta_dt, double dt) {
  if (target_signal > current_signal &&
      target_signal > current_signal + delta_dt * dt) {
    return current_signal + delta_dt * dt;
  }
  if (target_signal < current_signal &&
      target_signal < current_signal - delta_dt * dt) {
    return current_signal - delta_dt * dt;
  }
  return target_signal;
}

double normalizeTheta(double theta) {
  while (theta >= M_PI) {
    theta -= 2.0 * M_PI;
  }
  while (theta < -M_PI) {
    theta += 2.0 * M_PI;
  }
  return theta;
}

} /* namespace utils */
} /* namespace joint_lqr */