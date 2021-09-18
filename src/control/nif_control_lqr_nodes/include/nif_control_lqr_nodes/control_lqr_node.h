//
// Created by usrg on 8/22/21.
//

#ifndef ROS2MASTER_CONTROL_LQR_NODE_H
#define ROS2MASTER_CONTROL_LQR_NODE_H

/**
 * @brief path following node
 **/
#include "nif_control_common_nodes/i_controller_node.h"
#include <deep_orange_msgs/msg/joystick_command.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"

#include "nif_control_lqr_nodes/lqr/lateral_lqr.h"
#include "nif_control_lqr_nodes/utils/lateral_lqr_ros.h"
#include "nif_control_lqr_nodes/utils/pure_pursuit_tracker.h"


namespace nif {
namespace control {

class ControlLQRNode : public nif::control::IControllerNode {
public:
  explicit ControlLQRNode(const std::string &node_name);

  void
  publishSteeringDiagnostics(bool lqr_command_valid,
                             double lqr_steering_command, double track_distance,
                             geometry_msgs::msg::PoseStamped lqr_track_point,
                             bvs_control::lqr::LateralLQR::ErrorMatrix lqr_err);

  /** ROS Callbacks / Subscription Interface **/
  void afterReferencePathCallback() override {
    lqr_tracking_idx_ = 0; // Reset Tracking
  }

  void velocityCallback(
      const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
    current_speed_ms_ = (msg->rear_left + msg->rear_right) * 0.5 * nif::common::constants::KPH2MS;
  }

  void joystickCallback(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg);

private:
  //! Debug Interface
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lqr_command_valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
  lqr_steering_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr track_distance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
  lqr_tracking_point_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lqr_error_pub_;
  //! Command Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_command_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gear_command_pub_;
  //! Input Data
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr velocity_sub_;
  rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr joystick_sub_;
  rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr pt_report_sub_;

  //! Lateral LQR Controller
  bvs_control::lqr::LateralLQR::Ptr lateral_lqr_;

  //! Control command published by solve()
  nif::common::msgs::ControlCmd::SharedPtr control_cmd;

  //! Current Vehicle State
  double current_speed_ms_;
  uint8_t current_gear_;
  uint8_t current_engine_speed_;
  bool engine_running_;
  double last_steering_command_;

  //! LQR Tracking State
  unsigned int lqr_tracking_idx_;

  //! Manual Overrides for when auto mode is disabled
  double override_steering_target_;
  double override_brake_target_;
  double override_throttle_target_;
  uint8_t override_gear_target_;

  //! Misc. Parameters (see notes in constructor)
  double max_steering_angle_deg_;
  double steering_auto_override_deg_;
  double steering_units_multiplier_;
  double pure_pursuit_min_dist_m_;
  double pure_pursuit_max_dist_m_;
  double pure_pursuit_k_vel_m_ms_;
  bool use_tire_velocity_;
  double odometry_timeout_sec_;
  double path_timeout_sec_;
  double steering_max_ddeg_dt_;

  double secs(rclcpp::Time t) {
    return static_cast<double>(t.seconds()) +
    static_cast<double>(t.nanoseconds()) * 1e-9;
  }
  double secs(rclcpp::Duration t) {
    return static_cast<double>(t.seconds()) +
    static_cast<double>(t.nanoseconds()) * 1e-9;
  }

  nif::common::msgs::ControlCmd::SharedPtr solve() override;
}; /* class PathFollowerNode */

} // namespace nif
} // namespace control


#endif // ROS2MASTER_CONTROL_LQR_NODE_H
