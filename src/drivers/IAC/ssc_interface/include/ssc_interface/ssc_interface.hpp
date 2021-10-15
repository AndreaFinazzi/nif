// Copyright 2020 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \copyright Copyright 2020 The Autoware Foundation
/// \file
/// \brief This file defines the ssc_interface class.

#ifndef SSC_INTERFACE__SSC_INTERFACE_HPP_
#define SSC_INTERFACE__SSC_INTERFACE_HPP_

#include <ssc_interface/visibility_control.hpp>

#include <common/types.hpp>
#include <vehicle_interface/platform_interface.hpp>
#include <vehicle_interface/dbw_state_machine.hpp>

#include <automotive_platform_msgs/msg/gear_command.hpp>
#include <automotive_platform_msgs/msg/gear_feedback.hpp>
#include <automotive_platform_msgs/msg/speed_mode.hpp>
#include <automotive_platform_msgs/msg/steering_feedback.hpp>
#include <automotive_platform_msgs/msg/steer_mode.hpp>
#include <automotive_platform_msgs/msg/turn_signal_command.hpp>
#include <automotive_platform_msgs/msg/velocity_accel_cov.hpp>
#include <autoware_auto_msgs/msg/high_level_control_command.hpp>
#include <autoware_auto_msgs/msg/raw_control_command.hpp>
#include <autoware_auto_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_command.hpp>
#include <autoware_auto_msgs/msg/vehicle_state_report.hpp>
#include <autoware_auto_msgs/srv/autonomy_mode_change.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>

///////////////////////////////////////////////////
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_cmd.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/brake_cmd.hpp>
#include <raptor_dbw_msgs/msg/brake_report.hpp>
#include <deep_orange_msgs/msg/brake_temp_report.hpp>
#include <deep_orange_msgs/msg/misc_report.hpp>
#include <deep_orange_msgs/msg/joystick_command.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/gear_cmd.hpp>

/////////////////////////////////////////////////////


#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include "deep_orange_msgs/msg/base_to_car_summary.hpp"

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::TAU;
using std_msgs::msg::Int8;

using automotive_platform_msgs::msg::GearCommand;
using automotive_platform_msgs::msg::GearFeedback;
using automotive_platform_msgs::msg::SpeedMode;
using automotive_platform_msgs::msg::SteeringFeedback;
using automotive_platform_msgs::msg::SteerMode;
using automotive_platform_msgs::msg::TurnSignalCommand;
using automotive_platform_msgs::msg::VelocityAccelCov;
using autoware_auto_msgs::msg::HighLevelControlCommand;
using autoware_auto_msgs::msg::RawControlCommand;
using autoware_auto_msgs::msg::TrajectoryPoint;
using autoware_auto_msgs::msg::VehicleControlCommand;
using autoware_auto_msgs::msg::VehicleKinematicState;
using autoware_auto_msgs::msg::VehicleStateCommand;
using autoware_auto_msgs::srv::AutonomyModeChange;
using ModeChangeRequest = autoware_auto_msgs::srv::AutonomyModeChange_Request;

using autoware::drivers::vehicle_interface::DbwStateMachine;
using autoware::drivers::vehicle_interface::DbwState;
using autoware::drivers::vehicle_interface::CTState;
using autoware::drivers::vehicle_interface::SysState;
using autoware::drivers::vehicle_interface::TrackCondition;
using autoware::drivers::vehicle_interface::VehicleSignal;

/////////////////////////////////////////////////
using MSR = deep_orange_msgs::msg::MiscReport;
using CtReport = deep_orange_msgs::msg::CtReport;
using raptor_dbw_msgs::msg::BrakeCmd;
using raptor_dbw_msgs::msg::SteeringCmd;
// using JoyStickEnabled = autoware_auto_msgs::msg::JoyStickEnabled;
/////////////////////////////////////////////////
namespace ssc_interface
{

static constexpr float32_t STEERING_TO_TIRE_RATIO = 0.533F / 8.6F;

/// \brief Class for interfacing with AS SSC
class SSC_INTERFACE_PUBLIC SscInterface
  : public ::autoware::drivers::vehicle_interface::PlatformInterface
{
public:
  /// \brief Default constructor.
  /// \param[in] node Reference to node
  /// \param[in] front_axle_to_cog Distance from front axle to center-of-gravity in meters
  /// \param[in] rear_axle_to_cog Distance from rear axle to center-of-gravity in meters
  /// \param[in] max_accel_mps2 Maximum acceleration in m/s^2
  /// \param[in] max_decel_mps2 Maximum deceleration in m/s^2
  /// \param[in] max_yaw_rate_rad Maximum rate of change of heading in radians/sec
  /// \param[in] steering_gear_ratio
  /// \param[in] acc_pedal_min_limit
  /// \param[in] acc_pedal_max_limit
  /// \param[in] brake_pressure_min_limit
  /// \param[in] brake_pressure_max_limit
  /// \param[in] steering_min_limit
  /// \param[in] steering_max_limit
  /// \param[in] joy_enabled



  explicit SscInterface(
    rclcpp::Node & node,
    float32_t front_axle_to_cog,
    float32_t rear_axle_to_cog,
    float32_t max_accel_mps2,
    float32_t max_decel_mps2,
    float32_t max_yaw_rate_rad,
    float32_t steering_gear_ratio,
    float32_t acc_pedal_min_limit,
    float32_t acc_pedal_max_limit,
    float32_t brake_pressure_min_limit,
    float32_t brake_pressure_max_limit,
    float32_t steering_min_limit,
    float32_t steering_max_limit, 
    bool8_t joy_enabled
    
  );
  /// \brief Default destructor
  ~SscInterface() noexcept override = default;

  /// \brief Try to receive data from the vehicle platform, and update StateReport and Odometry.
  ///   Exceptions may be thrown on errors
  /// \param[in] timeout The maximum amount of time to check/receive data
  /// \return True if data was received before the timeout, false otherwise
  bool8_t update(std::chrono::nanoseconds timeout) override;
  /// \brief Send the state command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The state command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_state_command(const VehicleStateCommand & msg) override;
  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const HighLevelControlCommand & msg);
  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const RawControlCommand & msg) override;
  /// \brief Send the control command to the vehicle platform.
  ///   Exceptions may be thrown on errors
  /// \param[in] msg The control command to send to the vehicle
  /// \return false if sending failed in some way, true otherwise
  bool8_t send_control_command(const VehicleControlCommand & msg) override;
  /// \brief Handle a request from the user to enable or disable the DBW system.
  ///   Exceptions may be thrown on errors
  /// \param[in] request The requested autonomy mode
  /// \return false only if enabling the DBW system actually failed, true otherwise
  bool8_t handle_mode_change_request(ModeChangeRequest::SharedPtr request) override;

  static void kinematic_bicycle_model(
    float32_t dt, float32_t l_r, float32_t l_f, VehicleKinematicState * vks);

/////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////

private:
  // Publishers (to SSC)
  rclcpp::Publisher<GearCommand>::SharedPtr m_gear_cmd_pub;//Not thos one. This is Autoware's
  rclcpp::Publisher<SpeedMode>::SharedPtr m_speed_cmd_pub;
  rclcpp::Publisher<SteerMode>::SharedPtr m_steer_cmd_pub;
  rclcpp::Publisher<TurnSignalCommand>::SharedPtr m_turn_signal_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_command;

///////////////////////////////////////////////////////////

  rclcpp::Publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>::SharedPtr m_accelerator_pedal_pub;
  rclcpp::Publisher<raptor_dbw_msgs::msg::BrakeCmd>::SharedPtr m_brake_pressure_pub;
  rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr m_steering_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_gear_pub;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr m_gear_shift_cmd;
  rclcpp::Publisher<CtReport>::SharedPtr m_ct_report_pub;
  

///////////////////////////////////////////////////////////
  // Publishers (to Autoware)
  rclcpp::Publisher<VehicleKinematicState>::SharedPtr m_kinematic_state_pub;

  // Subscribers (from SSC)
  rclcpp::SubscriptionBase::SharedPtr m_dbw_state_sub, m_gear_feedback_sub, m_vel_accel_sub,
  m_steer_sub, m_rc_to_ct_sub, m_flag_summary_sub;
  
///////////////////////////////////////////////////////////
  rclcpp::SubscriptionBase::SharedPtr m_misc_report_sub, m_pt_report_sub, m_accelerator_report_sub,
    m_brake_report_sub, m_steer_report_sub, m_vehicle_kinematic_state_sub, 
    m_acc_pedal_demand_sub, m_counter_sub, m_brake_demand_sub, m_gear_demand_sub,m_joystick_app, 
    m_joystick_brake, m_joystick_steer, m_joystick_gear, m_joystick_emergency,m_emergency_joy_bool, m_emergency_hb_bool, 
    m_emergency_confirmation, m_enable_joy_sub, m_diag_hb_sub;
///////////////////////////////////////////////////////////
  rclcpp::Logger m_logger;
  float32_t m_front_axle_to_cog;
  float32_t m_rear_axle_to_cog;
  float32_t m_accel_limit;
  float32_t m_decel_limit;
  float32_t m_max_yaw_rate;
  std::unique_ptr<DbwStateMachine> m_dbw_state_machine;
  bool8_t m_rc_flag_received;
  bool8_t m_flag_summary_received;
  bool8_t emergency_joy_flag;
  bool8_t emergency_hb_flag;
  int emergency_confirmation;
  int enable_joy;
  float32_t vehicle_vel;
  int32_t temp;
  std_msgs::msg::Int32::SharedPtr counter;
  std_msgs::msg::Int32 rolling_counter;
  float32_t m_steering_gear_ratio;
  float32_t m_acc_pedal_min_limit,m_acc_pedal_max_limit,m_brake_pressure_min_limit,m_brake_pressure_max_limit,m_steering_min_limit,m_steering_max_limit;
  bool8_t m_joy_enabled;
  uint8_t m_gear_min = 0;
  uint8_t m_gear_max = 6;
  int8_t flag;
  uint8_t rolling_counter_RC, prev = 0;
  uint8_t counter_acc, counter_brake, counter_steer, counter_gear, counter_ct;

  // The vehicle kinematic state is stored because it needs information from
  // both on_steer_report() and on_vel_accel_report().
  VehicleKinematicState m_vehicle_kinematic_state;
  bool m_seen_steer{false};
  bool m_seen_vel_accel{false};
  // In case both arrive at the same time
  std::mutex m_vehicle_kinematic_state_mutex;

  void on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg);
  void on_gear_report(const GearFeedback::SharedPtr & msg);
  void on_steer_report(const SteeringFeedback::SharedPtr & msg);
  void on_vel_accel_report(const VelocityAccelCov::SharedPtr & msg);
//////////////////////////////////////////////////////////
  void misc_report_callback(const MSR::SharedPtr & msg);
  void pt_report_callback(const deep_orange_msgs::msg::PtReport::SharedPtr & msg);
  void accelerator_report_callback(const raptor_dbw_msgs::msg::AcceleratorPedalReport::SharedPtr & msg);
  void brake_report_callback(const raptor_dbw_msgs::msg::BrakeReport::SharedPtr & msg);
  void steer_report_callback(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr & msg);
  void vehicle_kinematic_state_callback(const VehicleKinematicState::SharedPtr & msg);
  void accelerator_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg);
  void brake_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg);
  // void counter_callback(const std_msgs::msg::Int32::SharedPtr & msg);
  // void counter_checker(const std_msgs::msg::Int32::SharedPtr & msg);
  void gear_cmd_callback(const std_msgs::msg::UInt8::SharedPtr & msg);
  void joystick_accelerator_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg);
  void joystick_brake_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg);
  void joystick_steer_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg);
  void joystick_gear_cmd_callback(const std_msgs::msg::UInt8::SharedPtr & msg);
  void emergency_joy_callback(const std_msgs::msg::Bool::SharedPtr & msg);
  void emergency_hb_callback(const std_msgs::msg::Bool::SharedPtr & msg);
  void emergency_confirmation_callback(const std_msgs::msg::Int8::SharedPtr & msg);
  void enable_joy_callback(const deep_orange_msgs::msg::JoystickCommand::SharedPtr & msg);
  void diagnostics_heartbeat_callback(const std_msgs::msg::Int32::SharedPtr & msg);
  void diagnostic_checker();
  // void joy_stick_callback(const JoyStickEnabled::SharedPtr & msg);
  //void raw_joystick_cmd_sub(const RawControlCommand::SharedPtr & msg);
  void rc_to_ct_callback(const deep_orange_msgs::msg::RcToCt::SharedPtr & msg);
  void flag_summary_callback(const deep_orange_msgs::msg::BaseToCarSummary::SharedPtr & msg);

  SysState raptor_mode;
  TrackCondition m_rec_track_cond; //Recevied track flag
  VehicleSignal m_rec_veh_signal; //Received veh flag
  raptor_dbw_msgs::msg::AcceleratorPedalCmd m_acc_cmd;
  raptor_dbw_msgs::msg::BrakeCmd m_brake_cmd;
  // raptor_dbw_msgs::msg::GearCmd m_gear_cmd;
  std_msgs::msg::UInt8 m_gear_cmd;
  std_msgs::msg::Bool stop_msg;

  CtReport m_ct_report_data;
  int8_t current_gear;
  int8_t gear_shift_status;
  MSR misc_report;
  deep_orange_msgs::msg::RcToCt rc_to_ct_data;
  deep_orange_msgs::msg::BaseToCarSummary flag_summary_data;
  std::string gnss_status;
  bool8_t safe_crank_switch;
  bool8_t mode_switch_state;
  bool8_t veh_signal_black[16];
  bool8_t veh_signal_purple[16];
  bool8_t veh_signal_checkered[16];
  bool8_t diag_hb_failure;
  int diagnostic_heartbeat_curr_value=32;
  int diagnostic_heartbeat_prev_value=23;
  int diagnostic_time = 0;
  int diagnostic_max_counter_drop = 20; //timeout 200ms

  deep_orange_msgs::msg::PtReport pt_report;
  raptor_dbw_msgs::msg::AcceleratorPedalReport accelerator_report;
  raptor_dbw_msgs::msg::BrakeReport brake_report;
  raptor_dbw_msgs::msg::SteeringReport steer_report;
  
};
}  // namespace ssc_interface

#endif  // SSC_INTERFACE__SSC_INTERFACE_HPP_
