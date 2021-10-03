/* Copyright 2021 Will Bryan

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef ACCEL_CONTROL_HPP
#define ACCEL_CONTROL_HPP

#include <chrono>
#include <functional>
#include <map>
#include <math.h>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <deep_orange_msgs/msg/joystick_command.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "nif_msgs/msg/accel_control_status.hpp"
#include "nif_vehicle_dynamics_manager/kalman.h"
#include <TractionABS.hpp>
#include <control_model.hpp>

namespace control {

// Gear State
// Contains: gear_number, downshift speed, upshift speed, pointer to previous
// gear, pointer to next gear
struct GearState {
  int gear;
  double gearRatio;
  double downshiftSpeed;
  double upshiftSpeed;

  GearState(int gear, double gearRatio, double downshiftSpeed,
            double upshiftSpeed) {
    this->gear = gear;
    this->gearRatio = gearRatio;
    this->downshiftSpeed = downshiftSpeed;
    this->upshiftSpeed = upshiftSpeed;
  }
};

class AccelControl : public rclcpp::Node {
public:
  AccelControl();

  std_msgs::msg::Float32 throttle_cmd;
  std_msgs::msg::Float32 brake_cmd;
  std_msgs::msg::UInt8 gear_cmd;
  std_msgs::msg::String status_msg;

  // diagnostic data
  double m_traction_activated = 0.; // 0 for false
  double m_abs_activated = 0.;      // 0 for false
  double m_sigma = 0.;
  double m_desired_engine_torque = 0.;
  double m_max_engine_torque = 0.;
  double m_max_a_lon = 0.;

private:
  void initializeGears();
  void controlCallback();
  void paramUpdateCallback();

  void calculateThrottleCmd(double vel_err);
  void calculateBrakeCmd(double vel_err);
  void setCmdsToZeros();
  void publishThrottleBrake();
  void publishDiagnostic(double is_engine_based, double traction_activated,
                         double abs_activated, double max_a_lon,
                         double tire_slip_ratio, double desired_engine_torque,
                         double max_engine_torque);
  void shiftCallback();
  void statusCallback();
  void
  receiveJoystick(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg);
  void
  receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
  void receiveDesAccel(const std_msgs::msg::Float32::SharedPtr msg);
  void receivePtReport(const deep_orange_msgs::msg::PtReport::SharedPtr msg);
  void receiveImu(const sensor_msgs::msg::Imu::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr gear_timer_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubThrottleCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubThrottleCmdRaw_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubBrakeCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubBrakeCmdRaw_;
  rclcpp::Publisher<nif_msgs::msg::AccelControlStatus>::SharedPtr
      pubDiagnostic_;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pubGearCmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubControlStatus_;
  rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr
      subJoystick;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr
      subVelocity_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subDesAccel_;
  rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr subPtReport_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;

  rclcpp::Time joy_recv_time_;
  rclcpp::Time vel_recv_time_;
  rclcpp::Time des_accel_recv_time_;
  rclcpp::Time m_imu_update_time = rclcpp::Clock().now();

  bool auto_enabled_ = false;
  double speed_ = 0.0;
  double front_speed_ = 0.0;
  double rear_speed_ = 0.0;
  double des_accel_ = 0.0;

  double max_throttle_ = 0.0;
  double brake_override_ = 0.0;
  unsigned int joy_gear_ = 1;
  double ts_;
  unsigned int current_gear_ = 0;
  unsigned int engine_speed_ = 0;
  bool engine_running_ = false;
  unsigned int shifting_counter_ = 0;

  bool kalman_init = false;
  double m_a_lon_kf = 0.0; // longitudinal accel from KF
  double m_a_lat_kf = 0.0; // lateral accel from KF

  bool engine_based_throttle_enabled_ = false;

  double throttle_k_accel_;
  double throttle_k_accel2_;
  double throttle_k_bias_;
  double throttle_pedalToCmd_;
  double throttleCmdMax_;
  double throttleCmdMin_;

  double brake_k_accel_;
  double brake_k_accel2_;
  double brake_k_bias_;
  double brake_pedalToCmd_;
  double brakeCmdMax_;
  double brakeCmdMin_;

  std::map<int, std::shared_ptr<GearState>> gear_states;
  std::shared_ptr<control::GearState> curr_gear_ptr_;

  // Kalman filters
  KalmanFilter kf_a_lat;
  KalmanFilter kf_a_lon;

  TireManager m_tire_manager_;
  ThrottleBrakeProfiler m_throttle_controller_profiler_;
  EngineMapAccelController m_throttle_controller_engine_;
  ThrottleBrakeProfiler m_brake_controller_;
  TractionABS m_traction_ABS_controller_;

  double secs(rclcpp::Time t) {
    return static_cast<double>(t.seconds()) +
           static_cast<double>(t.nanoseconds()) * 1e-9;
  }
  double secs(rclcpp::Duration t) {
    return static_cast<double>(t.seconds()) +
           static_cast<double>(t.nanoseconds()) * 1e-9;
  }

}; // end of class

} // namespace control

#endif
