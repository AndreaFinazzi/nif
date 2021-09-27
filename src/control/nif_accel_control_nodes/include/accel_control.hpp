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

#include <control_model.hpp>
#include <rclcpp/rclcpp.hpp>

#include <deep_orange_msgs/msg/joystick_command.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>

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

private:
  void initializeGears();
  void controlCallback();
  void paramUpdateCallback();

  void calculateThrottleCmd(double vel_err);
  void calculateBrakeCmd(double vel_err);
  void setCmdsToZeros();
  void publishThrottleBrake();
  double safeDesVelProfiler(double orig_des_vel);
  void shiftCallback();
  void statusCallback();
  void
  receiveJoystick(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg);
  void
  receiveVelocity(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg);
  void receiveDesAccel(const std_msgs::msg::Float32::SharedPtr msg);
  void receivePtReport(const deep_orange_msgs::msg::PtReport::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr gear_timer_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubThrottleCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubThrottleCmdRaw_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubBrakeCmd_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pubBrakeCmdRaw_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pubGearCmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pubControlStatus_;
  rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr
      subJoystick;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr
      subVelocity_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subDesAccel_;
  rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr subPtReport_;

  rclcpp::Time joy_recv_time_;
  rclcpp::Time vel_recv_time_;
  rclcpp::Time des_accel_recv_time_;

  bool auto_enabled_ = false;
  double speed_ = 0.0;
  double des_accel_ = 0.0;

  double max_throttle_ = 0.0;
  double brake_override_ = 0.0;
  unsigned int joy_gear_ = 1;
  double ts_;
  unsigned int current_gear_ = 0;
  unsigned int engine_speed_ = 0;
  bool engine_running_ = false;
  unsigned int shifting_counter_ = 0;

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

  AccelController throttle_controller_;
  AccelController brake_controller_;

}; // end of class

} // namespace control

#endif
