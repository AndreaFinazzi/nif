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

#include <long_control.hpp>

namespace control {
LongControl::LongControl() : Node("LongControlNode") {
  this->initializeGears();

  // Publishers
  this->pubThrottleCmd_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/accelerator_cmd", 1);
  this->pubBrakeCmd_ =
      this->create_publisher<std_msgs::msg::Float32>("/joystick/brake_cmd", 1);
  this->pubGearCmd_ =
      this->create_publisher<std_msgs::msg::UInt8>("/joystick/gear_cmd", 1);
  this->pubControlStatus_ = this->create_publisher<std_msgs::msg::String>(
      "/bvs_long_control/control_status", 1);

  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();

  // Subscribers
  this->subJoystick =
      this->create_subscription<deep_orange_msgs::msg::JoystickCommand>(
          "/joystick/command", qos,
          std::bind(&LongControl::receiveJoystick, this,
                    std::placeholders::_1));
  this->subVelocity_ =
      this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
          "/raptor_dbw_interface/wheel_speed_report", 1,
          std::bind(&LongControl::receiveVelocity, this,
                    std::placeholders::_1));
  this->subPtReport_ =
      this->create_subscription<deep_orange_msgs::msg::PtReport>(
          "/raptor_dbw_interface/pt_report", 1,
          std::bind(&LongControl::receivePtReport, this,
                    std::placeholders::_1));
  this->subSafetyStatus_ =
      this->create_subscription<bvs_msgs::msg::SafetyStatus>(
          "/bvs_safety/safety_status", 1,
          std::bind(&LongControl::receiveSafetyStatus, this,
                    std::placeholders::_1));

  // Declare Parameters
  this->declare_parameter("desired_velocity", 12.5);
  this->declare_parameter("time_step", 0.01);
  this->declare_parameter("auto_enabled", false);

  this->declare_parameter("throttle.proportional_gain", 4.0);
  this->declare_parameter("throttle.integral_gain", 0.0);
  this->declare_parameter("throttle.derivative_gain", 0.0);
  this->declare_parameter("throttle.max_integrator_error", 10.0);
  this->declare_parameter("throttle.cmd_max", 25.0);
  this->declare_parameter("throttle.cmd_min", 0.0);
  this->declare_parameter("throttle.reset_integral_below_this_cmd", 15.0);

  this->declare_parameter("brake.proportional_gain", 4.0);
  this->declare_parameter("brake.integral_gain", 0.0);
  this->declare_parameter("brake.derivative_gain", 0.0);
  this->declare_parameter("brake.max_integrator_error", 10.0);
  this->declare_parameter("brake.cmd_max", 2000000.7);
  this->declare_parameter("brake.cmd_min", 0.0);
  this->declare_parameter("brake.reset_integral_below_this_cmd", 15.0);
  this->declare_parameter("brake.vel_error_deadband_mps", 0.5);

  this->declare_parameter("gear.shift_up", 13.0);
  this->declare_parameter("gear.shift_down", 11.0);
  this->declare_parameter("gear.shift_time_ms", 300);

  this->declare_parameter("safe_des_vel.safe_vel_thres_mph", 30.0);
  this->declare_parameter("safe_des_vel.hard_braking_time", 1.5);
  this->declare_parameter("safe_des_vel.soft_braking_time", 1.0);

  safe_vel_thres_mph_ =
      this->get_parameter("safe_des_vel.safe_vel_thres_mph").as_double();
  hard_braking_time_ =
      this->get_parameter("safe_des_vel.hard_braking_time").as_double();
  soft_braking_time_ =
      this->get_parameter("safe_des_vel.soft_braking_time").as_double();

  // Create Callback Timers
  this->ts_ = this->get_parameter("time_step").as_double();
  int timer_ms_ = static_cast<int>(ts_ * 1000);
  this->control_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(timer_ms_),
                              std::bind(&LongControl::controlCallback, this));

  // publish info about health of control
  this->status_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(25 * timer_ms_),
                              std::bind(&LongControl::statusTimerCallback, this));

  int timer_param_ms_ = static_cast<int>(ts_ * 10000);
  this->param_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(timer_param_ms_),
      std::bind(&LongControl::paramUpdateCallback, this));

  int timer_gear_ms_ = static_cast<int>(ts_ * 10000);
  this->gear_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(timer_gear_ms_),
                              std::bind(&LongControl::shiftCallback, this));

  // Initialize Commands
  this->throttle_cmd.data = 0.0;
  this->brake_cmd.data = 0.0;
  this->gear_cmd.data = 1;

  // Create throttle PID object
  this->p_ = this->get_parameter("throttle.proportional_gain").as_double();
  this->i_ = this->get_parameter("throttle.integral_gain").as_double();
  this->d_ = this->get_parameter("throttle.derivative_gain").as_double();
  this->iMax_ =
      this->get_parameter("throttle.max_integrator_error").as_double();
  this->throttleCmdMax_ = this->get_parameter("throttle.cmd_max").as_double();
  this->throttleCmdMin_ = this->get_parameter("throttle.cmd_min").as_double();
  this->iThrottleReset_ =
      this->get_parameter("throttle.reset_integral_below_this_cmd").as_double();

  this->vel_pid_ =
      PID(p_, i_, d_, ts_, iMax_, throttleCmdMax_, throttleCmdMin_);

  // Create brake PID object
  this->bp_ = this->get_parameter("brake.proportional_gain").as_double();
  this->bi_ = this->get_parameter("brake.integral_gain").as_double();
  this->bd_ = this->get_parameter("brake.derivative_gain").as_double();
  this->biMax_ = this->get_parameter("brake.max_integrator_error").as_double();
  this->brakeCmdMax_ = this->get_parameter("brake.cmd_max").as_double();
  this->brakeCmdMin_ = this->get_parameter("brake.cmd_min").as_double();
  this->iBrakeReset_ =
      this->get_parameter("brake.reset_integral_below_this_cmd").as_double();

  this->brake_pid_ =
      PID(bp_, bi_, bd_, ts_, biMax_, brakeCmdMax_, brakeCmdMin_);
}

void LongControl::initializeGears() {
  // LOR params
  // this->gear_states = {
  //    {1, std::make_shared<control::GearState>(1, 2.92, -255, 11)},
  //    {2, std::make_shared<control::GearState>(2, 1.875, 9.5, 16)},
  //    {3, std::make_shared<control::GearState>(3, 1.38, 14, 22)},
  //    {4, std::make_shared<control::GearState>(4, 1.5, 17, 30)},
  //    {5, std::make_shared<control::GearState>(5, 0.96, 22, 35)},
  //    {6, std::make_shared<control::GearState>(6, 0.889, 30, 255)}};

  // IMS params
   this->gear_states = {
      {1, std::make_shared<control::GearState>(1, 2.92, -255, 11)},
      {2, std::make_shared<control::GearState>(2, 1.875, 9.5, 22)},
      {3, std::make_shared<control::GearState>(3, 1.38, 19.5, 28.5)},
      {4, std::make_shared<control::GearState>(4, 1.5, 25, 37.5)},
      {5, std::make_shared<control::GearState>(5, 0.96, 35, 44)},
      {6, std::make_shared<control::GearState>(6, 0.889, 41.5, 255)}};

  this->curr_gear_ptr_ = this->gear_states[1];
}

void LongControl::controlCallback() {
  rclcpp::Time control_time = rclcpp::Clock().now();
  rclcpp::Duration time_diff = control_time - this->joy_recv_time_;
  double dt = static_cast<double>(time_diff.seconds()) +
              static_cast<double>(time_diff.nanoseconds()) * 1e-9;

  double error = calculateVelocityError();
  
  if (dt < 0.5) {
    calculateThrottleCmd(error);
  } else {
    RCLCPP_DEBUG(this->get_logger(), "%s\n",
                 "Have not received throttle max in > 0.5s !");
    setCmdsToZeros();
    this->comms_ok_ = false;
    this->safety_ok_ = false;
  }

  calculateBrakeCmd(error);
  publishThrottleBrake();
}

void LongControl::paramUpdateCallback() {
  this->auto_enabled_ = this->get_parameter("auto_enabled").as_bool();

  this->vel_pid_.SetPGain(
      this->get_parameter("throttle.proportional_gain").as_double());
  this->vel_pid_.SetIGain(
      this->get_parameter("throttle.integral_gain").as_double());
  this->vel_pid_.SetDGain(
      this->get_parameter("throttle.derivative_gain").as_double());
  this->vel_pid_.SetIMax(
      this->get_parameter("throttle.max_integrator_error").as_double());
  this->vel_pid_.SetCmdBounds(
      this->get_parameter("throttle.cmd_min").as_double(),
      this->get_parameter("throttle.cmd_max").as_double());

  this->brake_pid_.SetPGain(
      this->get_parameter("brake.proportional_gain").as_double());
  this->brake_pid_.SetIGain(
      this->get_parameter("brake.integral_gain").as_double());
  this->brake_pid_.SetDGain(
      this->get_parameter("brake.derivative_gain").as_double());
  this->brake_pid_.SetIMax(
      this->get_parameter("brake.max_integrator_error").as_double());
  this->brake_pid_.SetCmdBounds(
      this->get_parameter("brake.cmd_min").as_double(),
      this->get_parameter("brake.cmd_max").as_double());
}

double LongControl::calculateVelocityError() {
  rclcpp::Duration time_diff = rclcpp::Clock().now() - this->vel_recv_time_;
  double dt = static_cast<double>(time_diff.seconds()) +
              static_cast<double>(time_diff.nanoseconds()) * 1e-9;

  if (dt > 100 * this->ts_) {
    this->vel_pid_.ResetErrorIntegral();
  }

  double des_vel = this->get_parameter("desired_velocity").as_double();

  // Apply safe desired vel profiler for safe braking w.r.t. pose uncertainty
  des_vel = safeDesVelProfiler(des_vel);

  double vel_error = des_vel - this->speed_mps_;
  return vel_error;
}

void LongControl::calculateThrottleCmd(double vel_err) {
  if (vel_err > 0.0) {
    this->vel_pid_.Update(vel_err);
    this->throttle_cmd.data = this->vel_pid_.CurrentControl();
  } else {
    this->throttle_cmd.data = 0.0;
  }
}

void LongControl::calculateBrakeCmd(double vel_err) {
  double db = this->get_parameter("brake.vel_error_deadband_mps").as_double();
  if (!this->safety_ok_ || vel_err < -db) {
    this->brake_pid_.Update(-vel_err);
    this->brake_cmd.data = this->brake_pid_.CurrentControl();
    if(!this->safety_ok_ && this->speed_mps_ < 1.5)
    {
      this->brake_cmd.data = this->brakeCmdMax_;
    }
  } else {
    this->brake_cmd.data = 0.0;
    this->brake_pid_.ResetErrorIntegral();
  }
}

void LongControl::setCmdsToZeros() {
  this->throttle_cmd.data = 0.0;
  this->brake_cmd.data = 0.0;
}

void LongControl::publishThrottleBrake() {
  // Uses joystick cmds if autonomous mode is not enabled
  if (!this->comms_ok_ || auto_enabled_) {
    // run controller if comms is bad or auto is enabled
    // Sets the joystick throttle cmd as the saturation limit on throttle
    if (this->throttle_cmd.data > this->max_throttle_) {
      RCLCPP_DEBUG(this->get_logger(), "%s\n", "Throttle Limit Max Reached");
      this->throttle_cmd.data = this->max_throttle_;
    }
    this->brake_cmd.data = (this->brake_override_ > 100.0)
                               ? this->brake_override_
                               : this->brake_cmd.data;
    this->throttle_cmd.data =
        (this->brake_cmd.data > 0.0) ? 0.0 : this->throttle_cmd.data;
  } else if (!auto_enabled_) {
    this->brake_cmd.data = this->brake_override_;
    this->throttle_cmd.data = this->max_throttle_;
  }
  pubThrottleCmd_->publish(this->throttle_cmd);
  pubBrakeCmd_->publish(this->brake_cmd);
}

double LongControl::safeDesVelProfiler(double orig_des_vel) {
  /*
  Safe desired velocity profiler w.r.t. pose uncertainty
  : Decrease desired velocity while the GPS uncertainty (cov) is larger than
  threshold
  @ Args
    - orig_des_vel : original desired velocity in ROS param
  @ Params
    - pose_stdev_thres     : Threshold of the pose stdev
    - safe_vel_thres_mph   : Velocity (Mph) threshold w.r.t. safe braking time
    - hard_braking_time    : safe braking time when faster than
  safe_vel_thres_mph
    - soft_braking_time    : safe braking time when lower than
  safe_vel_thres_mph
  */
  const double mphToMps = 1.0 / 2.237;

  rclcpp::Time curr_time = rclcpp::Clock().now();
  double curr_stamp = static_cast<double>(curr_time.seconds()) +
                      static_cast<double>(curr_time.nanoseconds()) * 1e-9;
  double curr_vel = this->speed_mps_;

  if (!this->safety_ok_) {
    // Set (initial tick, vel) and (safe braking time, slope (braking accel))
    if (this->init_tick_ == -1.0 && this->init_vel_ == -1.0 &&
        this->safe_braking_time_ == -1.0) {
      this->init_tick_ = curr_stamp;
      this->init_vel_ = curr_vel;
      this->safe_braking_time_ = (curr_vel > safe_vel_thres_mph_ * mphToMps)
                                     ? hard_braking_time_
                                     : soft_braking_time_;
    }

    // Calculate decreasing desired vel
    double duration = curr_stamp - this->init_tick_;
    double safe_des_vel = this->init_vel_ - (this->init_vel_) /
                                                this->safe_braking_time_ *
                                                duration;
    // Saturation (safe_des_vel >= 1.0)
    safe_des_vel = (safe_des_vel > 1.0) ? safe_des_vel : 0.0;

    this->safe_des_vel_ = safe_des_vel;
    this->orig_des_vel_ = orig_des_vel;

    return safe_des_vel;
  } else {
    // Reset params
    this->init_tick_ = -1.0;
    this->init_vel_ = -1.0;
    this->safe_braking_time_ = -1.0;

    return orig_des_vel;
  }
}

void LongControl::shiftCallback() {
  // Uses joystick cmds if autonomous mode is not enabled
  if (!this->auto_enabled_) {
    this->gear_cmd.data = this->joy_gear_;
    pubGearCmd_->publish(this->gear_cmd);
    return;
  }

  double upshift_speed = this->curr_gear_ptr_->upshiftSpeed;
  double downshift_speed = this->curr_gear_ptr_->downshiftSpeed;
  unsigned int shift_time_limit =
      this->get_parameter("gear.shift_time_ms").as_int();
  int curr_gear_num = curr_gear_ptr_->gear;

  // grab current translational speed of the car
  double curr_speed = this->speed_mps_;

  // Sets command to current gear if engine is not on or shift attempts denied
  // over the limit
  if (!this->engine_running_ ||
      this->shifting_counter_ * 100 >= shift_time_limit) {
    this->gear_cmd.data = this->current_gear_;
    this->shifting_counter_ = 0;
    pubGearCmd_->publish(this->gear_cmd);
    return;
  }

  // Determine if a shift is required
  if (curr_speed > upshift_speed && this->throttle_cmd.data > 0.0 &&
      curr_gear_num < 4) {
    // change to next gear if not in 4th
    curr_gear_ptr_ = this->gear_states[curr_gear_num + 1];
    this->gear_cmd.data = curr_gear_num + 1;
    this->shifting_counter_++;
    RCLCPP_INFO(this->get_logger(), "Shifting UP from %d to %d", curr_gear_num,
                curr_gear_num + 1);
  } else if (curr_speed < downshift_speed && curr_gear_num > 1) {
    // downshift if not already in 1st
    curr_gear_ptr_ = this->gear_states[curr_gear_num - 1];
    this->gear_cmd.data = curr_gear_num - 1;
    this->shifting_counter_++;
    RCLCPP_INFO(this->get_logger(), "Shifting DOWN from %d to %d",
                curr_gear_num, curr_gear_num - 1);
  } else {
    // if still within threshold maintain same gear
    this->gear_cmd.data = curr_gear_num;
    this->shifting_counter_ = 0;
  }

  pubGearCmd_->publish(this->gear_cmd);  // send gear command
}

void LongControl::statusTimerCallback() {
  std::stringstream ss;

  if (!this->safety_ok_) {
    ss << "WARNING: executing safe slowdown."
       << " Safe des vel: " << this->safe_des_vel_
       << " Orig des vel: " << this->orig_des_vel_;
  } else {
    ss << "System Healthy";
  }
  this->status_msg.data = ss.str();
  this->pubControlStatus_->publish(this->status_msg);
}

void LongControl::receiveJoystick(
    const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {
  this->max_throttle_ = msg->accelerator_cmd;
  if (this->max_throttle_ <
      this->get_parameter("throttle.reset_integral_below_this_cmd")
          .as_double()) {
    this->vel_pid_.ResetErrorIntegral();
  }
  this->brake_override_ = msg->brake_cmd;
  this->joy_gear_ = msg->gear_cmd;

  this->joy_recv_time_ = rclcpp::Clock().now();
}

void LongControl::receiveVelocity(
    const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
  const double kphToMps = 1.0 / 3.6;
  // front left wheel speed (kph)
  // double front_left = msg->front_left;
  // double front_right = msg->front_right;
  double rear_left = msg->rear_left;
  double rear_right = msg->rear_right;
  // average wheel speeds (kph) and convert to m/s
  this->speed_mps_ = (rear_left + rear_right) * 0.5 * kphToMps;
  this->vel_recv_time_ = rclcpp::Clock().now();
}

void LongControl::receivePtReport(
    const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
  this->current_gear_ = msg->current_gear;
  this->engine_speed_ = msg->engine_rpm;
  this->engine_running_ = (msg->engine_rpm > 500) ? true : false;
}

void LongControl::receiveSafetyStatus(
    const bvs_msgs::msg::SafetyStatus::SharedPtr msg) {
  this->gps_ok_ = msg->gps_healthy;
  this->comms_ok_ = msg->comms_healthy;
  this->safety_ok_ = this->gps_ok_ && this->comms_ok_;
}

}  // end namespace control

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::LongControl>());
  rclcpp::shutdown();
  return 0;
}
