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

#include "nif_common/constants.h"
#include "nif_utils/utils.h"
#include <accel_control.hpp>

namespace control {
AccelControl::AccelControl() : Node("AccelControlNode") {

  // Publishers
  this->pubThrottleCmd_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/accelerator_cmd", 1);
  this->pubThrottleCmdRaw_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/accelerator_cmd/raw", 1);
  this->pubBrakeCmd_ =
      this->create_publisher<std_msgs::msg::Float32>("/joystick/brake_cmd", 1);
  this->pubBrakeCmdRaw_ = this->create_publisher<std_msgs::msg::Float32>(
      "/joystick/brake_cmd/raw", 1);
  this->pubGearCmd_ =
      this->create_publisher<std_msgs::msg::UInt8>("/joystick/gear_cmd", 1);
  //  this->pubControlStatus_ = this->create_publisher<std_msgs::msg::String>(
  //      "/control_low_level/control_status", 1);
  this->pubDiagnostic_ =
      this->create_publisher<nif_msgs::msg::AccelControlStatus>(
          "/accel_control/diagnostic", 1);

  // setup QOS to be best effort
  auto qos = rclcpp::QoS(
      rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
  qos.best_effort();

  // Subscribers
  this->subJoystick =
      this->create_subscription<deep_orange_msgs::msg::JoystickCommand>(
          "/joystick/command", rclcpp::SensorDataQoS(),
          std::bind(&AccelControl::receiveJoystick, this,
                    std::placeholders::_1));
  this->subVelocity_ =
      this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
          "/raptor_dbw_interface/wheel_speed_report", 1,
          std::bind(&AccelControl::receiveVelocity, this,
                    std::placeholders::_1));
  this->subDesAccel_ = this->create_subscription<std_msgs::msg::Float32>(
      "/control_safety_layer/out/desired_accel", rclcpp::SensorDataQoS(),
      std::bind(&AccelControl::receiveDesAccel, this, std::placeholders::_1));
  this->subPtReport_ =
      this->create_subscription<deep_orange_msgs::msg::PtReport>(
          "/raptor_dbw_interface/pt_report", 1,
          std::bind(&AccelControl::receivePtReport, this,
                    std::placeholders::_1));
  this->subImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "in_imu_data", rclcpp::SensorDataQoS(),
      std::bind(&AccelControl::receiveImu, this, std::placeholders::_1));

  // Declare Parameters
  this->declare_parameter("time_step", 0.01);
  this->declare_parameter("engine_based_throttle_enabled", false);

  this->declare_parameter("throttle.k_accel", 0.05780);
  this->declare_parameter("throttle.k_accel2", 0.0);
  this->declare_parameter("throttle.k_bias", 0.09998);
  this->declare_parameter("throttle.pedalToCmd", 100.0);

  this->declare_parameter("throttle.cmd_max", 30.0);
  this->declare_parameter("throttle.cmd_min", 0.0);
  this->declare_parameter("throttle.des_accel_deadband", 0.05);

  this->declare_parameter("brake.k_accel", 0.1635);
  this->declare_parameter("brake.k_accel2", -0.0065);
  this->declare_parameter("brake.k_bias", 0.0299);
  this->declare_parameter("brake.pedalToCmd", 3447379.0);

  this->declare_parameter("brake.cmd_max", 2000000.7);
  this->declare_parameter("brake.cmd_min", 0.0);
  this->declare_parameter("brake.des_accel_deadband", 0.1);

  this->declare_parameter("gear.shift_up", 4000.0);
  this->declare_parameter("gear.shift_down", 2200.0);
  this->declare_parameter("gear.shift_time_ms", 1000);
  this->declare_parameter("gear.track", "IMS");

  this->declare_parameter("engine.model_safety_factor", 1.2); // larger than 1.0
  this->declare_parameter("engine.safety_rpm_thres", 3000);

  this->declare_parameter("throttle.traction_enabled", true);
  this->declare_parameter("throttle.traction_throttle_cmd_thres", 0.5);
  this->declare_parameter("throttle.traction_factor", 0.2);
  this->declare_parameter("throttle.traction_rate", 15.0);

  this->declare_parameter("brake.abs_enabled", true);
  this->declare_parameter("brake.abs_brake_cmd_thres", 0.5);
  this->declare_parameter("brake.abs_factor", 0.2);
  this->declare_parameter("brake.abs_rate", 15.0);

  this->declare_parameter("traction_abs.velocity_thres_mps", 20.0);
  this->declare_parameter("traction_abs.sigma_thres", 0.06);
  this->declare_parameter("traction_abs.control_rate", 100.0);

  // GEAR INIT
  this->initializeGears(this->get_parameter("gear.track").as_string());

  // Create Callback Timers
  this->ts_ = this->get_parameter("time_step").as_double();

  int timer_gear_ms_ = static_cast<int>(ts_ * 10000);
  this->gear_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(timer_gear_ms_),
                              std::bind(&AccelControl::shiftCallback, this));

  // Initialize Commands
  this->throttle_cmd.data = 0.0;
  this->brake_cmd.data = 0.0;
  this->gear_cmd.data = 1;

  // Create throttle accel controller object
  this->throttle_k_accel_ = this->get_parameter("throttle.k_accel").as_double();
  this->throttle_k_accel2_ =
      this->get_parameter("throttle.k_accel2").as_double();
  this->throttle_k_bias_ = this->get_parameter("throttle.k_bias").as_double();
  this->throttle_pedalToCmd_ =
      this->get_parameter("throttle.pedalToCmd").as_double();
  this->throttleCmdMax_ = this->get_parameter("throttle.cmd_max").as_double();
  this->throttleCmdMin_ = this->get_parameter("throttle.cmd_min").as_double();

  this->engine_based_throttle_enabled_ =
      this->get_parameter("engine_based_throttle_enabled").as_bool();

  if (engine_based_throttle_enabled_) {
    // Engine model-based throttle controller
    double engine_safety_factor =
        this->get_parameter("engine.model_safety_factor").as_double();
    int engine_safety_rpm_thres =
        this->get_parameter("engine.safety_rpm_thres").as_int();

    if (engine_safety_factor < 1.0) {
      RCLCPP_ERROR(this->get_logger(), "Got engine.model_safety_factor: %f;",
                   engine_safety_factor);
      throw std::range_error("Parameter engine.model_safety_factor must be "
                             "greater or equal than 1.0.");
    }

    this->m_throttle_controller_engine_ =
        EngineMapAccelController(engine_safety_factor, engine_safety_rpm_thres,
                                 1.0, throttleCmdMax_, throttleCmdMin_);
  } else {
    // Throttle profile-based throttle controller
    this->m_throttle_controller_profiler_ = ThrottleBrakeProfiler(
        throttle_k_accel_, throttle_k_accel2_, throttle_k_bias_,
        throttle_pedalToCmd_, ts_, throttleCmdMax_, throttleCmdMin_);
  }

  // Create brake accel controller object
  this->brake_k_accel_ = this->get_parameter("brake.k_accel").as_double();
  this->brake_k_accel2_ = this->get_parameter("brake.k_accel2").as_double();
  this->brake_k_bias_ = this->get_parameter("brake.k_bias").as_double();
  this->brake_pedalToCmd_ = this->get_parameter("brake.pedalToCmd").as_double();
  this->brakeCmdMax_ = this->get_parameter("brake.cmd_max").as_double();
  this->brakeCmdMin_ = this->get_parameter("brake.cmd_min").as_double();

  if (this->brakeCmdMax_ < 1000000) {
    RCLCPP_ERROR(this->get_logger(), "Got brake.cmd_max: %f;",
                 this->brakeCmdMax_);
    throw std::range_error(
        "Parameter brake.cmd_max must be greater or equal than 10000.0.");
  }

  this->m_brake_controller_ =
      ThrottleBrakeProfiler(brake_k_accel_, brake_k_accel2_, brake_k_bias_,
                            brake_pedalToCmd_, ts_, brakeCmdMax_, brakeCmdMin_);

  bool traction_enabled =
      this->get_parameter("throttle.traction_enabled").as_bool();
  double traction_throttle_cmd_thres =
      this->get_parameter("throttle.traction_throttle_cmd_thres")
          .as_double(); // 0.5
  double traction_factor =
      this->get_parameter("throttle.traction_factor").as_double(); // 0.2
  double traction_rate =
      this->get_parameter("throttle.traction_rate").as_double(); // 15.

  bool ABS_enabled = this->get_parameter("brake.abs_enabled").as_bool();
  double ABS_brake_cmd_thres =
      this->get_parameter("brake.abs_brake_cmd_thres").as_double(); // 0.5
  double ABS_factor =
      this->get_parameter("brake.abs_factor").as_double();             // 0.2
  double ABS_rate = this->get_parameter("brake.abs_rate").as_double(); // 15.

  double velocity_thres_mps =
      this->get_parameter("traction_abs.velocity_thres_mps")
          .as_double(); // 27.78 mps, 100 kph
  double sigma_thres =
      this->get_parameter("traction_abs.sigma_thres").as_double(); // 0.06
  double control_rate =
      this->get_parameter("traction_abs.control_rate").as_double();

  if (traction_enabled && ((traction_throttle_cmd_thres < 0.0 ||
                            traction_throttle_cmd_thres > 1.0) ||
                           (traction_factor < 0.0 || traction_factor > 1.0) ||
                           (traction_rate <= 0.0 || traction_rate >= 100.0))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Got traction_enabled and parameter out of range.");
    throw std::range_error("traction_enabled prameters out of range.");
  } else if (ABS_enabled &&
             ((ABS_brake_cmd_thres < 0.5 || ABS_brake_cmd_thres > 1.0) ||
              (ABS_factor < 0.0 || ABS_factor > 1.0) ||
              (ABS_rate <= 0.0 || ABS_rate >= 100.0) ||
              (velocity_thres_mps < 0.0 || velocity_thres_mps > 27.0) ||
              (sigma_thres < 0.0 || sigma_thres > 1.0) ||
              (control_rate <= ABS_rate))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Got ABS_enabled and parameter out of range.");
    throw std::range_error("ABS_enabled prameters out of range.");
  }

  this->m_traction_ABS_controller_ = TractionABS(
      traction_enabled, traction_throttle_cmd_thres, traction_factor,
      traction_rate, ABS_enabled, ABS_brake_cmd_thres, ABS_factor, ABS_rate,
      velocity_thres_mps, sigma_thres, control_rate);
}

void AccelControl::initializeGears(const std::string &track_id) {
  if (track_id == TRACK_ID_LOR) {
    // LOR params
    this->gear_states = {
        {1, std::make_shared<control::GearState>(1, 2.92, -255, 11)},
        {2, std::make_shared<control::GearState>(2, 1.875, 9.5, 16)},
        {3, std::make_shared<control::GearState>(3, 1.38, 14, 22)},
        {4, std::make_shared<control::GearState>(4, 1.5, 17, 30)},
        {5, std::make_shared<control::GearState>(5, 0.96, 22, 35)},
        {6, std::make_shared<control::GearState>(6, 0.889, 30, 255)}};
  } else if (track_id == TRACK_ID_IMS) {
    // IMS params
    this->gear_states = {
        {1, std::make_shared<control::GearState>(1, 2.92, -255, 13.5)},
        {2, std::make_shared<control::GearState>(2, 1.875, 11, 22)},
        {3, std::make_shared<control::GearState>(3, 1.38, 19.5, 30)},
        {4, std::make_shared<control::GearState>(4, 1.5, 27.5, 37.5)},
        {5, std::make_shared<control::GearState>(5, 0.96, 35, 44)},
        {6, std::make_shared<control::GearState>(6, 0.889, 41.5, 255)}};
  } else {
    RCLCPP_ERROR(this->get_logger(),
                 "Got unrecognized track_id: %s, parameter out of range.",
                 track_id.c_str());
    throw std::range_error("track_id out of range.");
  }

  this->curr_gear_ptr_ = this->gear_states[1];
}

void AccelControl::paramUpdateCallback() {}

void AccelControl::calculateThrottleCmd(double des_accel) {
  bool imu_ok = this->has_m_imu_ &&
                this->now() - this->m_imu_update_time <= rclcpp::Duration(1, 0);
  // If imu is not okay, set the acceleration zero.
  if (!imu_ok) {
    RCLCPP_ERROR_ONCE(this->get_logger(),
                      "Imu data is too old, unreliable acceleration control!");
    m_a_lon_kf = 0;
    m_a_lat_kf = 0;
  }
  // Get Longitudinal Acceleration Limit using Vehicle dynamics manager
  m_max_a_lon =
      m_tire_manager_.ComputeLongitudinalAccelLimit(m_a_lon_kf, m_a_lat_kf);
  // Constrain longitudinal acceleration
  des_accel = std::min(des_accel, m_max_a_lon);

  // throttle command
  double db = this->get_parameter("throttle.des_accel_deadband").as_double();
  double throttle_cmd_out = 0.0;
  int gear_num = this->current_gear_;
  int engine_speed = this->engine_speed_;

  if (des_accel > db) {
    if (engine_based_throttle_enabled_) {
      throttle_cmd_out = this->m_throttle_controller_engine_.CurrentControl(
          des_accel, gear_num, engine_speed);
    } else {
      throttle_cmd_out =
          this->m_throttle_controller_profiler_.CurrentControl(des_accel);
    }
  }

  // Traction Control
  // - calculate tire slip ratio
  m_sigma =
      m_tire_manager_.CalcTireSlipRatio(this->front_speed_, this->rear_speed_);
  // - get current translational speed of the car
  double curr_speed = this->speed_;
  // Compute Traction control output
  throttle_cmd_out = m_traction_ABS_controller_.tractionControl(
      throttle_cmd_out, curr_speed, m_sigma);
  // for diagnostic
  m_traction_activated = m_traction_ABS_controller_.m_traction_activated;
  m_desired_engine_torque =
      m_throttle_controller_engine_.m_engine_manager.m_desired_engine_torque;
  m_max_engine_torque =
      m_throttle_controller_engine_.m_engine_manager.m_max_engine_torque;
  // - final throttle command output
  this->throttle_cmd.data = throttle_cmd_out;
}

void AccelControl::calculateBrakeCmd(double des_accel) {
  // brake command
  double db = this->get_parameter("brake.des_accel_deadband").as_double();
  double brake_cmd_out = 0.0;
  if (des_accel < -db) {
    brake_cmd_out = this->m_brake_controller_.CurrentControl(des_accel);
  }
  // ABS Control
  // - calculate tire slip ratio
  m_sigma =
      m_tire_manager_.CalcTireSlipRatio(this->front_speed_, this->rear_speed_);
  // - get current translational speed of the car
  double curr_speed = this->speed_;
  // - compute ABS control output
  brake_cmd_out =
      m_traction_ABS_controller_.ABSControl(brake_cmd_out, curr_speed, m_sigma);
  m_abs_activated = m_traction_ABS_controller_.m_abs_activated;
  // - braking for complete stop
  // -- 46% brake bedal input at -3 m/s2 command
  if (curr_speed < 3.0 && des_accel < 0) {
    brake_cmd_out = this->m_brake_controller_.CurrentControl(-3.0);
  }
  // - final brake command output
  this->brake_cmd.data = brake_cmd_out;
}

void AccelControl::setCmdsToZeros() {
  this->throttle_cmd.data = 0.0;
  this->brake_cmd.data = 0.0;
}

void AccelControl::publishThrottleBrake() {
  // Uses joystick cmds if autonomous mode is not enabled
  // run controller if comms is bad or auto is enabled
  // Sets the joystick throttle cmd as the saturation limit on throttle

  pubThrottleCmdRaw_->publish(this->throttle_cmd);
  pubBrakeCmdRaw_->publish(this->brake_cmd);

  if (this->throttle_cmd.data > this->max_throttle_) {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Throttle Limit Max Reached");
    this->throttle_cmd.data = this->max_throttle_;
  }

  this->throttle_cmd.data =
      (this->brake_cmd.data > 0.0) ? 0.0 : this->throttle_cmd.data;

  if (!this->isDataOk()) {
    RCLCPP_ERROR_ONCE(
        this->get_logger(),
        "Data is too old, accel_control_node is blindly braking!");
    this->throttle_cmd.data = 0.0;
    this->brake_cmd.data = this->m_brake_controller_.CurrentControl(-8.0);
  }

  pubThrottleCmd_->publish(this->throttle_cmd);
  pubBrakeCmd_->publish(this->brake_cmd);
}

void AccelControl::publishDiagnostic(double is_engine_based,
                                     double traction_activated,
                                     double abs_activated, double max_a_lon,
                                     double tire_slip_ratio,
                                     double desired_engine_torque,
                                     double max_engine_torque) {
  // Diagnostic: {curvature, a_lat_max}
  nif_msgs::msg::AccelControlStatus diagnostic;

  diagnostic.stamp = this->now();

  diagnostic.is_engine_based = is_engine_based;
  diagnostic.traction_activated = traction_activated;
  diagnostic.abs_activated = abs_activated;
  diagnostic.max_a_lon = max_a_lon;

  diagnostic.tire_slip_ratio = tire_slip_ratio;
  diagnostic.desired_engine_torque = desired_engine_torque;
  diagnostic.max_engine_torque = max_engine_torque;

  pubDiagnostic_->publish(diagnostic);
}

void AccelControl::shiftCallback() {

  double upshift_speed = this->curr_gear_ptr_->upshiftSpeed;
  double downshift_speed = this->curr_gear_ptr_->downshiftSpeed;
  unsigned int shift_time_limit =
      this->get_parameter("gear.shift_time_ms").as_int();
  int curr_gear_num = curr_gear_ptr_->gear;

  // grab current translational speed of the car
  double curr_speed = this->speed_;

  // Sets command to current gear if engine is not on or shift attempts denied
  // over the limit
  if (!this->engine_running_ ||
      this->shifting_counter_ * 100 >= shift_time_limit) {
    this->gear_cmd.data = this->current_gear_;
    this->shifting_counter_ = 0;
    //    pubGearCmd_->publish(this->gear_cmd);
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

  //  pubGearCmd_->publish(this->gear_cmd); // send gear command
}

void AccelControl::receiveJoystick(
    const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {
  this->has_joy_ = true;
  this->max_throttle_ = msg->accelerator_cmd;
  this->joy_gear_ = msg->gear_cmd;
  this->joy_recv_time_ = this->now();
}

void AccelControl::receiveVelocity(
    const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
  this->has_vel_ = true;
  const double kphToMps = 1.0 / 3.6;
  // front left wheel speed (kph)
  double front_left = msg->front_left;
  double front_right = msg->front_right;
  double rear_left = msg->rear_left;
  double rear_right = msg->rear_right;
  // average wheel speeds (kph) and convert to m/s
  this->speed_ = (rear_left + rear_right) * 0.5 * kphToMps;
  // front/rear wheel speed
  this->front_speed_ = (front_left + front_right) * 0.5 * kphToMps;
  this->rear_speed_ = (rear_left + rear_right) * 0.5 * kphToMps;
  this->vel_recv_time_ = this->now();
}

void AccelControl::receiveDesAccel(
    const std_msgs::msg::Float32::SharedPtr msg) {
  // get desired acceleration (m/s^2) from high level controll
  this->has_des_accel_ = true;
  this->des_accel_ = msg->data;
  this->des_accel_recv_time_ = this->now();
  double current_des_accel = this->des_accel_;

  calculateThrottleCmd(current_des_accel);
  calculateBrakeCmd(current_des_accel);

  publishThrottleBrake();
  pubGearCmd_->publish(this->gear_cmd); // send gear command

  // publish diagnostic message
  publishDiagnostic(engine_based_throttle_enabled_, m_traction_activated,
                    m_abs_activated, m_max_a_lon, m_sigma,
                    m_desired_engine_torque, m_max_engine_torque);
}

void AccelControl::receivePtReport(
    const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
  this->has_pt_report_ = true;
  this->current_gear_ = msg->current_gear;
  this->engine_speed_ = msg->engine_rpm;
  this->engine_running_ = (msg->engine_rpm > 500) ? true : false;
  this->pt_report_recv_time_ = this->now();
}

void AccelControl::receiveImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // kalman filter initialization
  auto now = this->now();
  float dt = 0.01;
  if (this->has_m_imu_)
    dt = nif::common::utils::time::secs(now - this->m_imu_update_time);

  this->has_m_imu_ = true;
  this->m_imu_update_time = now;

  if (!kalman_init) {
    kf_a_lat.init(2);
    kf_a_lat.setProcessNoise(0.1, 0.01);
    kf_a_lat.setMeasurementNoise(0.4);

    kf_a_lon.init(2);
    kf_a_lon.setProcessNoise(0.1, 0.01);
    kf_a_lon.setMeasurementNoise(0.4);

    kalman_init = true;
  }
  // kalman filtering
  // - prediction
  kf_a_lat.predict(dt);
  kf_a_lon.predict(dt);
  // - get filtered values
  m_a_lon_kf = kf_a_lon.get();
  m_a_lat_kf = kf_a_lat.get();
  // - correction
  kf_a_lon.correct((float)(msg->linear_acceleration.x));
  kf_a_lat.correct((float)(msg->linear_acceleration.y));
}

} // end namespace control

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<control::AccelControl>());
  rclcpp::shutdown();
  return 0;
}
