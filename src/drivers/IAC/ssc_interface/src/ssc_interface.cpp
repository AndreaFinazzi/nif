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

#include "ssc_interface/ssc_interface.hpp"

#include <automotive_platform_msgs/msg/gear.hpp>
#include <rclcpp/logging.hpp>
#include <time_utils/time_utils.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cmath>
#include <stdexcept>
#include <iostream>
using SscGear = automotive_platform_msgs::msg::Gear;

namespace ssc_interface
{

SscInterface::SscInterface(
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
)
: m_logger{node.get_logger()},
  m_front_axle_to_cog{front_axle_to_cog},
  m_rear_axle_to_cog{rear_axle_to_cog},
  m_accel_limit{max_accel_mps2},
  m_decel_limit{max_decel_mps2},
  m_max_yaw_rate{max_yaw_rate_rad},
  m_dbw_state_machine(new DbwStateMachine{3}),
  m_rc_flag_received{false},
  m_flag_summary_received{false},
  m_steering_gear_ratio{steering_gear_ratio},
  m_acc_pedal_min_limit{acc_pedal_min_limit},
  m_acc_pedal_max_limit{acc_pedal_max_limit},
  m_brake_pressure_min_limit{brake_pressure_min_limit},
  m_brake_pressure_max_limit{brake_pressure_max_limit},
  m_steering_min_limit{steering_min_limit},
  m_steering_max_limit{steering_max_limit},
  m_joy_enabled{joy_enabled},
  counter_acc{0},
  counter_brake{0},
  counter_steer{0},
  counter_gear{0},
  counter_ct{0}
{
  // Publishers (to SSC)
  // m_gear_cmd_pub = node.create_publisher<GearCommand>("gear_select", 10);
  m_speed_cmd_pub = node.create_publisher<SpeedMode>("arbitrated_speed_commands", 10);
  m_steer_cmd_pub = node.create_publisher<SteerMode>("arbitrated_steering_commands", 10);
  m_turn_signal_cmd_pub = node.create_publisher<TurnSignalCommand>(
    "turn_signal_command", 10);
  
  // Publishers (to Autoware)
  m_kinematic_state_pub =
    node.create_publisher<VehicleKinematicState>("vehicle_kinematic_state_cog", 10);

////////////////////////////////////////////////////////////////////
  m_gear_pub = node.create_publisher<std_msgs::msg::UInt8>("gear_cmd", 10);
  m_accelerator_pedal_pub = node.create_publisher<raptor_dbw_msgs::msg::AcceleratorPedalCmd>("accelerator_pedal_cmd", 10);
  m_brake_pressure_pub = node.create_publisher<raptor_dbw_msgs::msg::BrakeCmd>(
    "brake_cmd",
    10);
  //Need to add acceleration pedal demand
  m_steering_cmd_pub = node.create_publisher<raptor_dbw_msgs::msg::SteeringCmd>("steering_cmd", 10);
  m_gear_shift_cmd = node.create_publisher<std_msgs::msg::UInt8>("gear_shift_cmd", 10);
  m_ct_report_pub = node.create_publisher<CtReport>("ct_status", 10);

  //Publish Stop Command to Graceful Stop Node
  stop_command = node.create_publisher<std_msgs::msg::Bool>("stop_command",10);
  
//////////////////////////////////////////////////////////////////////

  // Subscribers (from SSC)
  m_dbw_state_sub =
    node.create_subscription<std_msgs::msg::Bool>(
    "dbw_enabled_feedback", rclcpp::QoS{10},
    [this](std_msgs::msg::Bool::SharedPtr msg) {on_dbw_state_report(msg);});
  m_gear_feedback_sub =
    node.create_subscription<GearFeedback>(
    "gear_feedback", rclcpp::QoS{10},
    [this](GearFeedback::SharedPtr msg) {on_gear_report(msg);});
  m_vel_accel_sub =
    node.create_subscription<VelocityAccelCov>(
    "velocity_accel_cov", rclcpp::QoS{10},
    [this](VelocityAccelCov::SharedPtr msg) {on_vel_accel_report(msg);});
  m_steer_sub =
    node.create_subscription<SteeringFeedback>(
    "steering_feedback", rclcpp::QoS{10},
    [this](SteeringFeedback::SharedPtr msg) {on_steer_report(msg);});

/////////////////////////////////////////////////////////////////////////
  // Subscribers (from SSC)
  m_misc_report_sub =
    node.create_subscription<deep_orange_msgs::msg::MiscReport>(
    "misc_report", rclcpp::QoS{10},
    [this](deep_orange_msgs::msg::MiscReport::SharedPtr msg) {misc_report_callback(msg);});
  m_pt_report_sub =
    node.create_subscription<deep_orange_msgs::msg::PtReport>(
    "pt_report", rclcpp::QoS{10},
    [this](deep_orange_msgs::msg::PtReport::SharedPtr msg) {pt_report_callback(msg);});
  m_accelerator_report_sub =
    node.create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalReport>(
    "accelerator_report", rclcpp::QoS{10},
    [this](raptor_dbw_msgs::msg::AcceleratorPedalReport::SharedPtr msg) {
      accelerator_report_callback(msg);
    });
  m_brake_report_sub =
    node.create_subscription<raptor_dbw_msgs::msg::BrakeReport>(
    "brake_pressure_report", rclcpp::QoS{10},
    [this](raptor_dbw_msgs::msg::BrakeReport::SharedPtr msg) {brake_report_callback(msg);});
  m_steer_report_sub =
    node.create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
    "steer_report", rclcpp::QoS{10},
    [this](raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg) {steer_report_callback(msg);});
  m_rc_to_ct_sub = node.create_subscription<deep_orange_msgs::msg::RcToCt>(
    "rc_to_ct_info", rclcpp::QoS{10},
    [this](deep_orange_msgs::msg::RcToCt::SharedPtr msg) {rc_to_ct_callback(msg);});
  m_flag_summary_sub = node.create_subscription<deep_orange_msgs::msg::BaseToCarSummary>(
          "flag_summary", rclcpp::ServicesQoS(),
          [this](deep_orange_msgs::msg::BaseToCarSummary::SharedPtr msg) {flag_summary_callback(msg);});
  m_vehicle_kinematic_state_sub = node.create_subscription<VehicleKinematicState>(
    "vehicle_kinematic_state", rclcpp::QoS{10},
    [this](VehicleKinematicState::SharedPtr msg) {vehicle_kinematic_state_callback(msg);});
  m_acc_pedal_demand_sub = node.create_subscription<std_msgs::msg::Float32>(
    "accelerator_cmd_pt", rclcpp::QoS{10},
    [this](std_msgs::msg::Float32::SharedPtr msg) {accelerator_cmd_callback(msg);});
  m_brake_demand_sub = node.create_subscription<std_msgs::msg::Float32>(
    "brake_cmd_pt", rclcpp::QoS{10},
    [this](std_msgs::msg::Float32::SharedPtr msg) {brake_cmd_callback(msg);});
  
  m_gear_demand_sub = node.create_subscription<std_msgs::msg::UInt8>(
    "gear_cmd_pt", rclcpp::QoS{10},
    [this](std_msgs::msg::UInt8::SharedPtr msg) {gear_cmd_callback(msg);});
  //Subscriber (from Autoware)
  m_emergency_joy_bool = node.create_subscription<std_msgs::msg::Bool>(
    "/vehicle/emergency_joystick", rclcpp::QoS{10},
    [this](std_msgs::msg::Bool::SharedPtr msg) {emergency_joy_callback(msg);});
  m_emergency_hb_bool = node.create_subscription<std_msgs::msg::Bool>(
    "/vehicle/emergency_heartbeat", rclcpp::QoS{10},
    [this](std_msgs::msg::Bool::SharedPtr msg) {emergency_hb_callback(msg);});
  m_emergency_confirmation = node.create_subscription<std_msgs::msg::Int8>(
    "/vehicle/shutdown_confirmation", rclcpp::QoS{10},
    [this](std_msgs::msg::Int8::SharedPtr msg) {emergency_confirmation_callback(msg);});

  auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos.best_effort();

  m_enable_joy_sub = node.create_subscription<deep_orange_msgs::msg::JoystickCommand>(
    "/joystick/command", qos,
    [this](deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {enable_joy_callback(msg);});
  m_diag_hb_sub = node.create_subscription<std_msgs::msg::Int32>(
    "/diagnostics/heartbeat", rclcpp::QoS{10},
    [this](std_msgs::msg::Int32::SharedPtr msg) {diagnostics_heartbeat_callback(msg);});

  //Joystick  
  // if(m_joy_enabled){
  m_joystick_app = node.create_subscription<std_msgs::msg::Float32>(
    "/joystick/accelerator_cmd", rclcpp::QoS{10},
    [this](std_msgs::msg::Float32::SharedPtr msg) {joystick_accelerator_cmd_callback(msg);});
  m_joystick_brake = node.create_subscription<std_msgs::msg::Float32>(
    "/joystick/brake_cmd", rclcpp::QoS{10},
    [this](std_msgs::msg::Float32::SharedPtr msg) {joystick_brake_cmd_callback(msg);});
  // m_joystick_steer = node.create_subscription<autoware_auto_msgs::msg::RawControlCommand>(
  //   "/joystick/raw_command", rclcpp::QoS{10},
  //   [this](autoware_auto_msgs::msg::RawControlCommand::SharedPtr msg) {joystick_steer_cmd_callback(msg);});
  m_joystick_steer = node.create_subscription<std_msgs::msg::Float32>(
    "/joystick/steering_cmd", rclcpp::QoS{10},
    [this](std_msgs::msg::Float32::SharedPtr msg) {joystick_steer_cmd_callback(msg);});
  m_joystick_gear = node.create_subscription<std_msgs::msg::UInt8>(
    "/joystick/gear_cmd", rclcpp::QoS{10},
    [this](std_msgs::msg::UInt8::SharedPtr msg) {joystick_gear_cmd_callback(msg);});
  // }
  // m_joystick_status_sub =
  // node.create_subscription<autoware_auto_msgs::msg::JoyStickEnabled>(
  //   "joy_enabled", rclcpp::QoS{10},
  //   [this](autoware_auto_msgs::msg::JoyStickEnabled::SharedPtr msg) {joy_stick_callback(msg);});

  // m_raw_joystick_cmd_sub =
  //  node.create_subscription<RawControlCommand>(
  //   "raw_command", rclcpp::QoS{10},
  //   [this](RawControlCommand::SharedPtr msg) {raw_joystick_cmd_sub(msg);});



//////////////////////////////////////////////////////////////////////////
}

bool8_t SscInterface::update(std::chrono::nanoseconds timeout)
{
  (void)timeout;
  return true;
}

bool8_t SscInterface::send_state_command(const VehicleStateCommand & msg)
{
  // Turn signal command
  TurnSignalCommand tsc;
  tsc.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  tsc.turn_signal = TurnSignalCommand::NONE;

  switch (msg.blinker) {
    case VehicleStateCommand::BLINKER_NO_COMMAND:
      break;
    case VehicleStateCommand::BLINKER_OFF:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::NONE;
      break;
    case VehicleStateCommand::BLINKER_LEFT:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::LEFT;
      break;
    case VehicleStateCommand::BLINKER_RIGHT:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::RIGHT;
      break;
    case VehicleStateCommand::BLINKER_HAZARD:
      tsc.mode = 1;
      tsc.turn_signal = TurnSignalCommand::NONE;
      RCLCPP_WARN(m_logger, "Received command for unsuported turn signal state.");
      break;
    default:
      RCLCPP_ERROR(m_logger, "Received command for invalid turn signal state.");
  }

  tsc.header.stamp = msg.stamp;
  m_turn_signal_cmd_pub->publish(tsc);

  // Gear command
  GearCommand gc;
  // Has no mode - only listens if at least one
  // other DBW system is enabled
  gc.command.gear = SscGear::NONE;

  switch (msg.gear) {
    case VehicleStateCommand::GEAR_NO_COMMAND:
      break;
    case VehicleStateCommand::GEAR_DRIVE:
      gc.command.gear = SscGear::DRIVE;
      break;
    case VehicleStateCommand::GEAR_REVERSE:
      gc.command.gear = SscGear::REVERSE;
      break;
    case VehicleStateCommand::GEAR_PARK:
      gc.command.gear = SscGear::PARK;
      break;
    case VehicleStateCommand::GEAR_LOW:
      gc.command.gear = SscGear::LOW;
      break;
    case VehicleStateCommand::GEAR_NEUTRAL:
      gc.command.gear = SscGear::NEUTRAL;
      break;
    default:
      RCLCPP_ERROR(m_logger, "Received command for invalid gear state.");
  }

  gc.header.stamp = msg.stamp;
  m_gear_cmd_pub->publish(gc);

  m_dbw_state_machine->state_cmd_sent();

  return true;
}

bool8_t SscInterface::send_control_command(const HighLevelControlCommand & msg)
{
  auto desired_velocity{0.0F};

  // Handle velocities opposite the current direction of travel
  if (
    (state_report().gear == VehicleStateReport::GEAR_DRIVE && msg.velocity_mps < 0.0F) ||
    (state_report().gear == VehicleStateReport::GEAR_REVERSE && msg.velocity_mps > 0.0F))
  {
    desired_velocity = 0.0F;
  } else {
    desired_velocity = std::fabs(msg.velocity_mps);
  }

  // Publish speed command
  SpeedMode speed_mode;
  speed_mode.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  speed_mode.speed = desired_velocity;
  speed_mode.acceleration_limit = m_accel_limit;
  speed_mode.deceleration_limit = m_decel_limit;
  speed_mode.header.stamp = msg.stamp;
  m_speed_cmd_pub->publish(speed_mode);

  // Publish steering command
  SteerMode steer_mode;
  constexpr float32_t curvature_rate = 0.15F;  // assume the rate is constant.
  steer_mode.mode = m_dbw_state_machine->enabled() ? 1 : 0;
  steer_mode.curvature = msg.curvature;
  steer_mode.max_curvature_rate = curvature_rate;  // should be positive
  steer_mode.header.stamp = msg.stamp;
  m_steer_cmd_pub->publish(steer_mode);

  m_dbw_state_machine->control_cmd_sent();

  return true;
}

bool8_t SscInterface::send_control_command(const RawControlCommand & msg)
{
  (void)msg;
  RCLCPP_ERROR(m_logger, "SSC does not support sending raw pedal controls directly.");
  return false;
}

void SscInterface::joystick_steer_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg)
{

//////////////////////////////////////////////////////////////////
  if(enable_joy == 1){
    SteeringCmd steer;
    float front_steer = msg->data;
    if(front_steer * m_steering_gear_ratio > m_steering_max_limit){
      steer.angle_cmd = m_steering_max_limit;
    }
    else if(front_steer * m_steering_gear_ratio < m_steering_min_limit){
      steer.angle_cmd = m_steering_min_limit;
    }
    else{
    steer.angle_cmd = front_steer * m_steering_gear_ratio;//hand wheel steer angle value in degrees
    }
    // steer.angle_cmd = -steer.angle_cmd; // TODO: remember it

    steer.rolling_counter = counter_steer;
    m_steering_cmd_pub->publish(steer);
    counter_steer++;
    if(counter_steer == 8)
    {
        counter_steer = 0;
    }
  }
}

bool8_t SscInterface::send_control_command(const VehicleControlCommand & msg)
{

//////////////////////////////////////////////////////////////////
  if(enable_joy == 0){
    SteeringCmd steer;
    if(msg.front_wheel_angle_rad * m_steering_gear_ratio * (180.0F/3.14F) > m_steering_max_limit){
      steer.angle_cmd = m_steering_max_limit;
    }
    else if(msg.front_wheel_angle_rad * m_steering_gear_ratio * (180.0F/3.14F) < m_steering_min_limit){
      steer.angle_cmd = m_steering_min_limit;
    }
    else{
    steer.angle_cmd = msg.front_wheel_angle_rad * m_steering_gear_ratio * (180.0F/3.14F);//hand wheel steer angle value in degrees
    }
    // steer.angle_cmd = -steer.angle_cmd; // TODO: remember it

    steer.rolling_counter = counter_steer;
    m_steering_cmd_pub->publish(steer);
    counter_steer++;
    if(counter_steer == 8){
    counter_steer = 0;
    }
  
  }
//////////////////////////////////////////////////////////////////


  auto signed_velocity = msg.velocity_mps;

  if (msg.velocity_mps > 0.0F && state_report().gear == VehicleStateReport::GEAR_REVERSE) {
    signed_velocity = -msg.velocity_mps;
  }

  const auto wheelbase = m_front_axle_to_cog + m_rear_axle_to_cog;

  HighLevelControlCommand hlc_cmd;
  hlc_cmd.stamp = msg.stamp;

  // Convert from center-of-mass velocity to rear-axle-center velocity
  const auto beta =
    std::atan(m_front_axle_to_cog * std::tan(msg.front_wheel_angle_rad) / (wheelbase));
  hlc_cmd.velocity_mps = std::cos(beta) * signed_velocity;

  // Calculate curvature from desired steering angle
  hlc_cmd.curvature = std::tan(msg.front_wheel_angle_rad) / (wheelbase);

  return send_control_command(hlc_cmd);
}

bool8_t SscInterface::handle_mode_change_request(ModeChangeRequest::SharedPtr request)
{
  if (request->mode == ModeChangeRequest::MODE_MANUAL) {
    m_dbw_state_machine->user_request(false);
    return true;
  } else if (request->mode == ModeChangeRequest::MODE_AUTONOMOUS) {
    m_dbw_state_machine->user_request(true);
    return true;
  } else {
    RCLCPP_ERROR(m_logger, "Got invalid autonomy mode request value.");
    return false;
  }
}

void SscInterface::on_dbw_state_report(const std_msgs::msg::Bool::SharedPtr & msg)
{
  if (msg->data) {
    state_report().mode = VehicleStateReport::MODE_AUTONOMOUS;
  } else {
    state_report().mode = VehicleStateReport::MODE_MANUAL;
  }

  m_dbw_state_machine->dbw_feedback(msg->data);
}

void SscInterface::on_gear_report(const GearFeedback::SharedPtr & msg)
{
  switch (msg->current_gear.gear) {
    case SscGear::PARK:
      state_report().gear = VehicleStateReport::GEAR_PARK;
      break;
    case SscGear::REVERSE:
      state_report().gear = VehicleStateReport::GEAR_REVERSE;
      break;
    case SscGear::NEUTRAL:
      state_report().gear = VehicleStateReport::GEAR_NEUTRAL;
      break;
    case SscGear::DRIVE:
      state_report().gear = VehicleStateReport::GEAR_DRIVE;
      break;
    case SscGear::LOW:
      state_report().gear = VehicleStateReport::GEAR_LOW;
      break;
    case SscGear::NONE:
    default:
      state_report().gear = 0;
      RCLCPP_WARN(m_logger, "Received invalid gear value from SSC.");
  }
}

void SscInterface::vehicle_kinematic_state_callback(const VehicleKinematicState::SharedPtr & msg)
{
  m_vehicle_kinematic_state = *msg;
  vehicle_vel = m_vehicle_kinematic_state.state.longitudinal_velocity_mps;

}

void SscInterface::on_steer_report(const SteeringFeedback::SharedPtr & msg)
{
  const auto front_wheel_angle_rad = msg->steering_wheel_angle * STEERING_TO_TIRE_RATIO;
  odometry().stamp = msg->header.stamp;
  odometry().front_wheel_angle_rad = front_wheel_angle_rad;
  odometry().rear_wheel_angle_rad = 0.0F;

  std::lock_guard<std::mutex> guard(m_vehicle_kinematic_state_mutex);
  m_vehicle_kinematic_state.state.front_wheel_angle_rad = front_wheel_angle_rad;
  m_seen_steer = true;
}

void SscInterface::on_vel_accel_report(const VelocityAccelCov::SharedPtr & msg)
{
  odometry().stamp = msg->header.stamp;
  odometry().velocity_mps = msg->velocity;

  std::lock_guard<std::mutex> guard(m_vehicle_kinematic_state_mutex);
  // Input velocity is (assumed to be) measured at the rear axle, but we're
  // producing a velocity at the center of gravity.
  // Lateral velocity increases linearly from 0 at the rear axle to the maximum
  // at the front axle, where it is tan(δ)*v_lon.
  const float32_t wheelbase = m_rear_axle_to_cog + m_front_axle_to_cog;
  const float32_t delta = m_vehicle_kinematic_state.state.front_wheel_angle_rad;
  m_vehicle_kinematic_state.header.frame_id = "odom";
  m_vehicle_kinematic_state.state.longitudinal_velocity_mps = msg->velocity;
  m_vehicle_kinematic_state.state.lateral_velocity_mps = (m_rear_axle_to_cog / wheelbase) *
    msg->velocity * std::tan(delta);
  m_vehicle_kinematic_state.state.acceleration_mps2 = msg->accleration;
  // Dt can not be calculated from the first message alone
  if (!m_seen_vel_accel) {
    m_seen_vel_accel = true;
    m_vehicle_kinematic_state.header.stamp = msg->header.stamp;
    return;
  }

  // Calculate dt
  float32_t dt = static_cast<float32_t>(
    msg->header.stamp.sec - m_vehicle_kinematic_state.header.stamp.sec);
  dt += static_cast<float32_t>(
    msg->header.stamp.sec - m_vehicle_kinematic_state.header.stamp.sec) / 1000000000.0F;

  if (dt < 0.0F) {
    RCLCPP_WARN(m_logger, "Received inconsistent timestamps.");
  }

  m_vehicle_kinematic_state.header.stamp = msg->header.stamp;

  if (m_seen_steer) {
    // TODO(Takamasa Horibe): modify after AVP with TF specifications
    // position or yaw is 0 since odom=baselink with static TF in AVP demo
    m_vehicle_kinematic_state.state.x = 0.0F;
    m_vehicle_kinematic_state.state.y = 0.0F;
    m_vehicle_kinematic_state.state.heading.real = std::cos(/*yaw*/ 0.0F / 2.0F);
    m_vehicle_kinematic_state.state.heading.imag = std::sin(/*yaw*/ 0.0F / 2.0F);
    const float32_t beta = std::atan2(m_rear_axle_to_cog * std::tan(delta), wheelbase);
    m_vehicle_kinematic_state.state.heading_rate_rps = std::cos(beta) * std::tan(delta) / wheelbase;
    m_kinematic_state_pub->publish(m_vehicle_kinematic_state);
  }
}

////////////////////////////////////////////////////////////////////////////////////

void SscInterface::rc_to_ct_callback(const deep_orange_msgs::msg::RcToCt::SharedPtr & msg)
{
  rc_to_ct_data = *msg;
  if (msg) {m_rc_flag_received = true;}
  switch (rc_to_ct_data.track_cond) {
    case 1:    // Start engine, perform any final checks
      m_rec_track_cond = TrackCondition::TC1_RED;
      break;
    case 2:     // Proceed with caution
      m_rec_track_cond = TrackCondition::TC2_ORANGE;
      break;
    case 3:     // Race
      m_rec_track_cond = TrackCondition::TC3_YELLOW;
      break;
    case 4:     // Emergency Coordinated Stop
      m_rec_track_cond = TrackCondition::TC4_GREEN;
      break;
    case 255:
      m_rec_track_cond = TrackCondition::TC_DEFAULT;
      break;
    
    // case 4:     // Pit Entry
    //   race_mode_command = RaceFlag::CHECKERED;
    //   break;
    // case 5:     // ShutOff engine
    //   race_mode_command = RaceFlag::PURPLE;
    //   break;

    // case 7:     // Penalty - enter the pit and stay
    //   race_mode_command = RaceFlag::BLACK;
    //   break;
    default:
      RCLCPP_ERROR(m_logger, "Incorrect RC Mode Command Received.");
      // m_ct_to_rc_data = m_dbw_state_state_machine->ct_mode_transition(race_mode_command)
  }

  if(rc_to_ct_data.black[15]==1){
    m_rec_veh_signal = VehicleSignal::VS2_BLACK;}
  else if (rc_to_ct_data.purple[15]==1){
    m_rec_veh_signal = VehicleSignal::VS8_PURPLE;}
  else if (rc_to_ct_data.checkered[15]==1){
    m_rec_veh_signal = VehicleSignal::VS4_CHECK;}
  else{
    m_rec_veh_signal = VehicleSignal::VS1_NULL;}

rolling_counter_RC = rc_to_ct_data.rolling_counter;
// if(prev-rolling_counter_RC == 7 || prev-rolling_counter_RC == 1)
//   prev = rolling_counter_RC;
// else 
//   throw std::domain_error{"Missing messages"};

}

void SscInterface::flag_summary_callback(const deep_orange_msgs::msg::BaseToCarSummary::SharedPtr &msg)
{
    flag_summary_data = *msg;
    if (msg) {m_flag_summary_received = true;}
    switch (flag_summary_data.track_flag) {
        case 0:
            m_rec_track_cond = TrackCondition::TC0_NULL;
            break;
        case 1:    // Start engine, perform any final checks
            m_rec_track_cond = TrackCondition::TC1_RED;
            break;
        case 2:     // Proceed with caution
            m_rec_track_cond = TrackCondition::TC2_ORANGE;
            break;
        case 3:     // Race
            m_rec_track_cond = TrackCondition::TC3_YELLOW;
            break;
        case 4:     // Emergency Coordinated Stop
            m_rec_track_cond = TrackCondition::TC4_GREEN;
            break;
        case 255:
            m_rec_track_cond = TrackCondition::TC_DEFAULT;
            break;
        default:
            RCLCPP_ERROR(m_logger, "Incorrect RC Mode (track_flag) Command Received.");
            // m_ct_to_rc_data = m_dbw_state_state_machine->ct_mode_transition(race_mode_command)
    }

    switch (flag_summary_data.veh_flag) {
        case 0:
            m_rec_veh_signal = VehicleSignal::VS0_NULL;
            break;
        case 1:
            m_rec_veh_signal = VehicleSignal::VS1_NULL;
            break;
        case 2:
            m_rec_veh_signal = VehicleSignal::VS2_BLACK;
            break;
        case 4:
            m_rec_veh_signal = VehicleSignal::VS4_CHECK;
            break;
        case 8:
            m_rec_veh_signal = VehicleSignal::VS8_PURPLE;
            break;
        default:
            RCLCPP_ERROR(m_logger, "Incorrect RC Mode (veh_flag) Command Received.");
    }

    rolling_counter_RC = flag_summary_data.base_to_car_heartbeat;
    // if(prev-rolling_counter_RC == 7 || prev-rolling_counter_RC == 1)
    //   prev = rolling_counter_RC;
    // else
    //   throw std::domain_error{"Missing messages"};

}


void SscInterface::emergency_joy_callback(const std_msgs::msg::Bool::SharedPtr & msg){
  emergency_joy_flag = msg->data;
}

void SscInterface::emergency_hb_callback(const std_msgs::msg::Bool::SharedPtr & msg){
  emergency_hb_flag = msg->data;
}

void SscInterface::diagnostic_checker(){
  if(diagnostic_heartbeat_prev_value != diagnostic_heartbeat_curr_value){
            diagnostic_time = 0;
            diag_hb_failure = false;
            diagnostic_heartbeat_prev_value = diagnostic_heartbeat_curr_value;
        }
        else{
            diagnostic_time++;
            if(diagnostic_time>= diagnostic_max_counter_drop){
                diag_hb_failure = true;
            }
        }
}

void SscInterface::diagnostics_heartbeat_callback(const std_msgs::msg::Int32::SharedPtr & msg){
  diagnostic_heartbeat_curr_value = msg->data;
}

void SscInterface::emergency_confirmation_callback(const std_msgs::msg::Int8::SharedPtr & msg){
  emergency_confirmation = msg->data;
}

void SscInterface::enable_joy_callback(const deep_orange_msgs::msg::JoystickCommand::SharedPtr & msg){
  // std::cout << "Joystick Enable: " << enable_joy << std::endl;
  if (enable_joy == 0){
    enable_joy = msg->joy_enable;
  }
}

void SscInterface::joystick_gear_cmd_callback(const std_msgs::msg::UInt8::SharedPtr & msg){
  if(true){
    if(msg->data > m_gear_max){
      m_gear_cmd.data = m_gear_max;
    }
    else if(msg->data < m_gear_min){
      m_gear_cmd.data = m_gear_min;
    }
    else{
    m_gear_cmd.data = msg->data;
    }
    //  m_gear_cmd.rolling_counter = counter_gear;
    m_gear_pub->publish(m_gear_cmd);
    counter_gear++;
    if(counter_gear == 8){
      counter_gear = 0;
    } 
  }
}

void SscInterface::gear_cmd_callback(const std_msgs::msg::UInt8::SharedPtr & msg){
  if(false){
    if(msg->data > m_gear_max){
      m_gear_cmd.data = m_gear_max;
    }
    else if(msg->data < m_gear_min){
      m_gear_cmd.data = m_gear_min;
    }
    else{
    m_gear_cmd.data = msg->data;
    }
    //  m_gear_cmd.rolling_counter = counter_gear;
    m_gear_pub->publish(m_gear_cmd);
    counter_gear++;
    if(counter_gear == 8){
      counter_gear = 0;
    } 
  }
}

void SscInterface::joystick_accelerator_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg){
  if(true){
    if(msg->data > m_acc_pedal_max_limit)
    {
      m_acc_cmd.pedal_cmd = m_acc_pedal_max_limit;
    }
    else if(msg->data < m_acc_pedal_min_limit){
      m_acc_cmd.pedal_cmd = m_acc_pedal_min_limit;
    }
    else{
      m_acc_cmd.pedal_cmd = msg->data;
    }
    m_acc_cmd.rolling_counter = counter_acc;
    m_accelerator_pedal_pub->publish(m_acc_cmd);
    counter_acc++;
    if(counter_acc == 8){
      counter_acc = 0;
    } 
  }
}

void SscInterface::accelerator_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg){
  if(false){
    if(msg->data > m_acc_pedal_max_limit)
    {
      m_acc_cmd.pedal_cmd = m_acc_pedal_max_limit;
    }
    else if(msg->data < m_acc_pedal_min_limit){
      m_acc_cmd.pedal_cmd = m_acc_pedal_min_limit;
    }
    else{
      m_acc_cmd.pedal_cmd = msg->data;
    }
    m_acc_cmd.rolling_counter = counter_acc;
    m_accelerator_pedal_pub->publish(m_acc_cmd);
    counter_acc++;
    if(counter_acc == 8){
      counter_acc = 0;
    } 
  }
}

void SscInterface::joystick_brake_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg){
  if(true){
    auto demanded_brake_pressure = msg->data/1000.0F;  
    if(demanded_brake_pressure > m_brake_pressure_max_limit){
      m_brake_cmd.pedal_cmd  = m_brake_pressure_max_limit;
    }
    else if(demanded_brake_pressure < m_brake_pressure_min_limit){
      m_brake_cmd.pedal_cmd  = m_brake_pressure_min_limit;
    }
    else{
    m_brake_cmd.pedal_cmd = demanded_brake_pressure; //BRake pressure from joystick in Pa
    }

    m_brake_cmd.rolling_counter = counter_brake;
    m_brake_pressure_pub->publish(m_brake_cmd);
    counter_brake++;
    if(counter_brake == 8){
      counter_brake = 0;
    } 
  }
}

void SscInterface::brake_cmd_callback(const std_msgs::msg::Float32::SharedPtr & msg){
  if(false){
    if(msg->data > m_brake_pressure_max_limit){
      m_brake_cmd.pedal_cmd  = m_brake_pressure_max_limit;
    }
    else if(msg->data < m_brake_pressure_min_limit){
      m_brake_cmd.pedal_cmd  = m_brake_pressure_min_limit;
    }
    else{
      m_brake_cmd.pedal_cmd = msg->data;
    }
    
    m_brake_cmd.rolling_counter = counter_brake;
    m_brake_pressure_pub->publish(m_brake_cmd);
    counter_brake++;
    if(counter_brake == 8){
      counter_brake = 0;
    } 
  }
}

void SscInterface::misc_report_callback(const deep_orange_msgs::msg::MiscReport::SharedPtr & msg)
{
  misc_report = *msg;
  switch (msg->sys_state) {
    case 1:
      raptor_mode = SysState::SS1_PWR_ON;
      break;
    case 2:
      raptor_mode = SysState::SS2_SUBSYS_CON;
      break;
    case 3:
      raptor_mode = SysState::SS3_ACT_TESTING;
      break;
    case 4:
      raptor_mode = SysState::SS4_ACT_TEST_DONE;
      break;
    case 5:
      raptor_mode = SysState::SS5_CRANKREADY;
      break;
    case 6:
      raptor_mode = SysState::SS6_PRECRANK_CHECK;
      break;
    case 7:
      raptor_mode = SysState::SS7_CRANKING;
      break;
    case 8:
      raptor_mode = SysState::SS8_ENG_RUNNING;
      break;
    case 9:
      raptor_mode = SysState::SS9_DRIVING;
      break;
    case 10:
      raptor_mode = SysState::SS10_SHUT_ENG;
      break;
    case 11:
      raptor_mode = SysState::SS11_PWR_OFF;
      break;
    case 13:
      raptor_mode = SysState::SS13_CRANK_CHECK_INIT;
      break;
    default:
      RCLCPP_ERROR(m_logger, "Incorrect Raptor Mode Received. sys_state: %d", msg->sys_state);
  }

  // m_dbw_state_machine->gnss_feedback(gnss_status);
  // std::cout << msg->current_raptor_mode << std::endl;
  // safe_crank_switch = misc_report.crank_switch_enabled;
  mode_switch_state = misc_report.mode_switch_state;
  // counter_checker(counter);
  diagnostic_checker();
  m_ct_report_data = m_dbw_state_machine->ct_mode_transition(
    raptor_mode, m_rec_track_cond, vehicle_vel, m_rec_veh_signal, emergency_joy_flag, emergency_confirmation, diag_hb_failure);
  m_ct_report_data.track_cond_ack = flag_summary_data.track_flag;
  m_ct_report_data.veh_sig_ack = flag_summary_data.veh_flag;
  m_ct_report_data.rolling_counter = counter_ct;
  stop_msg = std_msgs::msg::Bool();
  if (emergency_hb_flag == true){
    stop_msg.data = true;
  }else{
    stop_msg.data = false;
  }
  stop_command->publish(stop_msg);
  m_ct_report_pub->publish(m_ct_report_data);
  counter_ct++;
  if(counter_ct == 8){
    counter_ct = 0;
  } 
  
  
}

// void SscInterface::raw_joystick_cmd_sub(const RawControlCommand::SharedPtr & msg){
//   BrakeCmd joy_brake;
//   joy_brake.pedal_cmd = static_cast<float32_t>(msg->brake);

//   SteeringCmd joy_steer;
//   joy_steer.angle_cmd = static_cast<float32_t>(msg->front_steer);

//   if(m_ct_status_data.current_ct_mode == 10)  //joystick
//   {
//     m_brake_pressure_pub->publish(joy_brake);
//     m_steering_cmd_pub->publish(joy_steer);
//   }
// }


// void SscInterface::joy_stick_callback(const JoyStickEnabled::SharedPtr & msg){

//   joy_stick_switch = msg->joy_stick_enabled;
// }

void SscInterface::pt_report_callback(const deep_orange_msgs::msg::PtReport::SharedPtr & msg)
{
  pt_report = *msg;
}

void SscInterface::accelerator_report_callback(
  const raptor_dbw_msgs::msg::AcceleratorPedalReport::SharedPtr & msg)
{
  accelerator_report = *msg;

}

void SscInterface::brake_report_callback(const raptor_dbw_msgs::msg::BrakeReport::SharedPtr & msg)
{
  brake_report = *msg;
}

void SscInterface::steer_report_callback(
  const raptor_dbw_msgs::msg::SteeringReport::SharedPtr & msg)
{
  steer_report = *msg;
}
/////////////////////////////////////////////////////////////////////////////////////

// Update x, y, heading, and heading_rate from the other variables
// TODO(nikolai.morin): Clean up and implement as a motion model
void SscInterface::kinematic_bicycle_model(
  float32_t dt, float32_t l_r, float32_t l_f, VehicleKinematicState * vks)
{
  // convert to yaw – copied from trajectory_spoofer.cpp
  // The below formula could probably be simplified if it would be derived directly for heading
  const float32_t sin_y = 2.0F * vks->state.heading.real * vks->state.heading.imag;
  const float32_t cos_y = 1.0F - 2.0F * vks->state.heading.imag * vks->state.heading.imag;
  float32_t yaw = std::atan2(sin_y, cos_y);
  if (yaw < 0) {
    yaw += TAU;
  }
  // δ: tire angle (relative to car's main axis)
  // φ: heading/yaw
  // β: direction of movement at point of reference (relative to car's main axis)
  // l_r: distance of point of reference to rear axle
  // l_f: distance of point of reference to front axle
  // x, y, v are at the point of reference
  // x' = v cos(φ + β)
  // y' = v sin(φ + β)
  // φ' = (cos(β)tan(δ)) / (l_r + l_f)
  // v' = a
  // β = arctan((l_r*tan(δ))/(l_r + l_f))

  // TODO(nikolai.morin): Decouple from VehicleKinematicState, use only v0 as
  // input. Currently v0_lat + v0_lon are redundant with beta/delta via
  // beta = atan2(v_lat, v_lon).
  float32_t v0_lat = vks->state.lateral_velocity_mps;
  float32_t v0_lon = vks->state.longitudinal_velocity_mps;
  float32_t v0 = std::sqrt(v0_lat * v0_lat + v0_lon * v0_lon);
  float32_t delta = vks->state.front_wheel_angle_rad;
  float32_t a = vks->state.acceleration_mps2;
  float32_t beta = std::atan2(l_r * std::tan(delta), l_r + l_f);
  // This is the direction in which the POI is moving at the beginning of the
  // integration step. "Course" may not be super accurate, but it's to
  // emphasize that the POI doesn't travel in the heading direction.
  const float32_t course = yaw + beta;
  // How much the yaw changes per meter traveled (at the reference point)
  const float32_t yaw_change =
    std::cos(beta) * std::tan(delta) / (l_r + l_f);
  // How much the yaw rate
  const float32_t yaw_rate = yaw_change * v0;
  // Threshold chosen so as to not result in division by 0
  if (std::abs(yaw_rate) < 1e-18f) {
    vks->state.x += std::cos(course) * (v0 * dt + 0.5f * a * dt * dt);
    vks->state.y += std::sin(course) * (v0 * dt + 0.5f * a * dt * dt);
  } else {
    vks->state.x +=
      (v0 + a * dt) / yaw_rate * std::sin(course + yaw_rate * dt) -
      v0 / yaw_rate * std::sin(course) +
      a / (yaw_rate * yaw_rate) * std::cos(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::cos(course);
    vks->state.y +=
      -(v0 + a * dt) / yaw_rate * std::cos(course + yaw_rate * dt) +
      v0 / yaw_rate * std::cos(course) +
      a / (yaw_rate * yaw_rate) * std::sin(course + yaw_rate * dt) -
      a / (yaw_rate * yaw_rate) * std::sin(course);
  }
  yaw += std::cos(beta) * std::tan(delta) / (l_r + l_f) * (v0 * dt + 0.5f * a * dt * dt);
  vks->state.heading.real = std::cos(yaw / 2.0f);
  vks->state.heading.imag = std::sin(yaw / 2.0f);

  // Rotations per second or rad per second?
  vks->state.heading_rate_rps = yaw_rate;
}

}  // namespace ssc_interface
