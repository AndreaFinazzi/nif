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

#include "control_model.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace control {
// ***** Model - ThrottleBrakeProfiler ***** //
ThrottleBrakeProfiler::ThrottleBrakeProfiler(
    const double &K_accel, const double &K_accel2, const double &K_bias,
    const double &pedalToCmd, const double &dt, const double &cmdMax,
    const double &cmdMin) {
  // Set control parameters
  k_accel_ = K_accel;
  k_accel2_ = K_accel2;
  k_bias_ = K_bias;
  pedalToCmd_ = pedalToCmd;

  // Set valid bounds
  SetCmdBounds(cmdMin, cmdMax);
}

/* ThrottleBrakeProfiler ALGORITHM */
double ThrottleBrakeProfiler::CurrentControl(double des_accel) {
  double cmd = 0.0;

  cmd = k_accel_ * std::abs(des_accel) + k_accel2_ * std::pow(des_accel, 2.) +
        k_bias_;
  cmd = pedalToCmd_ * cmd;

  return SaturateCmd(cmd);
}
/* SETTERS */
std::pair<bool, bool> ThrottleBrakeProfiler::SetCmdBounds(const double &min,
                                                          const double &max) {
  bool isMinSet = false, isMaxSet = false;
  if (min <= 0.0) {
    this->cmdMin_ = min;
    isMinSet = true;
  }
  if (max > 0.0) {
    this->cmdMax_ = max;
    isMaxSet = true;
  }
  return std::make_pair(isMinSet, isMaxSet);
}

/* PRIVATE METHODS */
double ThrottleBrakeProfiler::SaturateCmd(const double &cmd) {
  return std::min(std::max(cmdMin_, cmd), cmdMax_);
}

// ***** Model - EngineMapAccelController ***** //
EngineMapAccelController::EngineMapAccelController(
    const double &engine_safety_factor, const int &engine_safety_rpm_thres,
    const double &pedalToCmd, const double &cmdMax, const double &cmdMin) {
  // Set control parameters
  pedalToCmd_ = pedalToCmd; // pedalToCmd is 1.0
  // update engine manager parameters
  m_engine_manager.m_gamma = engine_safety_factor;
  m_engine_manager.m_rpm_safe_thres = engine_safety_rpm_thres;
  // Set valid bounds
  SetCmdBounds(cmdMin, cmdMax);
}
/* EngineMapAccelController ALGORITHM */
double EngineMapAccelController::CurrentControl(double desired_acceleration,
                                                int gear_num,
                                                int engine_speed) {
  double cmd = 0.0;

  // Compute desired_engine_torque from desired_acceleration
  // (desired longitudinal tire force)
  double desired_tire_longitudinal_force = mass * desired_acceleration;
  double desired_engine_torque = m_engine_manager.computeEngineTorque(
      desired_tire_longitudinal_force, gear_num);

  // Compute desired_throttle_position from desired_engine_torque
  double desired_throttle_position =
      m_engine_manager.inverseEngineModel(engine_speed, desired_engine_torque);

  cmd = pedalToCmd_ * desired_throttle_position; // [0~100]

  return SaturateCmd(cmd);
}

/* SETTERS */
std::pair<bool, bool>
EngineMapAccelController::SetCmdBounds(const double &min, const double &max) {
  bool isMinSet = false, isMaxSet = false;
  if (min <= 0.0) {
    this->cmdMin_ = min;
    isMinSet = true;
  }
  if (max > 0.0) {
    this->cmdMax_ = max;
    isMaxSet = true;
  }
  return std::make_pair(isMinSet, isMaxSet);
}

/* PRIVATE METHODS */
double EngineMapAccelController::SaturateCmd(const double &cmd) {
  return std::min(std::max(cmdMin_, cmd), cmdMax_);
}

} // End namespace control