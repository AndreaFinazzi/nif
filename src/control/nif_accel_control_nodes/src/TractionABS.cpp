/* Copyright 2021 Hyunki Seong

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

#include "TractionABS.hpp"
#include <cmath>
#include <iostream>

namespace control {
TractionABS::TractionABS(const bool &traction_enabled,
                         const double &traction_throttle_cmd_thres,
                         const double &traction_factor,
                         const double &traction_rate, const bool &ABS_enabled,
                         const double &ABS_brake_cmd_thres,
                         const double &ABS_factor, const double &ABS_rate,
                         const double &velocity_thres_mps,
                         const double &sigma_thres,
                         const double &control_rate) {
  // Set control parameters
  // - enabling Traction/ABS
  traction_enabled_ = traction_enabled;
  ABS_enabled_ = ABS_enabled;
  // - throttle/brake cmd threshold for ABS activation. 0.5
  traction_throttle_cmd_thres_ = traction_throttle_cmd_thres;
  ABS_brake_cmd_thres_ = ABS_brake_cmd_thres;
  // - decreasing throttle/brake cmd factor. 0.5
  traction_factor_ = traction_factor;
  ABS_factor_ = ABS_factor;
  // - control rate of Traction/ABS activagion
  traction_rate_ = traction_rate;
  ABS_rate_ = ABS_rate;
  // - velocity threshold for ABS activation.
  velocity_thres_mps_ = velocity_thres_mps;
  // - slip ratio threshold. 0.06
  sigma_thres_ = sigma_thres;
  // - control rate of low-level accel controller
  control_rate_ = control_rate;
}

double TractionABS::tractionControl(double throttle_cmd,
                                    double current_velocity, double sigma) {
  // Init
  double throttle_cmd_new = throttle_cmd;
  // Tire slip ratio
  // - assume 'vx == free wheel speed'
  // - sigma = (braking_wheelspeed - rolling_wheelspeed) /

  // Traction Control activation condition
  bool active_traction = (traction_enabled_ == true) &&
                         (current_velocity > velocity_thres_mps_) &&
                         (throttle_cmd > traction_throttle_cmd_thres_) &&
                         (std::abs(sigma) > sigma_thres_);
  // Traction Control algorithm
  if (active_traction) {
    // check count for Traction Control activation timing;
    int cnt_condition =
        cnt_traction % int(int(control_rate_) % int(traction_rate_));
    if (cnt_condition == 0) {
      // Release brake command
      throttle_cmd_new = traction_factor_ * throttle_cmd;
      std::cout << "Traction Control is activated!!!!!!!!!!!!!!!!!!!!!!"
                << std::endl;
    }
  }

  // Traction Control counter
  cnt_traction++;
  if (cnt_traction > 255)
    cnt_traction = 0;

  return throttle_cmd_new;
}

double TractionABS::ABSControl(double brake_cmd, double current_velocity,
                               double sigma) {
  // Init
  double brake_cmd_new = brake_cmd;
  // Tire slip ratio
  // - assume 'vx == free wheel speed'
  // - sigma = (braking_wheelspeed - rolling_wheelspeed) /

  // ABS activation condition
  bool active_ABS =
      (ABS_enabled_ == true) && (current_velocity > velocity_thres_mps_) &&
      (brake_cmd > ABS_brake_cmd_thres_) && (std::abs(sigma) > sigma_thres_);
  // ABS algorithm
  if (active_ABS) {
    // check count for ABS activation timing;
    int cnt_condition = cnt_ABS % int(int(control_rate_) % int(ABS_rate_));
    if (cnt_condition == 0) {
      // Release brake command
      brake_cmd_new = ABS_factor_ * brake_cmd;
      std::cout << "ABS is activated!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
    }
  }

  // ABS counter
  cnt_ABS++;
  if (cnt_ABS > 255)
    cnt_ABS = 0;

  return brake_cmd_new;
}

} // End namespace control