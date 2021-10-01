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

#include "ABS.hpp"
#include <cmath>
#include <iostream>

namespace control {
ABS::ABS(const bool &ABS_enabled, const double &velocity_thres_mps,
         const double &brake_cmd_thres, const double &sigma_thres,
         const double &ABS_factor, const double &control_rate,
         const double &ABS_rate) {
  // Set control parameters
  ABS_enabled_ = ABS_enabled;
  velocity_thres_mps_ = velocity_thres_mps;
  brake_cmd_thres_ = brake_cmd_thres;
  sigma_thres_ = sigma_thres;
  ABS_factor_ = ABS_factor;
  control_rate_ = control_rate;
  ABS_rate_ = ABS_rate;
}

double ABS::ABS_control(double brake_cmd, double current_velocity,
                        double sigma) {
  // Init
  double brake_cmd_new = brake_cmd;
  // Tire slip ratio
  // - assume 'vx == free wheel speed'
  // - sigma = (braking_wheelspeed - rolling_wheelspeed) /

  // ABS activation condition
  bool active_ABS =
      (ABS_enabled_ == true) && (current_velocity > velocity_thres_mps_) &&
      (brake_cmd > brake_cmd_thres_) && (std::abs(sigma) > sigma_thres_);
  // ABS algorithm
  if (active_ABS) {
    // check count for ABS activation timing;
    if (cnt % int(int(control_rate_) % int(ABS_rate_))) {
      // Release brake command
      brake_cmd_new = ABS_factor_ * brake_cmd;
    }
  }

  // ABS counter
  cnt++;
  if (cnt > 255)
    cnt = 0;

  return brake_cmd_new;
}

} // End namespace control