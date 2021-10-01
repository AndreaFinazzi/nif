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

#ifndef _TRACTION_ABS_H_
#define _TRACTION_ABS_H_

namespace control {
/// @class Traction Control & Anti-lock Braking System
/// @brief

class TractionABS {
public:
  /// @brief Constructor
  /// @param[in] ABS_enabled
  /// @param[in] velocity_thres_mps
  /// @param[in] brake_cmd_thres
  /// @param[in] sigma_thres
  /// @param[in] ABS_factor
  /// @param[in] control_rate
  /// @param[in] ABS_rate

  TractionABS(const bool &ABS_enabled = true,
              const double &velocity_thres_mps = 27.78,
              const double &brake_cmd_thres = 0.5,
              const double &sigma_thres = 0.06, const double &ABS_factor = 0.5,
              const double &control_rate = 100., const double &ABS_rate = 15.);

  /// @brief ABS parameters
  int cnt = 0;

  double ABS_control(double brake_cmd, double current_velocity, double sigma);

private:
  bool ABS_enabled_ = true;
  double velocity_thres_mps_ = 27.78;
  double brake_cmd_thres_ = 0.5;
  double sigma_thres_ = 0.06;
  double ABS_factor_ = 0.5;
  double control_rate_ = 100.0;
  double ABS_rate_ = 15.;

}; // End class TractionABS

} // End namespace control

#endif // End _TRACTION_ABS_H_