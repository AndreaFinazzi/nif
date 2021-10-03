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
  /// @param[in] traction_enabled
  /// @param[in] traction_throttle_cmd_thres
  /// @param[in] traction_factor
  /// @param[in] traction_rate
  /// @param[in] ABS_enabled
  /// @param[in] ABS_brake_cmd_thres
  /// @param[in] ABS_factor
  /// @param[in] ABS_rate
  /// @param[in] velocity_thres_mps
  /// @param[in] sigma_thres
  /// @param[in] control_rate

  TractionABS(const bool &traction_enabled = true,
              const double &traction_throttle_cmd_thres = 0.5,
              const double &traction_factor = 0.5,
              const double &traction_rate = 15.0,
              const bool &ABS_enabled = true,
              const double &ABS_brake_cmd_thres = 0.5,
              const double &ABS_factor = 0.5, const double &ABS_rate = 15.0,
              const double &velocity_thres_mps = 27.78,
              const double &sigma_thres = 0.06,
              const double &control_rate = 100.);

  /// @brief Parameters
  int cnt_traction = 0;
  int cnt_ABS = 0;

  /// variables for diagnostic
  double m_traction_activated = 0;
  double m_abs_activated = 0;

  double tractionControl(double throttle_cmd, double current_velocity,
                         double sigma);
  double ABSControl(double brake_cmd, double current_velocity, double sigma);

private:
  bool traction_enabled_ = true;
  double traction_throttle_cmd_thres_ = 0.5;
  double traction_factor_ = 0.5;
  double traction_rate_ = 15.;

  bool ABS_enabled_ = true;
  double ABS_brake_cmd_thres_ = 0.5;
  double ABS_factor_ = 0.5;
  double ABS_rate_ = 15.;

  double velocity_thres_mps_ = 27.78;
  double sigma_thres_ = 0.06;
  double control_rate_ = 100.0;

}; // End class TractionABS

} // End namespace control

#endif // End _TRACTION_ABS_H_