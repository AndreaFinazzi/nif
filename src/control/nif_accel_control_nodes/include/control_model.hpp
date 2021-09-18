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

#ifndef _CONTROL_MODEL_H_
#define _CONTROL_MODEL_H_

#include <utility>

namespace control {
/// @class AccelController
/// @brief
class AccelController {
public:
  /// @brief Constructor
  /// @param[in] K_accel
  /// @param[in] K_accel2
  /// @param[in] K_bias
  /// @param[in] pedalToCmd
  /// @param[in] cmdMin is a lower bound on the output command. Defaults to
  /// -1e8.
  AccelController(const double &K_accel = 0.0, const double &K_accel2 = 0.0,
                  const double &K_bias = 0.0, const double &pedalToCmd = 0.0,
                  const double &dt = 0.01, const double &cmdMax = 1e8,
                  const double &cmdMin = -1e8);

  /// @brief Input the current error and get back the current control value
  /// @param newError is the most recently calculated error value
  /// @return the control value from the PID algorithm
  double Update(const double &newError);

  /// @brief Get the current control value (calculated based on most recent
  /// errors)
  /// @return the current control value
  double CurrentControl(double des_accel);

  /// @brief Set the minimum and maximum bounds on the command output
  /// @param min is the minimum command output. Must be less than or equal to
  /// zero.
  /// @param max is the maximum command output. Must be greater than zero.
  /// @return a pair of bools. First is true if min is set correctly, second is
  /// true if max is set correctly.
  std::pair<bool, bool> SetCmdBounds(const double &min, const double &max);

private:
  /// @brief Saturate the command value based on the max and min limits
  double SaturateCmd(const double &cmd);

  /// @brief Controller parameters
  double k_accel_ = 0.0;
  double k_accel2_ = 0.0;
  double k_bias_ = 0.0;
  double pedalToCmd_ = 0.0;

  /// @brief Maximum output command. Must be greater than zero.
  double cmdMax_ = 1e8;
  /// @brief Minimum output command. Must be less than or equal to zero.
  double cmdMin_ = -1e8;

}; // End class AccelController

} // End namespace control

#endif // End _CONTROL_MODEL_H_