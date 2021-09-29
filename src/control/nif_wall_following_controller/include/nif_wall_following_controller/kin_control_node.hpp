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


 *  Modified on: Sep 27, 2021
 *       Author: Daegyu Lee
*/

#ifndef KIN_CONTROL_HPP
#define KIN_CONTROL_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <math.h>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "deep_orange_msgs/msg/joystick_command.hpp"

namespace nif {
namespace control {

class KinControl : public rclcpp::Node
{
  public:
    KinControl();

    void setCmdsToZeros();
    void setVelocity(const double &SpeedMps);
    void setPath(const nav_msgs::msg::Path &pathIn);
    void getSteering(double &SpeedMps);
    void getPredictivePath(nav_msgs::msg::Path &pathOut);
    void run();

    double steering_cmd;
    double lookahead_error;
    double lat_error;

    //params
    double min_la;
    double max_la;
    double la_ratio;
    bool auto_enabled;
    double ms;
    double steer_override_threshold ;
    double L;
    double Kp;

    rclcpp::Time recv_time_;

  private:
    void calculateFFW();
    void calculateFB();
    void calculateSteeringCmd();
    void publishSteering();
    void publishDebugSignals();
    int findLookaheadIndex(std::vector<geometry_msgs::msg::PoseStamped> refPath, double desLookaheadValue);

    double curvature_ = 0.0;
    double speed_ = 0.0;
    double feedforward_ = 0.0;
    double feedback_ = 0.0;
    double steering_override_ = 0.0;

}; // end of KinControl class

} // namespace control
} // namespace nif

#endif
