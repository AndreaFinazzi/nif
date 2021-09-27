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

#include <nif_wall_following_controller/kin_control_node.hpp>

using nif::control::KinControl;

KinControl::KinControl() : Node("kin_controller_for_wall_following") {

  lookahead_error = 0.0;
  lat_error = 0.0;
  steering_cmd = 0.0;
}

void KinControl::run()
{
  rclcpp::Time control_time = rclcpp::Clock().now();
  rclcpp::Duration time_diff = control_time - this->recv_time_;
  double dt = static_cast<double>(time_diff.seconds()) + static_cast<double>(time_diff.nanoseconds())*1e-9;

  if ( dt < 0.5)
  {
    calculateFFW();
    calculateFB();
    calculateSteeringCmd();
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "%s\n", "Have not received new path in > 0.5s !");
    setCmdsToZeros();
  }

}

void KinControl::calculateFFW()
{
  // Desired yaw rate from feedforward
  this->feedforward_ = 0.0; // can add feedforward with a curvature: this->speed_ * this->curvature_;  
}

void KinControl::calculateFB()
{
  // Desired yaw rate from feedback
  this->feedback_ = -Kp * this->lookahead_error;
}

void KinControl::calculateSteeringCmd()
{
  // Convert desired yaw rate to steering angle using kinematic model
  this->steering_cmd = (this->speed_ > 1.0) ? L * (this->feedback_ + this->feedforward_) / this->speed_ : L * (this->feedback_ + this->feedforward_);
  this->steering_cmd = this->steering_cmd*57.296*19.0/9.0; // times 19.0/9 because ratio is wrong


  // steering command saturator
  if (this->steering_cmd > ms) {
    // RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Max");
    this->steering_cmd = ms;
  }
  if (this->steering_cmd < -ms) {
    // RCLCPP_DEBUG(this->get_logger(), "%s\n", "Saturation Limit Min");
    this->steering_cmd = -ms;
  }

  // RCLCPP_INFO(this->get_logger(), "cmd : %f", this->steering_cmd);
  // RCLCPP_INFO(this->get_logger(), "lookahead_error : %f",
  //             this->lookahead_error);
  // RCLCPP_INFO(this->get_logger(), "lateral_error : %f", this->lat_error);
}

void KinControl::setCmdsToZeros()
{
  this->feedforward_ = 0.0;
  this->feedback_ = 0.0;
  this->lookahead_error = 0.0;
  this->curvature_ = 0.0;
  this->speed_ = 0.0;

  this->steering_cmd = 0.0;
}


int KinControl::findLookaheadIndex(std::vector<geometry_msgs::msg::PoseStamped> refPath, double desLookaheadValue)
{
  // Finds first path pose that has x distance > lookahead distance
  for (long unsigned int i=0; i< refPath.size(); i++)
  {
    if (refPath[i].pose.position.x >= desLookaheadValue)
    {
      return i;
    }
  }
  return refPath.size()-1;
}

void KinControl::setVelocity(const double &SpeedMps) {
  this->speed_ = SpeedMps; // average wheel speeds (kph) and convert to m/s
  // RCLCPP_INFO(this->get_logger(), "speed: %f", this->speed_);
}

void KinControl::setPath(const nav_msgs::msg::Path &pathIn){
  // Determines lookahead distance based on speed and bounds
  double lookahead_distance =
      std::max(min_la, std::min(max_la, this->speed_ * la_ratio));

  // Unpacks the message and finds the index correlated to the lookahead
  // distance
  std::vector<geometry_msgs::msg::PoseStamped> path = pathIn.poses;
  int idx = findLookaheadIndex(path, lookahead_distance);

  // RCLCPP_INFO(this->get_logger(), "lookahead_distance: %f", lookahead_distance);

  // Sets the lookahead and lateral error
  if (path.size() > 0) {
    this->lookahead_error = path[idx].pose.position.y;
    this->lat_error = path[0].pose.position.y;
  } else {
    this->lookahead_error = 0.0;
  }

  this->recv_time_ = rclcpp::Clock().now();
}

void KinControl::getPredictivePath(nav_msgs::msg::Path &pathOut)
{
  
}

void KinControl::getSteering(double &SpeedMps) { SpeedMps = - this->steering_cmd;}