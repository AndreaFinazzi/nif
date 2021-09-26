//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_CONTROL_NODE_H
#define ROS2MASTER_CONTROL_NODE_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_synchronized_node.h"
#include <nav_msgs/msg/odometry.hpp>

#include "rclcpp/rclcpp.hpp"

namespace nif {
namespace control {

class IControllerNode : public nif::common::IBaseSynchronizedNode {

public:

  explicit IControllerNode(const std::string &node_name);

protected:
  void run() final;

  const nif::common::msgs::Trajectory::SharedPtr &getReferenceTrajectory() const;
  const nif::common::msgs::ControlCmd::SharedPtr &getControlCmdPrev() const;
  const nif::common::msgs::Path::SharedPtr &getReferencePath() const;  
  const std_msgs::msg::Float32::SharedPtr &getDesiredVelocity() const;
  const rclcpp::Time &getReferenceTrajectoryUpdateTime() const;
  const rclcpp::Time &getControlCmdPrevUpdateTime() const;
  const rclcpp::Time &getReferencePathUpdateTime() const;  
  const rclcpp::Time &getDesiredVelocityUpdateTime() const;
  bool hasReferenceTrajectory() const;
  bool hasReferencePath() const;
  bool hasControlCmdPrev() const;  
  bool hasDesiredVelocity() const;

private:
  IControllerNode();

  /**
   * Trajectory received by the subscribed motion planner
   */
  nif::common::msgs::Trajectory::SharedPtr reference_trajectory;
  bool has_reference_trajectory = false;
  /**
   * reference_trajectory last update-time.
   */
  rclcpp::Time reference_trajectory_update_time;

  /**
  * Path received by subscribed path planner
  */
  nif::common::msgs::Path::SharedPtr reference_path;
  bool has_reference_path = false;

  /**
  * reference_path last update-time.
  */
  rclcpp::Time reference_path_update_time;


  /**
   * Actual cmd fed to the vehicle at the last iteration.
   */
  nif::common::msgs::ControlCmd::SharedPtr control_cmd_prev;
  bool has_control_cmd_prev = false;

  /**
   * control_cmd_prev last update-time.
   */
  rclcpp::Time control_cmd_prev_update_time;

  /**
   * Desired velocity fed by a velocity planner.
   */
  std_msgs::msg::Float32::SharedPtr desired_velocity;
  bool has_desired_velocity = false;

  /**
   * control_cmd_prev last update-time.
   */
  rclcpp::Time desired_velocity_update_time;


  rclcpp::Subscription<nif::common::msgs::Trajectory>::SharedPtr
      reference_trajectory_sub;
  rclcpp::Subscription<nif::common::msgs::Path>::SharedPtr
    reference_path_sub;
  rclcpp::Subscription<nif::common::msgs::ControlCmd>::SharedPtr
      control_cmd_prev_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      desired_velocity_sub;

  rclcpp::Publisher<nif::common::msgs::ControlCmd>::SharedPtr control_cmd_pub;

  void controlCmdPrevCallback(nif::common::msgs::ControlCmd::SharedPtr msg);
  void referenceTrajectoryCallback(nif::common::msgs::Trajectory::SharedPtr msg);
  void referencePathCallback(nif::common::msgs::Path::SharedPtr msg);
  void desiredVelocityCallback(std_msgs::msg::Float32::SharedPtr msg);

  /**
   * It's called at the end of referenceTrajectoryCallback(...)
   * and it can be customized.
   * @param trajectory
   */
  virtual void afterReferenceTrajectoryCallback() {}

  /**
  * It's called at the end of referencePathCallback(...)
  * and it can be customized.
  * @param path
  */
  virtual void afterReferencePathCallback() {}

  /**
  * It's called at the end of controlCmdPrevCallback(...)
  * and it can be customized.
  * @param path
  */
  virtual void afterControlCmdPrevCallback() {}

  /**
  * It's called at the end of desiredVelocityCallback(...)
  * and it can be customized.
  * @param path
  */
  virtual void afterDesiredVelocityCallback() {}


  /**
   * slove() is called at each time-step and expects the next control command to be returned.
   * The returned message is automatically stamped by IControllerNode, to avoid empty stamp errors.
   *
   * @return the control_command to be published.
   */
  virtual nif::common::msgs::ControlCmd::SharedPtr solve() = 0;
};

} // namespace control
} // namespace nif
#endif // ROS2MASTER_CONTROL_NODE_H
