//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_CONTROL_NODE_H
#define ROS2MASTER_CONTROL_NODE_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_synchronized_node.h"
#include <nav_msgs/msg/detail/odometry__struct.hpp>

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

private:
  IControllerNode();


  /**
   * Received by subscribed planner
   */
  nif::common::msgs::Trajectory::SharedPtr reference_trajectory;

  /**
   * Actual cmd fed to the vehicle at the last iteration.
   */
  nif::common::msgs::ControlCmd::SharedPtr control_cmd_prev;


  rclcpp::Subscription<nif::common::msgs::Trajectory>::SharedPtr
      reference_trajectory_sub;

  rclcpp::Subscription<nif::common::msgs::ControlCmd>::SharedPtr
      control_cmd_prev_sub;

  rclcpp::Publisher<nif::common::msgs::ControlCmd>::SharedPtr control_cmd_pub;

  void controlCmdPrevCallback(nif::common::msgs::ControlCmd::SharedPtr msg);

  void referenceTrajectoryCallback(nif::common::msgs::Trajectory::SharedPtr msg);

  /**
   * Store the last trajectory computed by the subscribed planner. It's called
   * by the subscription callback, and it can be customized.
   * @param trajectory
   */
  virtual void
  storeReferenceTrajectory(nif::common::msgs::Trajectory::SharedPtr msg) {};

  virtual nif::common::msgs::ControlCmd &solve() = 0;
};

} // namespace control
} // namespace nif
#endif // ROS2MASTER_CONTROL_NODE_H
