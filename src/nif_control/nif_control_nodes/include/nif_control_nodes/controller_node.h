//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_CONTROL_NODE_H
#define ROS2MASTER_CONTROL_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "nif_common_nodes/base_synchronized_node.h"
#include "autoware_auto_msgs/msg/trajectory.hpp"

namespace nif {
namespace control {

class IControllerNode : public nif::common::IBaseSynchronizedNode {

public:

protected:
    /**
     * Received by subscribed planner
     */
    autoware_auto_msgs::msg::Trajectory::SharedPtr reference_trajectory;

    /**
     * Actual cmd fed to the vehicle at the last iteration.
     */
    ControlCmd::SharedPtr control_cmd_prev;

private:
    /**
     * Store the last trajectory computed by the subscribed planner. It's called by the subscription callback, and it can be customized.
     * @param trajectory
     */
    virtual void storeTraj(autoware_auto_msgs::msg::Trajectory::SharedPtr trajectory);

    rclcpp::Subscription<autoware_auto_msgs::msg::Trajectory>::SharedPtr planner_sub;
    rclcpp::Subscription<ControlCmd>::SharedPtr control_cmd_prev_sub;
    rclcpp::Publisher<ControlCmd>::SharedPtr control_cmd_pub;

    void controlCmdPrevCallback(const ControlCmd::SharedPtr & msg);

    void plannerCallback(const autoware_auto_msgs::msg::Trajectory::SharedPtr & msg);

    virtual ControlCmd::SharedPtr solve() = 0;
};

}}
#endif //ROS2MASTER_CONTROL_NODE_H
