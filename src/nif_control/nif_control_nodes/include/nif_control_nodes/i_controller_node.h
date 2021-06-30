//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_CONTROL_NODE_H
#define ROS2MASTER_CONTROL_NODE_H

#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include "nif_common/types.h"
#include "nif_common_nodes/i_base_synchronized_node.h"

#include "rclcpp/rclcpp.hpp"


namespace nif {
namespace control {

class IControllerNode : public nif::common::IBaseSynchronizedNode {

public:

protected:
    /**
     * Received by subscribed planner
     */
    nif::common::msgs::Trajectory::SharedPtr reference_trajectory;

    /**
     * Actual cmd fed to the vehicle at the last iteration.
     */
    nif::common::msgs::ControlCmd::SharedPtr control_cmd_prev;

private:
    rclcpp::Subscription<nif::common::msgs::Trajectory>::SharedPtr planner_sub;
    rclcpp::Subscription<nif::common::msgs::ControlCmd>::SharedPtr control_cmd_prev_sub;
    rclcpp::Publisher<nif::common::msgs::ControlCmd>::SharedPtr control_cmd_pub;

    /**
     * Store the last trajectory computed by the subscribed planner. It's called by the subscription callback, and it can be customized.
     * @param trajectory
     */
    virtual void storeTraj(nif::common::msgs::Trajectory::SharedPtr trajectory);

    void controlCmdPrevCallback(const nif::common::msgs::ControlCmd::SharedPtr & msg);

    void plannerCallback(const nif::common::msgs::Trajectory::SharedPtr & msg);

    virtual nif::common::msgs::ControlCmd & solve() = 0;
};

}}
#endif //ROS2MASTER_CONTROL_NODE_H
