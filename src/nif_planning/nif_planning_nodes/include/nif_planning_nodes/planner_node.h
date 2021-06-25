//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_PLANNER_NODE_H
#define ROS2MASTER_PLANNER_NODE_H

#include "nif_common_nodes/base_node.h"
#include "autoware_auto_msgs/msg/trajectory.hpp"

namespace nif {
namespace planning {

class PlannerNode : public nif::common::IBaseNode {

public:
    PlannerNode(nif::planning::algorithms::IPlannerAlgorithm & planner_algorithm_): planner_algorithm(planner_algorithm_);

protected:
    rclcpp::Subscription<Collection<OppoVehicleState>> opponents_state_sub;

    rclcpp::Subscription<TrackState> track_state_sub;

    rclcpp::Publisher<autoware_auto_msgs::msg::Trajectory> trajectory_pub;

    nif::planning::algorithms::IPlannerAlgorithm & planner_algorithm;


    void opponentsStateCallback(const Collection<OppoVehicleState>::SharedPtr & msg);

    void trackStateCallback(const TrackState::SharedPtr & msg);


private:

};

}
}

#endif //ROS2MASTER_PLANNER_NODE_H
