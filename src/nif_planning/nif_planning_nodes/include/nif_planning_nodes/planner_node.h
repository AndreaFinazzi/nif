//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_PLANNER_NODE_H
#define ROS2MASTER_PLANNER_NODE_H

#include "nif_common/types.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_planning_algorithms/i_planner_algorithm.h"

#include "autoware_auto_msgs/msg/trajectory.hpp"

namespace nif {
namespace planning {

class PlannerNode : public nif::common::IBaseNode {

public:
    PlannerNode(nif::planning::algorithms::IPlannerAlgorithm & planner_algorithm_);

protected:
    /**
     * Subscribtion to opponents vehicles' states.
     */
    rclcpp::Subscription<nif::common::t_oppo_collection_states> opponents_state_sub;


    rclcpp::Subscription<nif::common::TrackState> track_state_sub;

    rclcpp::Publisher<autoware_auto_msgs::msg::Trajectory> trajectory_pub;

    nif::planning::algorithms::IPlannerAlgorithm & planner_algorithm;


    void opponentsStateCallback(const nif::common::t_oppo_collection_states & msg);

    void trackStateCallback(const nif::common::TrackState::SharedPtr & msg);


private:

};

}
}

#endif //ROS2MASTER_PLANNER_NODE_H
