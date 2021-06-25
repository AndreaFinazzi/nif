//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_I_PLANNER_ALGORITHM_H
#define ROS2MASTER_I_PLANNER_ALGORITHM_H

#include "autoware_auto_msgs/msg/trajectory.hpp"

namespace nif {
namespace planning {
namespace algorithms {

class IPlannerAlgorithm {
public:

protected:

private:
    EgoVehicleState vehicle_state_prev;
    Collection <OppoVehicleState> opponents_state;
    TrackState track_state;

    // TODO: Will it be static or not?
    GRacingLineManager g_racing_line_manager;
    VehicleModel vehicle_model;
    BehaviorState behavior_state;

    virtual autoware_auto_msgs::msg::Trajectory solve() = 0;
};
}
}
}


#endif //ROS2MASTER_I_PLANNER_ALGORITHM_H
