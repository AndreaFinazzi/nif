//
// Created by usrg on 6/26/21.
//

#ifndef NIFCOMMON_TYPES_H
#define NIFCOMMON_TYPES_H

#include "constants.h"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nif {
    namespace common {
//        TODO : THIS IS WRONG AND MUST BE CHANGED -- START --
        typedef autoware_auto_msgs::msg::VehicleKinematicState TrackState;
        typedef autoware_auto_msgs::msg::VehicleKinematicState SystemState;
        typedef autoware_auto_msgs::msg::VehicleKinematicState RaceControlState;

        // TODO: replace with real polynomial!
        typedef nav_msgs::msg::Odometry Polynomial;
        typedef nav_msgs::msg::Odometry ControlCmd;
//        TODO : THIS IS WRONG AND MUST BE CHANGED -- END --

        typedef autoware_auto_msgs::msg::VehicleKinematicState t_vehicle_kinematic_state;

        template<typename T>
        using t_oppo_collection = std::array<T, nif::common::NUMBER_OF_OPPO_MAX>;

        typedef t_oppo_collection<t_vehicle_kinematic_state> t_oppo_collection_states;

    }
}
#endif //NIFCOMMON_TYPES_H
