//
// Created by usrg on 6/26/21.
//

#ifndef NIFCOMMON_TYPES_H
#define NIFCOMMON_TYPES_H

#include "constants.h"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/odometry.hpp"

namespace nif {
    namespace common {
        namespace msgs {
//          TODO : THIS IS WRONG AND MUST BE CHANGED -- START --
            using TrackState = autoware_auto_msgs::msg::VehicleKinematicState;
            using SystemState = autoware_auto_msgs::msg::VehicleKinematicState;
            using RaceControlState = autoware_auto_msgs::msg::VehicleKinematicState;

            // TODO: replace with real polynomial!
            using Polynomial = nav_msgs::msg::Odometry;
            using ControlCmd = nav_msgs::msg::Odometry;
//          TODO : THIS IS WRONG AND MUST BE CHANGED -- END --

            using Trajectory = autoware_auto_msgs::msg::Trajectory;
            using VehicleKinematicState = autoware_auto_msgs::msg::VehicleKinematicState;
        }

        namespace types {

            template<typename T>
            using t_oppo_collection = std::array<T, nif::common::NUMBER_OF_OPPO_MAX>;

            using t_oppo_collection_states = t_oppo_collection<msgs::VehicleKinematicState>;

        }
    }
}
#endif //NIFCOMMON_TYPES_H