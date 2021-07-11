//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/26/21.
//

#ifndef NIFCOMMON_TYPES_H
#define NIFCOMMON_TYPES_H

#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "constants.h"
#include "nav_msgs/msg/odometry.hpp"

#include "nif_msgs/msg/autonomy_status.hpp"
#include "nif_msgs/msg/health.hpp"
#include "nif_msgs/msg/perception3_d.hpp"
#include "nif_msgs/msg/perception3_d_array.hpp"
#include "nif_msgs/msg/powertrain_status.hpp"
#include "nif_msgs/msg/system_status.hpp"
#include "nif_msgs/msg/terrain_status.hpp"
#include "nif_msgs/msg/waypoints.hpp"
#include "nif_msgs/msg/waypoints_array.hpp"

namespace nif {
namespace common {
namespace msgs {

/**
 * This message contains the odometry (pose + twist) information which should be updated by the
 * localization node.
 */
using Odometry = nav_msgs::msg::Odometry;

/**
 * This message contains the terrain information which should updated in the
 * terrain manager. (e.g. Frictions, back angle)
 */
using TerrainState = nif_msgs::msg::TerrainStatus;

/**
 * This message contains the raptor status information which comes from the
 * raptor computer in the racing vehicle.
 * TODO: should be changed based on the bag file
 */
//using RaptorState = raptor_dbw_msgs::msg::;

/**
 * This message contains the race flag information from the race control.
 * TODO: should be changed based on the bag file
 */
using RaceControlState = autoware_auto_msgs::msg::VehicleKinematicState;

/**
 * This message contains the autonomy status information which should updated
 * in the ??.
 */
using AutonomyState = nif_msgs::msg::AutonomyStatus;

/**
 * This message contains the system status information which should updated in
 * the system status monitor node. It contains the Autonomy status and Health
 * status of the every node.
 */
using SystemState = nif_msgs::msg::SystemStatus;

/**
 * This message contains the health status of the node which should updated in
 * the system status monitor node.
 */
using Health = nif_msgs::msg::Health;

/**
 * This message contains the perception result which should updated
 * in the perception node. It contains the class, id, score, 3d position and
 * prediction result.
 */
using PerceptionResult = nif_msgs::msg::Perception3D;

/**
 * This message contains the list of perception result which should updated
 * in the perception node. It composed with the list of PerceptionResult.
 */
using PerceptionResultList = nif_msgs::msg::Perception3DArray;

/**
 * This message contains the vehicle powertrain data which come from the
 * vehicle. It should be updated in the BaseNode using Raptor message.
 */
using PowertrainState = nif_msgs::msg::PowertrainStatus;

/**
 * This message contains the truncated waypoints and the current index. It
 * should be updated in the waypoint mananger.
 */
using WaypointState = nif_msgs::msg::Waypoints;

/**
 * This message contains the list of truncated waypoints and the current index
 * regarding to the multiple racing lines. It should be updated in the
 * waypoint mananger.
 */
using WaypointStateList = nif_msgs::msg::WaypointsArray;

// TODO: replace with real polynomial!
using Polynomial = nav_msgs::msg::Odometry;
using ControlCmd = std_msgs::msg::Header;

using Trajectory = autoware_auto_msgs::msg::Trajectory;
// using VehicleKinematicState =
// autoware_auto_msgs::msg::VehicleKinematicState;
} // namespace msgs

namespace types {

template <typename T>
using t_oppo_collection =
    std::array<T, nif::common::constants::NUMBER_OF_OPPO_MAX>;

using t_oppo_collection_states = t_oppo_collection<nif::common::msgs::PerceptionResult>;

} // namespace types
} // namespace common
} // namespace nif
#endif // NIFCOMMON_TYPES_H
