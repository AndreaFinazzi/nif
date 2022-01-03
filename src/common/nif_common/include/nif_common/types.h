//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/26/21.
//

#ifndef NIFCOMMON_TYPES_H
#define NIFCOMMON_TYPES_H

#include "constants.h"

#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "radar_msgs/msg/radar_detection_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "deep_orange_msgs/msg/joystick_command.hpp"
#include "deep_orange_msgs/msg/rc_to_ct.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"
#include "deep_orange_msgs/msg/base_to_car_summary.hpp"

#include "nif_msgs/msg/system_health_status.hpp"
#include "nif_msgs/msg/autonomy_status.hpp"
#include "nif_msgs/msg/control_command.hpp"
#include "nif_msgs/msg/dynamic_trajectory.hpp"
#include "nif_msgs/msg/mission_status.hpp"
#include "nif_msgs/msg/node_status.hpp"
#include "nif_msgs/msg/perception3_d.hpp"
#include "nif_msgs/msg/perception3_d_array.hpp"
#include "nif_msgs/msg/powertrain_status.hpp"
#include "nif_msgs/msg/system_health_status.hpp"
#include "nif_msgs/msg/system_status.hpp"
#include "nif_msgs/msg/terrain_status.hpp"
#include "nif_msgs/msg/trajectory.hpp"
#include "nif_msgs/msg/waypoints.hpp"
#include "nif_msgs/msg/waypoints_array.hpp"
#include "nif_msgs/msg/trajectory.hpp"
#include "nif_msgs/msg/dynamic_trajectory.hpp"

namespace nif {
namespace common {

enum SystemStatusCode : std::uint8_t {
  SYSTEM_OK = nif_msgs::msg::SystemHealthStatus::SYSTEM_OK,
  SYSTEM_INITIALIZED = nif_msgs::msg::SystemHealthStatus::SYSTEM_INITIALIZED,
  SYSTEM_NOT_INITIALIZED = nif_msgs::msg::SystemHealthStatus::SYSTEM_NOT_INITIALIZED,
  SYSTEM_ERROR = nif_msgs::msg::SystemHealthStatus::SYSTEM_ERROR,
  SYSTEM_LOW_LEVEL_ERROR = nif_msgs::msg::SystemHealthStatus::SYSTEM_LOW_LEVEL_ERROR,
  SYSTEM_FATAL_ERROR = nif_msgs::msg::SystemHealthStatus::SYSTEM_FATAL_ERROR,
  SYSTEM_DEAD = nif_msgs::msg::SystemHealthStatus::SYSTEM_DEAD
};

enum NodeStatusCode : std::uint8_t {
  NODE_NOT_INITIALIZED = nif_msgs::msg::NodeStatus::NODE_NOT_INITIALIZED,
  NODE_INITIALIZED = nif_msgs::msg::NodeStatus::NODE_INITIALIZED,
  NODE_OK = nif_msgs::msg::NodeStatus::NODE_OK,
  NODE_INACTIVE = nif_msgs::msg::NodeStatus::NODE_INACTIVE,
  NODE_ERROR = nif_msgs::msg::NodeStatus::NODE_ERROR,
  NODE_FATAL_ERROR = nif_msgs::msg::NodeStatus::NODE_FATAL_ERROR,
  NODE_DEAD = nif_msgs::msg::NodeStatus::NODE_DEAD
};

enum NodeType : std::int8_t {
  PERCEPTION,
  LOCALIZATION,
  PLANNING,
  PREDICTION,
  CONTROL,
  TOOL,
  SYSTEM
};

static bool isNodeTypeInRange(nif::common::NodeType type_id) {
  if (type_id == common::NodeType::SYSTEM ||
      type_id == common::NodeType::TOOL ||
      type_id == common::NodeType::PERCEPTION ||
      type_id == common::NodeType::LOCALIZATION ||
      type_id == common::NodeType::PREDICTION ||
      type_id == common::NodeType::PLANNING ||
      type_id == common::NodeType::CONTROL)
    return true;
  return false;
}

namespace msgs {

/**
 * This message contains the odometry (pose + twist) information which should be
 * updated by the localization node.
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
// using RaptorState = raptor_dbw_msgs::msg::;

/**
 * This message contains the race flag information from the race control.
 * TODO: should be changed based on the bag file
 */
using RCFlagSummary = deep_orange_msgs::msg::BaseToCarSummary;
using OverrideRCFlagSummary = deep_orange_msgs::msg::BaseToCarSummary;

static bool isTrackFlagInRange(
    nif::common::msgs::RCFlagSummary::_track_flag_type track_flag) {
  if (track_flag == RCFlagSummary::TRACK_FLAG_NULL ||
      track_flag == RCFlagSummary::TRACK_FLAG_RED ||
      track_flag == RCFlagSummary::TRACK_FLAG_ORANGE ||
      track_flag == RCFlagSummary::TRACK_FLAG_YELLOW ||
      track_flag == RCFlagSummary::TRACK_FLAG_GREEN ||
      track_flag == RCFlagSummary::TRACK_FLAG_BLUE ||
      track_flag == RCFlagSummary::TRACK_FLAG_WAVING_GREEN)
    return true;
  return false;
}

static bool
isVehFlagInRange(nif::common::msgs::RCFlagSummary::_veh_flag_type veh_flag) {
  if (veh_flag == RCFlagSummary::VEH_FLAG_NULL ||
      veh_flag == RCFlagSummary::VEH_FLAG_BLANK ||
      veh_flag == RCFlagSummary::VEH_FLAG_BLACK ||
      veh_flag == RCFlagSummary::VEH_FLAG_CHECKERED ||
      veh_flag == RCFlagSummary::VEH_FLAG_DEFENDER ||
      veh_flag == RCFlagSummary::VEH_FLAG_PINK ||
      veh_flag == RCFlagSummary::VEH_FLAG_ATTACKER ||
      veh_flag == RCFlagSummary::VEH_FLAG_PURPLE)
    return true;
  return false;
}

/**
* This message contains the autonomy status information which should updated
* in the ??.
*/
using AutonomyState = nif_msgs::msg::AutonomyStatus;

/**
* This message contains the system status information which should be updated in
* the system status manager node. It contains the Autonomy status, the Health
* status and Mission status.
*/
using SystemStatus = nif_msgs::msg::SystemStatus;

/**
* This message contains the health status of the system.
*/
using SystemHealthStatus = nif_msgs::msg::SystemHealthStatus;

/**
* This message contains the health status of the node which should updated in
* the system status monitor node.
*/
using MissionStatus = nif_msgs::msg::MissionStatus;

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
* This message contains a list of radar tracks.
*/
using RadarTrackList = radar_msgs::msg::RadarDetectionArray;

/**
* This message contains radar track information.
*/
using RadarTrack = radar_msgs::msg::RadarDetection;

/**
* This message contains the vehicle powertrain data which come from the
* vehicle. It should be updated in the BaseNode using Raptor message.
*/
using PowertrainState = deep_orange_msgs::msg::PtReport; // nif_msgs::msg::PowertrainStatus;

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

/**
* NodeStatus report message
*/
using NodeStatus = nif_msgs::msg::NodeStatus;

// TODO: replace with real polynomial!
using Polynomial = nav_msgs::msg::Odometry;

using OverrideControlCmd = deep_orange_msgs::msg::JoystickCommand;
using ControlCmd = nif_msgs::msg::ControlCommand;

using ControlAcceleratorCmd = std_msgs::msg::Float32;
using ControlBrakingCmd = std_msgs::msg::Float32;
using ControlGearCmd = std_msgs::msg::UInt8;
using ControlSteeringCmd = std_msgs::msg::Float32;

/**
 * Message produced by a Motion Planner
 */
//  using Trajectory = autoware_auto_msgs::msg::Trajectory;
using Trajectory = nif_msgs::msg::DynamicTrajectory;

/**
 * Message produced by a Path Planner
 */
using Path = nav_msgs::msg::Path;

// using VehicleKinematicState =
// autoware_auto_msgs::msg::VehicleKinematicState;

static bool isMissionCodeInRange(
    nif::common::msgs::MissionStatus::_mission_status_code_type mission_code) {
  if (mission_code == MissionStatus::MISSION_RACE ||
      mission_code == MissionStatus::MISSION_KEEP_POSITION ||
      mission_code == MissionStatus::MISSION_CONSTANT_SPEED ||
      mission_code == MissionStatus::MISSION_STANDBY ||
      mission_code == MissionStatus::MISSION_PIT_IN ||
      mission_code == MissionStatus::MISSION_PIT_STANDBY ||
      mission_code == MissionStatus::MISSION_PIT_OUT ||
      mission_code == MissionStatus::MISSION_PIT_TO_TRACK ||
      mission_code == MissionStatus::MISSION_SLOW_DRIVE ||
      mission_code == MissionStatus::MISSION_COMMANDED_STOP ||
      mission_code == MissionStatus::MISSION_EMERGENCY_STOP ||
      mission_code == MissionStatus::MISSION_COLLISION_AVOIDNACE ||
      mission_code == MissionStatus::MISSION_TIRE_WARMUP ||
      mission_code == MissionStatus::MISSION_TEST ||
      mission_code == MissionStatus::MISSION_INIT ||
      mission_code == MissionStatus::MISSION_PIT_INIT ||
      mission_code == MissionStatus::MISSION_DEFAULT)
    return true;
  return false;
}

} // namespace msgs

namespace types {

using t_node_id = uint16_t;

template <typename T>
using t_oppo_collection =
    std::array<T, nif::common::constants::NUMBER_OF_OPPO_MAX>;

using t_oppo_collection_states =
    t_oppo_collection<nif::common::msgs::PerceptionResult>;

using t_clock_period_ns = std::chrono::nanoseconds;
using t_clock_period_us = std::chrono::microseconds;
using t_clock_period_ms = std::chrono::milliseconds;

} // namespace types
} // namespace common
} // namespace nif
#endif // NIFCOMMON_TYPES_H
