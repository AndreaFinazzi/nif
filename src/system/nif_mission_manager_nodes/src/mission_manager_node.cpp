//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_mission_manager_nodes/mission_manager_node.h"

using nif::system::MissionManagerNode;
using nif::common::msgs::RCFlagSummary;
using namespace std::chrono_literals;

MissionManagerNode::MissionManagerNode(
        const std::string &node_name)
        : IBaseSynchronizedNode(
            node_name, 
            nif::common::NodeType::SYSTEM, 
            std::chrono::milliseconds(50)) {
//    Initialize to false
    this->declare_parameter("timeout_rc_flag_summary_s", 10);
    this->declare_parameter("timeout_velocity_ms", 1500);

    this->declare_parameter("velocity.zero", 0.0);
    this->declare_parameter("velocity.max", 67.0);
    this->declare_parameter("velocity.pit_in", 8.0);
    this->declare_parameter("velocity.pit_out", 8.0);
    // this->declare_parameter("velocity.slow_drive", 15.0);
    this->declare_parameter("velocity.slow_drive", 8.0);
    this->declare_parameter("safeloc.threshold_stop", 40.0);
    this->declare_parameter("safeloc.threshold_slow_down", 20.0);
    this->declare_parameter("safeloc.velocity_slow_down_max", 22.2);
    this->declare_parameter("safeloc.velocity_slow_down_min", 8.0);

    this->declare_parameter("missions_file_path", "");

    long int timeout_rc_flag_summary_s = this->get_parameter("timeout_rc_flag_summary_s").as_int();
    long int timeout_velocity_ms = this->get_parameter("timeout_velocity_ms").as_int();
    if (timeout_rc_flag_summary_s <= 0 ||
        timeout_velocity_ms <= 0)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "timeouts must be greater than zero. Got timeout_rc_flag_summary_s: %d; timeout_velocity_ms: %d", timeout_rc_flag_summary_s, timeout_velocity_ms);
        throw std::range_error("Parameter out of range.");
    }

    this->timeout_rc_flag_summary = rclcpp::Duration(timeout_rc_flag_summary_s, 0);
    this->timeout_current_velocity = rclcpp::Duration(timeout_velocity_ms * 1000000);

    this->velocity_zero = this->get_parameter("velocity.zero").as_double();
    this->velocity_max = this->get_parameter("velocity.max").as_double();
    this->velocity_pit_in = this->get_parameter("velocity.pit_in").as_double();
    this->velocity_pit_out = this->get_parameter("velocity.pit_out").as_double();
    this->velocity_slow_drive = this->get_parameter("velocity.slow_drive").as_double();
    this->safeloc_threshold_stop = this->get_parameter("safeloc.threshold_stop").as_double();
    this->safeloc_threshold_slow_down = this->get_parameter("safeloc.threshold_slow_down").as_double();
    this->safeloc_velocity_slow_down_max = this->get_parameter("safeloc.velocity_slow_down_max").as_double();
    this->safeloc_velocity_slow_down_min = this->get_parameter("safeloc.velocity_slow_down_min").as_double();

    auto missions_file_path = this->get_parameter("missions_file_path").as_string();

    this->parameters_callback_handle = this->add_on_set_parameters_callback(
            std::bind(&MissionManagerNode::parametersCallback, this, std::placeholders::_1));

    // Subscribers
    // TODO define internal msg and proper conversion
    this->rc_flag_summary_sub = this->create_subscription<common::msgs::RCFlagSummary>(
            "rc_interface/rc_flag_summary", nif::common::constants::QOS_RACE_CONTROL,
            std::bind(&MissionManagerNode::RCFlagSummaryCallback, this, std::placeholders::_1));

    this->velocity_sub =
      this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
          "/raptor_dbw_interface/wheel_speed_report", nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&MissionManagerNode::velocityCallback, this,
                    std::placeholders::_1));

    //  Publishers
    this->mission_status_pub = this->create_publisher<nif::common::msgs::MissionStatus>(
            "out_mission_status", nif::common::constants::QOS_INTERNAL_STATUS
    );

    MissionParser::loadMissionsDescription(missions_file_path, this->missions_description);
    
    this->setNodeStatus(common::NodeStatusCode::NODE_INITIALIZED);
}

void MissionManagerNode::run() {
    auto node_status = common::NodeStatusCode::NODE_ERROR;
    if (!this->isDataOk())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Data timeout in MissionManagerNode::run();");
        this->mission_status_msg.mission_status_code = MissionStatus::MISSION_EMERGENCY_STOP;
        
    } else {
        // Mission encoding
        auto next_mission = this->getMissionStatusCode();
        this->mission_status_msg.mission_status_code = this->validateMissionTransition(next_mission);
        node_status = common::NodeStatusCode::NODE_OK;
    }
    this->mission_status_msg.max_velocity_mps = static_cast<MissionStatus::_max_velocity_mps_type>(
            this->getMissionMaxVelocityMps(this->mission_status_msg.mission_status_code));

    this->mission_status_pub->publish(this->mission_status_msg);
    this->setNodeStatus(node_status);
}

// TODO this is just a draft, MUST BE REFINED AND FINALIZED
MissionStatus::_mission_status_code_type MissionManagerNode::getMissionStatusCode()
{
  if (
      this->has_rc_flag_summary &&
      this->now() - this->rc_flag_summary_update_time < this->timeout_rc_flag_summary )
  {
    switch (this->rc_flag_summary.veh_flag)
    {
    case RCFlagSummary::VEH_FLAG_PURPLE:
      return MissionStatus::MISSION_EMERGENCY_STOP;
      break;

    case RCFlagSummary::VEH_FLAG_BLACK:
        switch (this->rc_flag_summary.track_flag)
        {
        case RCFlagSummary::TRACK_FLAG_RED: // TODO ASK TO RC WHAT SHOULD HAPPEN HERE
            if (is_system_startup)
            {
                return MissionStatus::MISSION_DEFAULT; // No missions on startup
            } else {
                return MissionStatus::MISSION_COMMANDED_STOP;
            }
            break;

        case RCFlagSummary::TRACK_FLAG_ORANGE:
            if (is_system_startup && this->missionIs(MissionStatus::MISSION_DEFAULT))
            {
                return MissionStatus::MISSION_PIT_INIT;

            } else if (this->missionIs(MissionStatus::MISSION_PIT_INIT)) {
                is_system_startup = false;
                return MissionStatus::MISSION_PIT_STANDBY;
    
            } else if (this->missionIs(MissionStatus::MISSION_INIT)) {
                is_system_startup = false;
                return MissionStatus::MISSION_STANDBY;

            } else if ( this->missionIs(MissionStatus::MISSION_STANDBY) ||
                        this->missionIs(MissionStatus::MISSION_PIT_STANDBY)) {
                return this->mission_status_msg.mission_status_code;

            } else {
                return MissionStatus::MISSION_STANDBY;
            }
            break;
          
        default:
            return MissionStatus::MISSION_PIT_STANDBY;

        }
        
        return MissionStatus::MISSION_PIT_STANDBY;
        break;
    
    case RCFlagSummary::VEH_FLAG_CHECKERED:
      return MissionStatus::MISSION_PIT_STANDBY;
      break;

    case RCFlagSummary::VEH_FLAG_BLANK:
        return getMissionVehFlagNull();      

    case RCFlagSummary::VEH_FLAG_NULL:
        return getMissionVehFlagNull();

      break; // VEH_FLAG_BLANK
    
    default:
      return MissionStatus::MISSION_COMMANDED_STOP;
      break;
    }
  } else {
      return MissionStatus::MISSION_COMMANDED_STOP;
  }
}

MissionStatus::_mission_status_code_type MissionManagerNode::getMissionVehFlagNull()
{
    switch (this->rc_flag_summary.track_flag)
      {
        case RCFlagSummary::TRACK_FLAG_RED: // TODO ASK TO RC WHAT SHOULD HAPPEN HERE
            if (is_system_startup)
            {
                return MissionStatus::MISSION_DEFAULT; // No missions on startup
            } else {
                return MissionStatus::MISSION_COMMANDED_STOP;
            }
            break;

        case RCFlagSummary::TRACK_FLAG_ORANGE:
            if (is_system_startup && this->missionIs(MissionStatus::MISSION_DEFAULT))
            {
                return MissionStatus::MISSION_PIT_INIT;

            } else if (this->missionIs(MissionStatus::MISSION_PIT_INIT)) {
                is_system_startup = false;
                return MissionStatus::MISSION_PIT_STANDBY;

            } else if (this->missionIs(MissionStatus::MISSION_INIT)) {
                is_system_startup = false;
                return MissionStatus::MISSION_STANDBY;

            } else if ( this->missionIs(MissionStatus::MISSION_STANDBY) ||
                        this->missionIs(MissionStatus::MISSION_PIT_STANDBY)) {
                return this->mission_status_msg.mission_status_code;

            } else {
                return MissionStatus::MISSION_STANDBY;
            }
            break;
          
        case RCFlagSummary::TRACK_FLAG_YELLOW:
            if (this->missionIs(MissionStatus::MISSION_PIT_STANDBY) || 
                this->missionIs(MissionStatus::MISSION_PIT_OUT)) {
                return MissionStatus::MISSION_PIT_TO_TRACK;
            } else if (this->missionIs(MissionStatus::MISSION_PIT_IN)) {
                return MissionStatus::MISSION_PIT_STANDBY; 
            } else {
                return MissionStatus::MISSION_SLOW_DRIVE;
            }
            break;


        case RCFlagSummary::TRACK_FLAG_GREEN:
            if (this->missionIs(MissionStatus::MISSION_PIT_IN)) {
                return MissionStatus::MISSION_PIT_STANDBY;
            } else {
                // TODO If on track, RACE should be set and maintained.
                return MissionStatus::MISSION_TEST;
            }
            break;

        default:
          return MissionStatus::MISSION_COMMANDED_STOP;
          break;
      }
}

void
MissionManagerNode::RCFlagSummaryCallback(
        const nif::common::msgs::RCFlagSummary::UniquePtr msg) {
    this->rc_flag_summary = std::move(*msg);
    this->rc_flag_summary_update_time = this->now();
    this->has_rc_flag_summary = true;
}

void 
MissionManagerNode::velocityCallback(
      const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
    this->current_velocity_mps = (msg->rear_left + msg->rear_right) * 0.5 * nif::common::constants::KPH2MS;
    this->current_velocity_update_time = this->now();
    this->has_current_velocity = true;
  }

rcl_interfaces::msg::SetParametersResult
MissionManagerNode::parametersCallback(
        const std::vector<rclcpp::Parameter> &vector) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "";
    for (const auto &param : vector) 
    {
    }
}

double nif::system::MissionManagerNode::getMissionMaxVelocityMps(
        MissionStatus::_mission_status_code_type mission_code) {
    double max_vel_mps = 0.0;
    switch (mission_code) {
        case MissionStatus::MISSION_EMERGENCY_STOP:
            max_vel_mps = this->velocity_zero;
            break;

        case MissionStatus::MISSION_COMMANDED_STOP:
            max_vel_mps = this->velocity_zero;
            break;

        case MissionStatus::MISSION_STANDBY:
            max_vel_mps = this->velocity_zero;
            break;

        case MissionStatus::MISSION_SLOW_DRIVE:
            max_vel_mps = this->velocity_slow_drive;
            break;

        case MissionStatus::MISSION_PIT_IN:
            max_vel_mps = this->velocity_pit_in;
            break;

        case MissionStatus::MISSION_PIT_STANDBY:
            max_vel_mps = this->velocity_zero;
            break;

        case MissionStatus::MISSION_PIT_OUT:
            max_vel_mps = this->velocity_pit_out;
            break;

        case MissionStatus::MISSION_RACE:
            // Race at max speed, if localization is good enough. 
            max_vel_mps = this->velocity_max;
            break;

        case MissionStatus::MISSION_TEST:
            max_vel_mps = this->velocity_max;
            break;
            
        default:
            max_vel_mps = this->velocity_zero;
            break;
    }

    return max_vel_mps;
}

MissionStatus::_mission_status_code_type MissionManagerNode::validateMissionTransition(MissionStatus::_mission_status_code_type next_mission)
{
    if (this->missionIs(next_mission)) return next_mission;
    
    auto next_mission_desc = this->missions_description.find(next_mission);
    if (next_mission_desc != this->missions_description.end())
    {
        if (next_mission_desc->second.isValid(
            this->mission_status_msg.mission_status_code,
            next_mission,
            this->getEgoOdometry(),
            this->current_velocity_mps))
            {
                return next_mission;
            } else if (next_mission_desc->second.fallback.isActive())
            {   
                // Fallback to other mission while rejecting next_mission, but only if allowed
                return this->validateMissionTransition(next_mission_desc->second.fallback.mission_code);
            }
    } else {
        // No criteria specified
        return next_mission; 
    }

    // Transition rejected, return current state
    return this->mission_status_msg.mission_status_code;
}
