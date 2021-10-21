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
    this->declare_parameter("velocity.max", 37.0);
    this->declare_parameter("velocity.avoidance", 20.0);
    this->declare_parameter("velocity.warmup", 20.0);
    this->declare_parameter("velocity.pit_in", 8.0);
    this->declare_parameter("velocity.pit_out", 8.0);
    this->declare_parameter("velocity.slow_drive", 8.0);
    this->declare_parameter("safeloc.threshold_stop", 40.0);
    this->declare_parameter("safeloc.threshold_slow_down", 20.0);
    this->declare_parameter("safeloc.velocity_slow_down_max", 22.2);
    this->declare_parameter("safeloc.velocity_slow_down_min", 8.0);

    this->declare_parameter("missions_file_path", "");
    this->declare_parameter("zones_file_path", "");

    this->declare_parameter("mission.avoidance.auto_switch", false);
    this->declare_parameter("mission.avoidance.lap_count", 0);
    this->declare_parameter("mission.avoidance.previous_track_flag", 1);
    this->declare_parameter("mission.avoidance.lap_distance_min", 0);
    this->declare_parameter("mission.avoidance.lap_distance_max", 0);

    this->declare_parameter("mission.warmup.auto_switch", false);

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
    this->velocity_avoidance = this->get_parameter("velocity.avoidance").as_double();
    this->velocity_warmup = this->get_parameter("velocity.warmup").as_double();
    this->velocity_pit_in = this->get_parameter("velocity.pit_in").as_double();
    this->velocity_pit_out = this->get_parameter("velocity.pit_out").as_double();
    this->velocity_slow_drive = this->get_parameter("velocity.slow_drive").as_double();
    this->safeloc_threshold_stop = this->get_parameter("safeloc.threshold_stop").as_double();
    this->safeloc_threshold_slow_down = this->get_parameter("safeloc.threshold_slow_down").as_double();
    this->safeloc_velocity_slow_down_max = this->get_parameter("safeloc.velocity_slow_down_max").as_double();
    this->safeloc_velocity_slow_down_min = this->get_parameter("safeloc.velocity_slow_down_min").as_double();

    auto missions_file_path = this->get_parameter("missions_file_path").as_string();
    auto zones_file_path = this->get_parameter("zones_file_path").as_string();

    this->mission_avoidance_auto_switch = this->get_parameter("mission.avoidance.auto_switch").as_bool();
    this->mission_avoidance_lap_count = this->get_parameter("mission.avoidance.lap_count").as_int();
    this->mission_avoidance_previous_track_flag = this->get_parameter("mission.avoidance.previous_track_flag").as_int();
    this->mission_avoidance_lap_distance_min = this->get_parameter("mission.avoidance.lap_distance_min").as_int();
    this->mission_avoidance_lap_distance_max = this->get_parameter("mission.avoidance.lap_distance_max").as_int();    

    this->mission_warmup_auto_switch = this->get_parameter("mission.warmup.auto_switch").as_bool();

    if (!nif::common::msgs::isTrackFlagInRange(this->mission_avoidance_previous_track_flag))
    {
        throw std::runtime_error("parameter mission.avoidance.previous_track_flag does not represent a valid track flag.");
    }

    if (this->mission_avoidance_lap_distance_max == 0)
        this->mission_avoidance_lap_distance_max = std::numeric_limits<decltype(this->mission_avoidance_lap_distance_max)>().max();

    if (this->mission_avoidance_lap_distance_min > this->mission_avoidance_lap_distance_max)
    {
        throw std::runtime_error("parameter mission.avoidance.lap_distance_min cannot be greater than mission.avoidance.lap_distance_max");
    }

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

    // Services
    this->recovery_service = this->create_service<std_srvs::srv::Trigger>(
            "/mission_manager/recover",
            std::bind(
                    &MissionManagerNode::recoveryServiceHandler,
                    this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    this->avoidance_service = this->create_service<std_srvs::srv::Trigger>(
            "/mission_manager/avoidance",
            std::bind(
                    &MissionManagerNode::avoidanceServiceHandler,
                    this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

    this->warmup_service = this->create_service<std_srvs::srv::Trigger>(
            "/mission_manager/warmup",
            std::bind(
                    &MissionManagerNode::warmupServiceHandler,
                    this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));


    MissionParser::loadMissionsDescription(missions_file_path, this->missions_description);
    MissionParser::loadZonesDescription(zones_file_path, this->zones_description);
    
    if (this->mission_warmup_auto_switch)
        this->is_warmup_enabled = true;

    this->setNodeStatus(common::NodeStatusCode::NODE_INITIALIZED);
}

void MissionManagerNode::run() {
    auto node_status = common::NodeStatusCode::NODE_ERROR;
    if (!this->isDataOk())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Data timeout in MissionManagerNode::run();");
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
                if (this->missionIs(MissionStatus::MISSION_PIT_INIT) || 
                    this->missionIs(MissionStatus::MISSION_PIT_IN) || 
                    this->missionIs(MissionStatus::MISSION_PIT_STANDBY)) {
                    return this->mission_status_msg.mission_status_code;
                } else {
                    return MissionStatus::MISSION_COMMANDED_STOP;
                }
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

            } else {
                return this->mission_status_msg.mission_status_code; // ignore orange flag after startup
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
                if (this->missionIs(MissionStatus::MISSION_PIT_INIT)) {
                    return MissionStatus::MISSION_PIT_INIT;

                } else if (this->missionIs(MissionStatus::MISSION_INIT)) {
                    return MissionStatus::MISSION_STANDBY;

                } else if ( this->missionIs(MissionStatus::MISSION_STANDBY) ||
                            this->missionIs(MissionStatus::MISSION_PIT_STANDBY)) {
                    return this->mission_status_msg.mission_status_code;

                }     
                return MissionStatus::MISSION_PIT_INIT;
            }
            break;

        case RCFlagSummary::TRACK_FLAG_ORANGE:
            if (is_system_startup && this->missionIs(MissionStatus::MISSION_DEFAULT))
            {
                // is_system_startup = false;
                return MissionStatus::MISSION_PIT_INIT;
            } 
            else if (this->missionIs(MissionStatus::MISSION_PIT_INIT)) {
                is_system_startup = false;
                return this->mission_status_msg.mission_status_code;

            } 
            else if (this->missionIs(MissionStatus::MISSION_INIT)) {
                is_system_startup = false;
                return MissionStatus::MISSION_STANDBY;

            }
            return this->mission_status_msg.mission_status_code;
            break;

        case RCFlagSummary::TRACK_FLAG_YELLOW:
            if (this->missionIs(MissionStatus::MISSION_PIT_STANDBY) || 
                this->missionIs(MissionStatus::MISSION_PIT_OUT)     ||
                this->missionIs(MissionStatus::MISSION_PIT_INIT)
                ) {
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
                if (this->is_warmup_enabled)
                    return MissionStatus::MISSION_TIRE_WARMUP;
                if (this->is_avoidance_enabled)
                    return MissionStatus::MISSION_COLLISION_AVOIDNACE;
                return MissionStatus::MISSION_RACE;
            }
            break;

        default:
          return MissionStatus::MISSION_COMMANDED_STOP;
          break;
      }
    }

void
MissionManagerNode::RCFlagSummaryCallback(
        const nif::common::msgs::RCFlagSummary::UniquePtr msg) 
{
    // TODO Remove this stuff
    this->lap_count = msg->lap_count;
    this->lap_distance = msg->lap_distance;

    // Auto transition to collision avoidance mode
    if (
        !this->is_system_startup    &&
        this->mission_avoidance_auto_switch && 
        this->mission_avoidance_lap_count == this->lap_count  &&
        this->mission_avoidance_previous_track_flag == this->rc_flag_summary.track_flag  &&
        this->mission_avoidance_lap_distance_min <= this->lap_distance &&
        this->mission_avoidance_lap_distance_max >= this->lap_distance  )
    {
        this->is_avoidance_enabled = true;
    }

    if (
        this->mission_warmup_auto_switch && 
        this->track_zones_hit_count_map.find(5) != this->track_zones_hit_count_map.end() &&
        this->track_zones_hit_count_map[5] >= 2 )
    {
        this->is_warmup_enabled = false;
    }

    this->rc_flag_summary = std::move(*msg);
    this->rc_flag_summary_update_time = this->now();
    this->has_rc_flag_summary = true;
}

void 
MissionManagerNode::velocityCallback(
      const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
    this->current_velocity_mps = (msg->front_left + msg->front_right) * 0.5 * nif::common::constants::KPH2MS;
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
        if (param.get_name() == "velocity.max") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param.as_double() >= 0.0 && param.as_double() <= 67.0) // TODO implement switching policy, if needed
                {
                    this->velocity_max = param.as_double();
                    result.successful = true;
                }
            }
        } else if (param.get_name() == "velocity.pit_in") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param.as_double() >= 0.0 && param.as_double() <= 20.0) // TODO implement switching policy, if needed
                {
                    this->velocity_pit_in = param.as_double();
                    result.successful = true;
                }
            }
        } else if (param.get_name() == "velocity.pit_out") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param.as_double() >= 0.0 && param.as_double() <= 20.0) // TODO implement switching policy, if needed
                {
                    this->velocity_pit_out = param.as_double();
                    result.successful = true;
                }
            }
        } else if (param.get_name() == "velocity.slow_drive") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param.as_double() >= 0.0 && param.as_double() <= 30.0) // TODO implement switching policy, if needed
                {
                    this->velocity_slow_drive = param.as_double();
                    result.successful = true;
                }
            }
        } else if (param.get_name() == "velocity.avoidance") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param.as_double() >= 0.0 && param.as_double() <= 67.0) // TODO implement switching policy, if needed
                {
                    this->velocity_avoidance = param.as_double();
                    result.successful = true;
                }
            }
        } else if (param.get_name() == "velocity.warmup") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param.as_double() >= 0.0 && param.as_double() <= 40.0) // TODO implement switching policy, if needed
                {
                    this->velocity_warmup = param.as_double();
                    result.successful = true;
                }
            }
        }
    }

    return result;
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

        case MissionStatus::MISSION_PIT_TO_TRACK:
            max_vel_mps = this->velocity_pit_out;
            break;

        case MissionStatus::MISSION_RACE:
            // Race at max speed, if localization is good enough. 
            max_vel_mps = this->velocity_max;
            break;

        case MissionStatus::MISSION_TEST:
            max_vel_mps = this->velocity_max;
            break;
            
        case MissionStatus::MISSION_COLLISION_AVOIDNACE:
            max_vel_mps = this->velocity_avoidance;
            break;
            
        case MissionStatus::MISSION_TIRE_WARMUP:
            max_vel_mps = this->velocity_warmup;
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

void MissionManagerNode::recoveryServiceHandler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response) 
{
        if (this->missionIs(MissionStatus::MISSION_STANDBY)         ||
            this->missionIs(MissionStatus::MISSION_PIT_INIT)        ||
            this->missionIs(MissionStatus::MISSION_EMERGENCY_STOP)  ||
            this->missionIs(MissionStatus::MISSION_COMMANDED_STOP))
        {
            this->is_system_startup = true;
            this->mission_status_msg.mission_status_code = MissionStatus::MISSION_DEFAULT;
            response->success = this->is_system_startup;
            response->message = "is_system_startup set to ";
            response->message.append(this->is_system_startup ? "true" : "false");
            // response->message.append(";/tmission_status_code set to %d", this->mission_status_msg.mission_status_code);
            response->success = true;
        } else {
            response->message = "Mission reset not allowed in the current status.";
            response->success = false;
        }
}

  void MissionManagerNode::avoidanceServiceHandler(
          const std::shared_ptr<rmw_request_id_t> request_header,
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            this->is_avoidance_enabled = !this->is_avoidance_enabled;
            response->message = "is_avoidance_enabled set to ";
            response->message.append(this->is_avoidance_enabled ? "true" : "false");
            response->success = true;
          }

void MissionManagerNode::warmupServiceHandler(
          const std::shared_ptr<rmw_request_id_t> request_header,
          const std_srvs::srv::Trigger::Request::SharedPtr request,
          std_srvs::srv::Trigger::Response::SharedPtr response)
          {
            this->is_warmup_enabled = !this->is_warmup_enabled;
            response->message = "is_warmup_enabled set to ";
            response->message.append(this->is_warmup_enabled ? "true" : "false");
            response->success = true;
          }

void MissionManagerNode::afterEgoOdometryCallback() {
  // Update the current zone and increase hit count if it's a zone transition.
  for (auto &&zone_pair : this->zones_description)
  {
    if (zone_pair.second.isValid(
          this->mission_status_msg.mission_status_code, 
          this->mission_status_msg.mission_status_code, 
          this->getEgoOdometry(), 
          this->current_velocity_mps))
    {
        if (this->current_track_zone_id != zone_pair.second.id)
        {
            if (this->track_zones_hit_count_map.find(zone_pair.second.id) == this->track_zones_hit_count_map.end()) {
                this->track_zones_hit_count_map.insert(std::make_pair(zone_pair.second.id, 1));
            }
            else
            {
                this->track_zones_hit_count_map[zone_pair.second.id]++;
            }
        }
        this->current_track_zone_id = zone_pair.second.id;
    }
  }
}
