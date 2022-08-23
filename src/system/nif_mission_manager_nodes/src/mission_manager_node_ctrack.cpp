//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_mission_manager_nodes/mission_manager_node.h"

using nif::common::msgs::RCFlagSummary;
using nif::system::MissionManagerNode;
using namespace std::chrono_literals;

MissionManagerNode::MissionManagerNode(
    const std::string &node_name)
    : IBaseSynchronizedNode(
          node_name,
          nif::common::NodeType::SYSTEM,
          std::chrono::milliseconds(50))
{
    //    Initialize to false
    this->declare_parameter("timeout_rc_flag_summary_s", 20);
    this->declare_parameter("timeout_velocity_ms", 1500);

    this->declare_parameter("velocity.zero", 0.0);
    this->declare_parameter("velocity.max", 30.0);
    this->declare_parameter("velocity.keep_position", 10.0);
    this->declare_parameter("velocity.constant", 10.0);
    this->declare_parameter("velocity.avoidance", 10.0);
    this->declare_parameter("velocity.warmup", 20.0);
    this->declare_parameter("velocity.pit_in", 5.0);
    this->declare_parameter("velocity.pit_out", 5.0);
    this->declare_parameter("velocity.slow_drive", 10.0);
    this->declare_parameter("safeloc.threshold_stop", 10.0);
    this->declare_parameter("safeloc.threshold_slow_down", 10.0);
    this->declare_parameter("safeloc.velocity_slow_down_max", 12.2);
    this->declare_parameter("safeloc.velocity_slow_down_min", 8.0);

    this->declare_parameter("missions_file_path", "");
    this->declare_parameter("zones_file_path", "");

    this->declare_parameter("mission.avoidance.auto_switch", false);
    this->declare_parameter("mission.avoidance.lap_count_min", 0);
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
    this->velocity_keep_position = this->get_parameter("velocity.keep_position").as_double();
    this->velocity_constant = this->get_parameter("velocity.constant").as_double();
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
    this->mission_avoidance_lap_count_min = this->get_parameter("mission.avoidance.lap_count_min").as_int();
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
        "out_mission_status", nif::common::constants::QOS_INTERNAL_STATUS);

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

void MissionManagerNode::run()
{
    auto node_status = common::NodeStatusCode::NODE_ERROR;

    if(!this->missionIs(MissionStatus::MISSION_DEFAULT))
        {
            is_system_startup = false;
        }
    if (!this->isDataOk())
    {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "Data timeout in MissionManagerNode::run();");
        this->mission_status_msg.mission_status_code = MissionStatus::MISSION_DEFAULT;
        std::cout<<"sibal1"<<std::endl;
    }
    else
    {
        // Mission encoding
        auto next_mission = this->getMissionStatusCode(); // type is MissionStatus::_mission_status_code_type
        this->mission_status_msg.mission_status_code = this->validateMissionTransition(next_mission);
        node_status = common::NodeStatusCode::NODE_OK;
        std::cout<<"sibal2"<<std::endl;
    }
    this->mission_status_msg.max_velocity_mps = static_cast<MissionStatus::_max_velocity_mps_type>(
        this->getMissionMaxVelocityMps(this->mission_status_msg.mission_status_code));

    this->mission_status_pub->publish(this->mission_status_msg);
    this->setNodeStatus(node_status);
}

// TODO this is just a draft, MUST BE REFINED AND FINALIZED
MissionStatus::_mission_status_code_type MissionManagerNode::getMissionStatusCode()
{
    if (this->has_rc_flag_summary)
    {
        switch (this->rc_flag_summary.veh_flag) //! VEHICLE FLAG START
        {

        case RCFlagSummary::VEH_FLAG_NULL:
            return getMissionVehFlagNull(true);

            break; // VEH_FLAG_BLANK

        case RCFlagSummary::VEH_FLAG_DEFENDER:
            return getMissionVehFlagNull(true); // defender_mode on

            break; // VEH_FLAG_BLANK

        case RCFlagSummary::VEH_FLAG_ATTACKER:
            return getMissionVehFlagNull(false);

            break; // VEH_FLAG_BLANK

        default:
            return MissionStatus::MISSION_DEFAULT;
            break;
        }
    }
    else
    {
        return MissionStatus::MISSION_DEFAULT;
    }
}

MissionStatus::_mission_status_code_type MissionManagerNode::getMissionVehFlagNull(bool defender_mode)
{
    switch (this->rc_flag_summary.track_flag) // TRACK FLAG
    {
    case RCFlagSummary::TRACK_FLAG_RED: // TODO ASK TO RC WHAT SHOULD HAPPEN HERE
    
        return MissionStatus::MISSION_DEFAULT;

        // if (is_system_startup)          // INITIAL VALUE IS TRUE
        // {
        //     return MissionStatus::MISSION_DEFAULT; // No missions on startup
        // }
        // else
        // {
        //     return MissionStatus::MISSION_PIT_IN; // GO BACK TO PIT LANE
        //     is_system_startup = true;
        // }
        // break;

    case RCFlagSummary::TRACK_FLAG_ORANGE: // TURN ON THE ENGINE AND PIT IN/OUT SIGNAL
        if (this->missionIs(MissionStatus::MISSION_DEFAULT))
        {
            // is_system_startup = false;
            return MissionStatus::MISSION_PIT_OUT;
        }
        else if (this->missionIs(MissionStatus::MISSION_PIT_OUT))
        {
            // is_system_startup = false;
            return MissionStatus::MISSION_PIT_OUT;
        }
        else // OTHER MISSION STATUS
        {
  
            return MissionStatus::MISSION_PIT_IN;
        }
        break;

    case RCFlagSummary::TRACK_FLAG_YELLOW: // SLOW DRIVE
             
        return MissionStatus::MISSION_SLOW_DRIVE;
       

    case RCFlagSummary::TRACK_FLAG_WAVING_GREEN:

        if (defender_mode)
                return MissionStatus::MISSION_SLOW_DRIVE;

        return MissionStatus::MISSION_RACE;
        // if (this->missionIs(MissionStatus::MISSION_PIT_IN))
        // {
        //     return MissionStatus::;
        // }
        // else if (this->missionIs(MissionStatus::MISSION_PIT_OUT))
        // {
        //     return MissionStatus::MISSION_SLOW_DRIVE;
        // }
        // else
        // { // TODO If on track, RACE should be set and maintained.
        //     

        //     return MissionStatus::MISSION_RACE;
        // }
        // break;

    default:
        return MissionStatus::MISSION_DEFAULT;
        break;
    }
}

void MissionManagerNode::RCFlagSummaryCallback(
    const nif::common::msgs::RCFlagSummary::UniquePtr msg)
{
    // TODO Remove this stuff
    this->lap_count = msg->lap_count;
    this->lap_distance = msg->lap_distance;

    this->mission_status_msg.track_flag = msg->track_flag;
    this->mission_status_msg.veh_flag = msg->veh_flag;

    this->mission_status_msg.lap_count = msg->lap_count;
    this->mission_status_msg.lap_distance = msg->lap_distance;

    // Auto transition to collision avoidance mode
    if (
        !this->is_system_startup &&
        this->mission_avoidance_auto_switch &&
        this->track_zones_hit_count_map.find(5) != this->track_zones_hit_count_map.end() &&
        this->track_zones_hit_count_map[5] >= this->mission_avoidance_lap_count_min &&
        this->mission_avoidance_previous_track_flag == this->rc_flag_summary.track_flag &&
        this->mission_avoidance_lap_distance_min <= this->lap_distance &&
        this->mission_avoidance_lap_distance_max >= this->lap_distance)
    {
        // this->is_avoidance_enabled = true;
        RCLCPP_INFO(this->get_logger(), "Mission Avoidance triggered");
    }

    if (
        this->mission_warmup_auto_switch &&
        this->track_zones_hit_count_map.find(5) != this->track_zones_hit_count_map.end() &&
        this->track_zones_hit_count_map[5] >= 2)
    {
        this->is_warmup_enabled = false;
        RCLCPP_INFO(this->get_logger(), "Mission WarmUp triggered");
    }

    this->rc_flag_summary = std::move(*msg);
    this->rc_flag_summary_update_time = this->now();
    this->has_rc_flag_summary = true;
}

void MissionManagerNode::velocityCallback(
    const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
{
    this->current_velocity_mps = (msg->front_left + msg->front_right) * 0.5 * nif::common::constants::KPH2MS;
    this->current_velocity_update_time = this->now();
    this->has_current_velocity = true;
}

rcl_interfaces::msg::SetParametersResult
MissionManagerNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &vector)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "";
    for (const auto &param : vector)
    {
        if (param.get_name() == "velocity.max")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param.as_double() >= 0.0 && param.as_double() <= 40.0) // TODO implement switching policy, if needed
                {
                    this->velocity_max = param.as_double();
                    result.successful = true;
                }
            }
        }

        else if (param.get_name() == "velocity.pit_in")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param.as_double() >= 0.0 && param.as_double() <= 10.0) // TODO implement switching policy, if needed
                {
                    this->velocity_pit_in = param.as_double();
                    result.successful = true;
                }
            }
        }
        else if (param.get_name() == "velocity.pit_out")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param.as_double() >= 0.0 && param.as_double() <= 10.0) // TODO implement switching policy, if needed
                {
                    this->velocity_pit_out = param.as_double();
                    result.successful = true;
                }
            }
        }
        else if (param.get_name() == "velocity.slow_drive")
        {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                if (param.as_double() >= 0.0 && param.as_double() <= 10.0) // TODO implement switching policy, if needed
                {
                    this->velocity_slow_drive = param.as_double();
                    result.successful = true;
                }
            }
        }
    }

    return result;
}

double nif::system::MissionManagerNode::getMissionMaxVelocityMps(
    MissionStatus::_mission_status_code_type mission_code)
{
    double max_vel_mps = 0.0;
    switch (mission_code)
    {
    case MissionStatus::MISSION_SLOW_DRIVE:
        max_vel_mps = this->velocity_slow_drive;
        break;

    case MissionStatus::MISSION_PIT_IN:
        max_vel_mps = this->velocity_pit_in;
        break;

    case MissionStatus::MISSION_PIT_OUT:
        max_vel_mps = this->velocity_pit_out;
        break;

    case MissionStatus::MISSION_RACE:
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
    if (this->missionIs(next_mission))
        return next_mission; // no mission change

    auto next_mission_desc = this->missions_description.find(next_mission); // missions_description is unordered map
    if (next_mission_desc != this->missions_description.end())              //
    {
        if (next_mission_desc->second.isValid(
                this->mission_status_msg.mission_status_code,
                next_mission,
                this->getEgoOdometry(),
                this->current_velocity_mps))
        {
            return next_mission;
        }
        else if (next_mission_desc->second.fallback.isActive()) // fallback is active!!
        {
            // Fallback to other mission while rejecting next_mission, but only if allowed
            return this->validateMissionTransition(next_mission_desc->second.fallback.mission_code);
        }
    }
    else
    {
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
    if (this->missionIs(MissionStatus::MISSION_STANDBY) ||
        this->missionIs(MissionStatus::MISSION_PIT_INIT) ||
        this->missionIs(MissionStatus::MISSION_EMERGENCY_STOP) ||
        this->missionIs(MissionStatus::MISSION_COMMANDED_STOP))
    {
        this->is_system_startup = true;
        this->mission_status_msg.mission_status_code = MissionStatus::MISSION_DEFAULT;
        response->success = this->is_system_startup;
        response->message = "is_system_startup set to ";
        response->message.append(this->is_system_startup ? "true" : "false");
        // response->message.append(";/tmission_status_code set to %d", this->mission_status_msg.mission_status_code);
        response->success = true;
    }
    else
    {
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

void MissionManagerNode::afterEgoOdometryCallback()
{
    // Update the current zone and increase hit count if it's a zone transition.
    for (auto &&zone_pair : this->zones_description)
    {
        if (zone_pair.second.isValid(
                this->mission_status_msg.mission_status_code,
                this->mission_status_msg.mission_status_code,
                this->getEgoOdometry(),
                this->current_velocity_mps))
        {
            if (this->mission_status_msg.zone_status.zone_id != zone_pair.second.id)
            {
                if (this->track_zones_hit_count_map.find(zone_pair.second.id) == this->track_zones_hit_count_map.end())
                {
                    this->track_zones_hit_count_map.insert(std::make_pair(zone_pair.second.id, 1));
                }
                else
                {
                    this->track_zones_hit_count_map[zone_pair.second.id]++;
                }
            }

            this->mission_status_msg.zone_status.zone_id = zone_pair.second.id;
            this->mission_status_msg.zone_status.zone_type = zone_pair.second.type;
            this->mission_status_msg.zone_status.long_acceleration_max = zone_pair.second.dynamics.long_acceleration_max;
            this->mission_status_msg.zone_status.long_acceleration_min = zone_pair.second.dynamics.long_acceleration_min;
        }
    }
}
