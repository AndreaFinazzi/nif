//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_system_status_manager_nodes/system_status_manager_node.h"
#include <chrono>

using nif::system::SystemStatusManagerNode;
using nif_msgs::msg::MissionStatus;
using namespace std::chrono_literals;

SystemStatusManagerNode::SystemStatusManagerNode(
        const std::string &node_name)
        : Node(node_name) {
//    Initialize to false
    this->system_status_msg.health_status.system_failure = false;

    this->declare_parameter("timeout_node_inactive_ms", 1000);
    this->declare_parameter("timeout_bestpos_msg_ms", 500);
    this->declare_parameter("timeout_bestpos_diff_age_ms", 60000);
    this->declare_parameter("timeout_mission_ms", 1000);
    this->declare_parameter("lat_autonomy_enabled", false);
    this->declare_parameter("long_autonomy_enabled", false);
    this->declare_parameter("insstdev_threshold", 2.0);

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


    this->system_status_msg.autonomy_status.lateral_autonomy_enabled = this->get_parameter(
            "lat_autonomy_enabled").as_bool();
    this->system_status_msg.autonomy_status.longitudinal_autonomy_enabled = this->get_parameter(
            "long_autonomy_enabled").as_bool();

    auto timeout_node_inactive_ms = this->get_parameter("timeout_node_inactive_ms").as_int();
    auto timeout_bestpos_msg_ms = this->get_parameter("timeout_bestpos_msg_ms").as_int();
    auto timeout_bestpos_diff_age_ms = this->get_parameter("timeout_bestpos_diff_age_ms").as_int();
    auto timeout_mission_ms = this->get_parameter("timeout_mission_ms").as_int();

    if (timeout_node_inactive_ms <= 0 ||
        timeout_bestpos_msg_ms <= 0 ||
        timeout_bestpos_diff_age_ms <= 0 ||
        timeout_mission_ms <= 0)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "timeouts must be greater than zero. Got timeout_node_inactive_ms: %d; timeout_bestpos_msg_ms: %d; timeout_bestpos_diff_age_ms: %d; timeout_mission_ms: %d",
                     timeout_node_inactive_ms, timeout_bestpos_msg_ms, timeout_bestpos_diff_age_ms, timeout_mission_ms);
        throw std::range_error("Parameter out of range.");
    }

    this->timeout_node_inactive = rclcpp::Duration(timeout_node_inactive_ms * 1000000);
    this->timeout_bestpos_last_update = rclcpp::Duration(timeout_bestpos_msg_ms * 1000000);
    this->timeout_bestpos_diff_age = rclcpp::Duration(timeout_bestpos_diff_age_ms * 1000000);
    this->timeout_mission = rclcpp::Duration(timeout_mission_ms * 1000000);

    this->insstdev_threshold = this->get_parameter("insstdev_threshold").as_double();

    this->velocity_zero = this->get_parameter("velocity.zero").as_double();
    this->velocity_max = this->get_parameter("velocity.max").as_double();
    this->velocity_pit_in = this->get_parameter("velocity.pit_in").as_double();
    this->velocity_pit_out = this->get_parameter("velocity.pit_out").as_double();
    this->velocity_slow_drive = this->get_parameter("velocity.slow_drive").as_double();
    this->safeloc_threshold_stop = this->get_parameter("safeloc.threshold_stop").as_double();
    this->safeloc_threshold_slow_down = this->get_parameter("safeloc.threshold_slow_down").as_double();
    this->safeloc_velocity_slow_down_max = this->get_parameter("safeloc.velocity_slow_down_max").as_double();
    this->safeloc_velocity_slow_down_min = this->get_parameter("safeloc.velocity_slow_down_min").as_double();

    this->parameters_callback_handle = this->add_on_set_parameters_callback(
            std::bind(&SystemStatusManagerNode::parametersCallback, this, std::placeholders::_1));

    // Inintializa to failure state
    this->system_status_msg.health_status.communication_failure = true;
    this->system_status_msg.health_status.localization_failure = true;
    this->system_status_msg.health_status.system_failure = true;
    this->system_status_msg.health_status.commanded_stop = false;

    // Subscribers
    this->joystick_sub = this->create_subscription<deep_orange_msgs::msg::JoystickCommand>(
            "in_joystick_cmd", nif::common::constants::QOS_CONTROL_CMD_OVERRIDE,
            std::bind(&SystemStatusManagerNode::joystickCallback, this, std::placeholders::_1));

    // TODO define internal msg and proper conversion
    this->mission_status_sub = this->create_subscription<common::msgs::MissionStatus>(
            "in_mission_status", nif::common::constants::QOS_INTERNAL_STATUS,
            std::bind(&SystemStatusManagerNode::missionCallback, this, std::placeholders::_1));

    this->localization_error_sub = this->create_subscription<std_msgs::msg::Float64>(
            "in_localization_error", nif::common::constants::QOS_EGO_ODOMETRY,
            std::bind(&SystemStatusManagerNode::localizationErrorCallback, this, std::placeholders::_1));

    // TODO this is only temporary, as the localization status should be handled in the localization nodes.
    this->subscriber_bestpos = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
            "in_novatel_bestpos", 1,
            std::bind(&SystemStatusManagerNode::receive_bestpos, this, std::placeholders::_1));

    // TODO this is only temporary, as the localization status should be handled in the localization nodes.
    this->subscriber_insstdev = this->create_subscription<novatel_oem7_msgs::msg::INSSTDEV>(
            "in_novatel_insstdev", 1,
            std::bind(&SystemStatusManagerNode::receive_insstdev, this, std::placeholders::_1));


    //  Publishers
    this->system_status_pub = this->create_publisher<nif::common::msgs::SystemStatus>(
            "out_system_status", nif::common::constants::QOS_INTERNAL_STATUS
    );

    this->joy_emergency_pub = this->create_publisher<std_msgs::msg::Bool>(
            "/vehicle/emergency_joystick", 10);
    this->hb_emergency_pub = this->create_publisher<std_msgs::msg::Bool>(
            "/vehicle/emergency_heartbeat", 10);

    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
    qos.best_effort();

    // Services
    // TODO make global parameter
    this->register_node_service = this->create_service<nif_msgs::srv::RegisterNodeStatus>(
            "/system_status_manager/register",
            std::bind(
                    &SystemStatusManagerNode::registerNodeServiceHandler,
                    this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)
    );

    this->system_status_timer = this->create_wall_timer(
            nif::common::constants::SYNC_PERIOD_DEFAULT_US,
            [this] { systemStatusTimerCallback(); });

    this->recovery_service = this->create_service<std_srvs::srv::Trigger>(
            "/system_status_manager/recover",
            std::bind(
                    &SystemStatusManagerNode::recoveryServiceHandler,
                    this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void SystemStatusManagerNode::systemStatusTimerCallback() {
    // Check for passed out topics
    nodeStatusesAgeCheck();

    // check safety conditions
    bool hb_ok = heartbeatOk();
    bool localization_ok = true; // localizationOk(); // gps_health_ok();

    if (!hb_ok || !this->recovery_enabled) {
        hb_ok = false;
    }
    this->system_status_msg.health_status.communication_failure = !hb_ok;
    this->system_status_msg.health_status.localization_failure = !localization_ok;

    // if operator commands emergency stop
    if (joy_emergency_stop) {
        this->system_status_msg.health_status.commanded_stop = true;
    }

    // TODO Node status list is ot used, at the moment
    this->system_status_msg.header.stamp = this->now();
    this->system_status_msg.health_status.system_failure = !isSystemHealthy();
    this->system_status_msg.health_status.system_status_code = getSystemStatusCode();

    auto message_joy = std_msgs::msg::Bool();
    auto message_hb = std_msgs::msg::Bool();
    message_joy.data = this->system_status_msg.health_status.commanded_stop;
    message_hb.data = this->system_status_msg.health_status.communication_failure;
    // Publish diagnostic msgs 
    this->joy_emergency_pub->publish(message_joy);
    this->hb_emergency_pub->publish(message_hb);

    // Mission age check
    if (!this->has_mission ||
        this->now() - this->mission_update_time > this->timeout_mission) {
        this->system_status_msg.mission_status.mission_status_code = MissionStatus::MISSION_EMERGENCY_STOP;
        // Mission velocity check
        this->system_status_msg.mission_status.max_velocity_mps = static_cast<nif_msgs::msg::MissionStatus::_max_velocity_mps_type>(
            this->getMissionMaxVelocityMps(this->system_status_msg.mission_status.mission_status_code));
    }

    this->system_status_pub->publish(this->system_status_msg);
}

void SystemStatusManagerNode::subscribeNodeStatus(
        const std::string &topic_name) {

    auto subscription = this->create_subscription<nif::common::msgs::NodeStatus>(
            topic_name, nif::common::constants::QOS_INTERNAL_STATUS,
            [&](const nif::common::msgs::NodeStatus::SharedPtr msg) {
                nodeStatusUpdate(msg);
            });
    this->node_statuses_subs.push_back(std::move(subscription));
}

void SystemStatusManagerNode::registerNodeServiceHandler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const nif_msgs::srv::RegisterNodeStatus::Request::SharedPtr request,
        nif_msgs::srv::RegisterNodeStatus::Response::SharedPtr response) {
//  Request data check
    if (
            !request->node_name.empty() &&
            this->status_index_by_name.find(request->node_name) == this->status_index_by_name.end() &&
            // Process only if not already subscribed
            isNodeTypeInRange(static_cast<NodeType>(request->node_type)) &&
            !request->status_topic_name.empty()) {
        auto node_id = this->newStatusRecord(request);

        if (node_id > 0) {
            response->node_id = node_id;
            response->success = true;
            response->message = "Node " + request->node_name + " hase been registered in SSM.";
            return;
        }
    }

    response->node_id = 0;
    response->success = false;
    response->message = "ERROR: Bad service request.";
}

void SystemStatusManagerNode::nodeStatusUpdate(
        const nif::common::msgs::NodeStatus::SharedPtr msg) {
    // TODO implement checks over msg.
    if (this->now() - msg->stamp_last_update >= this->timeout_node_inactive) {
        // TODO implement check on last_update_stamp to detect inactive nodes.
        msg->node_status_code = common::NODE_INACTIVE;
    }
    auto index = this->status_index_by_id[msg->node_id];
    this->node_status_records[index]->node_status = msg;
    this->system_status_msg.health_status.node_statuses_list[index].node_status_code = msg->node_status_code;
}

bool SystemStatusManagerNode::isSystemHealthy() {
    for (const auto &record : this->node_status_records) {
        if (!record->node_status ||
            record->node_status->node_status_code != common::NODE_OK) {
            this->is_system_healthy = false;
            return this->is_system_healthy;
        }
    }
    // All nodes OK
    this->is_system_healthy = true;
    return this->is_system_healthy;
}

SystemStatusCode SystemStatusManagerNode::getSystemStatusCode() {
    // TODO implement meaningful FSM
    if (isSystemHealthy()) {
        for (const auto &record : this->node_status_records) {
            if (record->node_status &&
                record->node_status->node_status_code != common::NODE_OK) {
                return SystemStatusCode::SYSTEM_ERROR;
            }
        }

    } else {
        return SystemStatusCode::SYSTEM_ERROR;
    }
    // All nodes OK
    return SystemStatusCode::SYSTEM_OK;
}

void SystemStatusManagerNode::nodeStatusesAgeCheck() {
    // Extra iteration over all the records, it could be moved smw else, but
    // assuming a reasonable number of nodes, it shouldn't affect performance significantly.
    for (const auto &record : this->node_status_records) {
        // Check if last update is recent enough
        if (record->node_status && (this->now() - record->node_status->stamp) >=
                                   nif::system::NODE_DEAD_TIMEOUT) {
            record->node_status->node_status_code = common::NODE_DEAD;
            auto index = this->status_index_by_id[record->node_status->node_id];
            this->system_status_msg.health_status.node_statuses_list[index].node_status_code = record->node_status->node_status_code;
        }
    }
}

nif::common::types::t_node_id SystemStatusManagerNode::newStatusRecord(
        const nif_msgs::srv::RegisterNodeStatus::Request::SharedPtr request) {
    nif::common::types::t_node_id node_id;
    node_id = next_node_id++;
    nif_msgs::msg::NodeStatusShort node_status_short;

    {
        // Add the calling node to the registry of watched nodes
        auto node_status_record_ptr = std::make_unique<NodeStatusRecord>();
        node_status_record_ptr->node_status = std::make_shared<nif_msgs::msg::NodeStatus>();
        node_status_record_ptr->node_status->node_id = node_id;
        node_status_record_ptr->node_status->node_status_code = common::NODE_NOT_INITIALIZED;
        this->node_status_records.push_back(std::move(node_status_record_ptr));
    }
    auto node_index = this->node_status_records.size() - 1;
    node_status_short.node_id = node_id;
    node_status_short.node_name = request->node_name;
    node_status_short.node_status_code = common::NODE_INACTIVE;

    this->system_status_msg.health_status.node_list.push_back(request->node_name);
    auto it = this->system_status_msg.health_status.node_statuses_list.begin();
    this->system_status_msg.health_status.node_statuses_list.insert(it + node_index, std::move(node_status_short));

    this->status_index_by_id.insert({
                                      node_id, node_index
                              });

    this->status_index_by_name.insert(
            {
                    std::move(request->node_name),
                    node_index
            });

    auto node_type = static_cast<const NodeType>(request->node_type);
    auto by_type_registry = this->status_indices_by_type.find(node_type);
    // Key is not present
    if (by_type_registry == this->status_indices_by_type.end()) {
        auto type_vector = std::make_unique<std::vector<t_record_index>>();
        type_vector->push_back(node_index);
        this->status_indices_by_type.insert({
                                              node_type,
                                              std::move(type_vector)
                                      });
    } else {
        this->status_indices_by_type[node_type]->push_back(node_index);
    }

    // Subscribe to node status topic
    subscribeNodeStatus(request->status_topic_name);

    return node_id;
}

bool SystemStatusManagerNode::heartbeatOk() {
    // check for timeouts
    if (counter_joy_prev != counter_joy) {
        // received new message, heartbeat ok
        t = 0;
        if (counter_joy != default_counter) {
            counter_joy_prev = counter_joy;
        }
        return true;
    } else {
        // have not received update; check for timeout
        // if (t < 3 * max_counter_drop) t++; // Avoid huge (hardly recoverable) numbers
        t++;

        if (t >= max_counter_drop) {
            this->recovery_enabled = false;
            return false;
        } else {
            // timeout not reached yet
            return true;
        }
    }
}

bool SystemStatusManagerNode::localizationOk() {
    bool has_bestpos_trigger = !this->has_bestpos;
    
    bool bestpos_diff_age_trigger = (this->bestpos_diff_age_s > this->timeout_bestpos_diff_age.seconds() );
    bool bestpos_last_update_trigger = !this->has_bestpos || (( this->now() - this->bestpos_last_update ) > this->timeout_bestpos_last_update );

    bool loc_error_trigger = !this->has_localization_error || ( this->localization_error > this->safeloc_threshold_stop );

    return (!has_bestpos_trigger && !bestpos_diff_age_trigger && !bestpos_last_update_trigger && !loc_error_trigger);
}

void SystemStatusManagerNode::recoveryServiceHandler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (!this->recovery_enabled) {

        this->recovery_enabled = true;
        response->success = this->recovery_enabled;
        response->message = "recovery_enabled set to ";
        response->message.append(this->recovery_enabled ? "true" : "false");
    } else {
        response->success = true;
        response->message = "WARN: service call has been ignored, recovery_enable is already true. (pull-up only)";
    }
}

void SystemStatusManagerNode::joystickCallback(const nif::common::msgs::OverrideControlCmd::SharedPtr msg) {
    // parse counter
    this->counter_joy = msg->counter;
    // parse emergency
    if (msg->emergency_stop == 1) {
        joy_emergency_stop = true;
    }
}

void SystemStatusManagerNode::localizationErrorCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
    this->has_localization_error = true;
    this->localization_error_last_update = this->now();
    this->localization_error = msg->data;
}

void
SystemStatusManagerNode::missionCallback(
        const nif::common::msgs::MissionStatus::UniquePtr msg) {
    this->mission_update_time = this->now();
    this->has_mission = true;
    this->system_status_msg.mission_status = std::move(*msg);

    // Mission velocity check
    this->system_status_msg.mission_status.max_velocity_mps = static_cast<nif_msgs::msg::MissionStatus::_max_velocity_mps_type>(
            this->getMissionMaxVelocityMps(this->system_status_msg.mission_status.mission_status_code));
}

rcl_interfaces::msg::SetParametersResult
SystemStatusManagerNode::parametersCallback(
        const std::vector<rclcpp::Parameter> &vector) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "";
    for (const auto &param : vector) {
        if (param.get_name() == "lat_autonomy_enabled") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                if (true) // TODO implement switching policy, if needed
                {
                    this->system_status_msg.autonomy_status.lateral_autonomy_enabled = param.as_bool();
                    result.successful = true;
                }
            }
        } else if (param.get_name() == "long_autonomy_enabled") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
                if (true) {
                    this->system_status_msg.autonomy_status.longitudinal_autonomy_enabled = param.as_bool();
                    result.successful = true;
                }
            }
        } else if (param.get_name() == "velocity.max") {
            if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
                if (param.as_bool() < 100.) {
                    this->velocity_max = param.as_double();
                    result.successful = true;
                }
            }
        }
        return result;
    }
}

double nif::system::SystemStatusManagerNode::getMissionMaxVelocityMps(
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
            processSafelocVelocity(max_vel_mps);
            break;

        case MissionStatus::MISSION_TEST:
            max_vel_mps = this->velocity_max;
            processSafelocVelocity(max_vel_mps);
            break;

        default:
            max_vel_mps = this->velocity_zero;
            break;
    }

    return max_vel_mps;
}

void SystemStatusManagerNode::processSafelocVelocity(double & max_vel_mps) 
{
    if (!this->has_localization_error ||
        this->now() - this->localization_error_last_update > this->timeout_localization_error)
        {
            max_vel_mps = this->velocity_zero;
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Data timeout in SystemStatusManagerNode::processSafelocVelocity();");
            return;
        }

    // Over safeloc_threshold_slow_down, reduce max speed according to the localization error
    if (this->localization_error > this->safeloc_threshold_slow_down)
    {
        auto slope = (this->safeloc_velocity_slow_down_max - this->safeloc_velocity_slow_down_min) / (this->safeloc_threshold_slow_down - this->safeloc_threshold_stop);
        max_vel_mps = slope * (this->localization_error - this->safeloc_threshold_slow_down) + this->safeloc_velocity_slow_down_max;
    }

    if (max_vel_mps < this->safeloc_velocity_slow_down_min) 
        max_vel_mps = this->safeloc_velocity_slow_down_min;
}