//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_system_status_manager_nodes/system_status_manager_node.h"
#include <chrono>

using nif::system::SystemStatusManagerNode;
using namespace std::chrono_literals;

SystemStatusManagerNode::SystemStatusManagerNode(
        const std::string &node_name)
        : Node(node_name) {
//    Initialize to false
    this->system_status_msg.health_status.system_failure = false;

    this->declare_parameter("timeout_node_inactive_ms", 1000);
    this->declare_parameter("timeout_bestpos_msg_ms", 500);
    this->declare_parameter("timeout_bestpos_diff_age_ms", 60000);
    this->declare_parameter("timeout_rc_flag_summary_s", 10.0);
    this->declare_parameter("lat_autonomy_enabled", false);
    this->declare_parameter("long_autonomy_enabled", false);
    this->declare_parameter("insstdev_threshold", 2.0);

    this->node_inactive_timeout = rclcpp::Duration(1, 0);
    this->system_status_msg.autonomy_status.lateral_autonomy_enabled = this->get_parameter(
            "lat_autonomy_enabled").as_bool();
    this->system_status_msg.autonomy_status.longitudinal_autonomy_enabled = this->get_parameter(
            "long_autonomy_enabled").as_bool();

    auto timeout_bestpos_last_update_ms = this->get_parameter("timeout_bestpos_msg_ms").as_int();
    auto timeout_bestpos_diff_age_ms = this->get_parameter("timeout_bestpos_diff_age_ms").as_int();
    auto timeout_rc_flag_summary_s = this->get_parameter("timeout_bestpos_diff_age_ms").as_int();

    this->timeout_bestpos_last_update = rclcpp::Duration(timeout_bestpos_last_update_ms * 1000000);
    this->timeout_bestpos_diff_age = rclcpp::Duration(timeout_bestpos_diff_age_ms * 1000000);
    this->timeout_rc_flag_summary = rclcpp::Duration(timeout_rc_flag_summary_s, 0);

    this->insstdev_threshold = this->get_parameter("insstdev_threshold").as_double();

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
    this->rc_flag_summary_sub = this->create_subscription<common::msgs::RCFlagSummary>(
            "rc_interface/rc_flag_summary", nif::common::constants::QOS_RACE_CONTROL,
            std::bind(&SystemStatusManagerNode::RCFlagSummaryCallback, this, std::placeholders::_1));

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

    this->system_status_telem_pub = this->create_publisher<nif::common::msgs::SystemStatus>(
            "/telemetry/system_status", qos
    );

    this->telemetry_timer = this->create_wall_timer(
            250ms, std::bind(&SystemStatusManagerNode::telemetry_timer_callback, this));


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
    bool localization_ok = gps_health_ok();

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

    // Mission encoding
    this->system_status_msg.mission_status.mission_status_code = this->getMissionStatus();


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
    if (this->now() - msg->stamp_last_update >= this->node_inactive_timeout) {
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

// TODO this is just a draft, MUST BE REFINED AND FINALIZED
  nif_msgs::msg::MissionStatus::_mission_status_code_type SystemStatusManagerNode::getMissionStatus()
  {
    using nif_msgs::msg::MissionStatus;
    using nif::common::msgs::RCFlagSummary;
    
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
        return MissionStatus::MISSION_PIT_IN;
        break;
      
      case RCFlagSummary::VEH_FLAG_CHECKERED:
        return MissionStatus::MISSION_PIT_IN;
        break;

      case RCFlagSummary::VEH_FLAG_BLANK:

        switch (this->rc_flag_summary.track_flag)
        {
        case RCFlagSummary::TRACK_FLAG_RED:
            return MissionStatus::MISSION_COMMANDED_STOP;
            break;
        case RCFlagSummary::TRACK_FLAG_YELLOW:
            // TODO If in pit, STANDBY?
            return MissionStatus::MISSION_SLOW_DRIVE;
            break;

        case RCFlagSummary::TRACK_FLAG_ORANGE:
            return MissionStatus::MISSION_STANDBY;
            break;

        case RCFlagSummary::TRACK_FLAG_GREEN:
            // TODO If in pit, PIT_OUT should be set and maintained.
            // TODO If on track, RACE should be set and maintained.
            // TODO  
            return MissionStatus::MISSION_TEST;
            break;

        default:
            return MissionStatus::MISSION_COMMANDED_STOP;
            break;
        }

      case RCFlagSummary::VEH_FLAG_NULL:

        switch (this->rc_flag_summary.track_flag)
        {
        case RCFlagSummary::TRACK_FLAG_RED:
            return MissionStatus::MISSION_COMMANDED_STOP;
            break;

        case RCFlagSummary::TRACK_FLAG_ORANGE:
            return MissionStatus::MISSION_STANDBY;
            break;

        case RCFlagSummary::TRACK_FLAG_YELLOW:
            // TODO If in pit, STANDBY?
            return MissionStatus::MISSION_SLOW_DRIVE;
            break;

        case RCFlagSummary::TRACK_FLAG_GREEN:
            // TODO If in pit, PIT_OUT should be set and maintained.
            // TODO If on track, RACE should be set and maintained.
            // TODO  
            return MissionStatus::MISSION_TEST;
            break;

        default:
            return MissionStatus::MISSION_COMMANDED_STOP;
            break;
        }

        break; // VEH_FLAG_BLANK
      
      default:
        return MissionStatus::MISSION_COMMANDED_STOP;
        break;
      }
    } else {
        return MissionStatus::MISSION_COMMANDED_STOP;
    }
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
        t--;
        if (t < 0) {
            t = 0;
        }
        if (counter_joy != default_counter) {
            counter_joy_prev = counter_joy;
        }
        return true;
    } else {
        // have not received update; check for timeout
        if (t < 3 * max_counter_drop) t++; // Avoid huge (hardly recoverable) numbers

        if (t >= max_counter_drop) {
            this->recovery_enabled = false;
            return false;
        } else {
            // timeout not reached yet
            return true;
        }
    }
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

void
SystemStatusManagerNode::RCFlagSummaryCallback(
        const nif::common::msgs::RCFlagSummary::UniquePtr msg) {
    this->rc_flag_summary = std::move(*msg);
    this->rc_flag_summary_update_time = this->now();
    this->has_rc_flag_summary = true;
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
        }
        return result;
    }
}
