//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Andrea Finazzi

#include "nif_system_status_manager_nodes/system_status_manager_node.h"

using nif::system::SystemStatusManagerNode;

SystemStatusManagerNode::SystemStatusManagerNode(
        const std::string &node_name)
        : Node(node_name) {
//    Initialize to false
    this->system_status_msg.health_status.system_failure = false;

    this->declare_parameter("timeout_node_inactive_ms", 1000);

    this->node_inactive_timeout = rclcpp::Duration(1, 0);

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
            "/novatel_top/bestpos", 1, std::bind(&SystemStatusManagerNode::receive_bestpos, this, std::placeholders::_1));

    // TODO this is only temporary, as the localization status should be handled in the localization nodes.
    this->subscriber_insstdev = this->create_subscription<novatel_oem7_msgs::msg::INSSTDEV>(
            "/novatel_top/insstdev", 1, std::bind(&SystemStatusManagerNode::receive_insstdev, this, std::placeholders::_1));

    //  Publishers
    this->system_status_pub = this->create_publisher<nif::common::msgs::SystemStatus>(
            "out_system_status", nif::common::constants::QOS_INTERNAL_STATUS
    );

    this->joy_emergency_pub = this->create_publisher<std_msgs::msg::Bool>(
            "/ssm/emergency_joystick", 10);
    this->hb_emergency_pub = this->create_publisher<std_msgs::msg::Bool>(
            "/ssm/emergency_heartbeat", 10);
    this->diagnostic_hb_pub = this->create_publisher<std_msgs::msg::Int32>(
            "/ssm/heartbeat", 10);

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

    this->joy_emergency_pub->publish({this->system_status_msg.health_status.commanded_stop});
    this->hb_emergency_pub->publish({this->system_status_msg.health_status.communication_failure});
    this->system_status_pub->publish(this->system_status_msg);

    // send diagnostic hb to vehicle interface
    this->diagnostic_hb_pub->publish({static_cast<size_t>(counter_hb)});
    counter_hb++;
    if (counter_hb == 8){
        counter_hb = 0;
    }
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
            this->status_by_name.find(request->node_name) == this->status_by_name.end() &&
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
    if (this->now() - msg->stamp_last_update >= this->node_inactive_timeout)
    {
    // TODO implement check on last_update_stamp to detect inactive nodes.
        msg->node_status_code = common::NODE_INACTIVE;
    }
    auto index = this->status_by_id[msg->node_id];
    this->node_status_records[index]->node_status = msg;
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
        if (record->node_status &&
            record->node_status->node_status_code != common::NodeStatusCode::NODE_NOT_INITIALIZED) {
            // Check if last update is recent enough
            if ((this->now() - record->node_status->stamp).nanoseconds() <=
                std::chrono::duration_cast<std::chrono::nanoseconds>(nif::system::NODE_DEAD_TIMEOUT_US).count()) {
                record->node_status->node_status_code = common::NODE_DEAD;
            }
        }
    }
}

nif::common::types::t_node_id SystemStatusManagerNode::newStatusRecord(
        const nif_msgs::srv::RegisterNodeStatus::Request::SharedPtr request) {
    nif::common::types::t_node_id node_id;
    node_id = next_node_id++;

    {
        // Add the calling node to the registry of watched nodes
        auto node_status_record_ptr = std::make_unique<NodeStatusRecord>();
        node_status_record_ptr->node_status = std::make_shared<nif_msgs::msg::NodeStatus>();
        node_status_record_ptr->node_status->node_id = node_id;
        node_status_record_ptr->node_status->node_status_code = common::NODE_NOT_INITIALIZED;
        this->node_status_records.push_back(std::move(node_status_record_ptr));
    }
    auto node_index = this->node_status_records.size() - 1;

    this->system_status_msg.health_status.node_list.push_back(request->node_name);

    this->status_by_id.insert({
                                      node_id, node_index
                              });

    this->status_by_name.insert(
            {
                    std::move(request->node_name),
                    node_index
            });

    auto node_type = static_cast<const NodeType>(request->node_type);
    auto by_type_registry = this->statuses_by_type.find(node_type);
    // Key is not present
    if (by_type_registry == this->statuses_by_type.end()) {
        auto type_vector = std::make_unique<std::vector<t_record_index>>();
        type_vector->push_back(node_index);
        this->statuses_by_type.insert({
                                              node_type,
                                              std::move(type_vector)
                                      });
    } else {
        this->statuses_by_type[node_type]->push_back(node_index);
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
        const nif::common::msgs::RCFlagSummary::UniquePtr msg)
{
    this->rc_flag_summary = std::move(*msg);
    this->rc_flag_summary_update_time = this->now();
    this->has_rc_flag_summary = true;
}
