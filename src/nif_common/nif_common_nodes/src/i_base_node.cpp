//
// Created by usrg on 6/24/21.
//

#include "nif_common_nodes/i_base_node.h"

using namespace nif::common;

IBaseNode::IBaseNode() :
        Node("no_name_node")
{
    throw std::invalid_argument("Cannot construct IBaseNode without specifying node_name. Creating empty node.");
}


IBaseNode::IBaseNode(const std::string &node_name, const rclcpp::NodeOptions &options) :
        Node(node_name, options)
{
    gclock_node_init = this->now();

//                TODO : Define QoS macros
    this->system_state_sub = this->create_subscription<nif::common::SystemState>("", 5, std::bind(&IBaseNode::systemStateCallback, this, std::placeholders::_1));
    this->race_control_state_sub = this->create_subscription<nif::common::RaceControlState>("", 5, std::bind(&IBaseNode::raceControlStateCallback, this, std::placeholders::_1));
    this->ego_vehicle_state_sub = this->create_subscription<nif::common::t_vehicle_kinematic_state>("", 5, std::bind(&IBaseNode::egoVehicleStateCallback, this, std::placeholders::_1));
}

