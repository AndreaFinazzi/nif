//
// Created by usrg on 9/17/21.
//

/**
 * Interface to handle race control spoofing
 */
#ifndef ROS2MASTER_RC_INTERFACE_NODE_H
#define ROS2MASTER_RC_INTERFACE_NODE_H

#include "nif_common/types.h"

using nif::common::msgs::RCFlagSummary;
using nif::common::msgs::OverrideRCFlagSummary;

namespace nif {


class RCInterfaceNode : public rclcpp::Node {
public:
    RCInterfaceNode(const std::string & node_name)
        : Node(node_name)
    {
        this->rc_flag_summary_pub = this->create_publisher<nif::common::msgs::RCFlagSummary>("rc_interface/rc_flag_summary", nif::common::constants::QOS_RACE_CONTROL);
        std::weak_ptr<std::remove_pointer<decltype(this->rc_flag_summary_pub.get())>::type> captured_pub = this->rc_flag_summary_pub;

        this->declare_parameter("listen_to_override", true);
        this->declare_parameter("listen_to_nominal", false);

        this->listen_to_override = this->get_parameter("listen_to_override").as_bool();
        this->listen_to_nominal = this->get_parameter("listen_to_nominal").as_bool();

        RCLCPP_INFO(this->get_logger(), "RC Interface listen_to_override: %s", this->listen_to_override ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "RC Interface listen_to_nominal: %s", this->listen_to_nominal ? "true" : "false");

        // Handle race flags from override (spoofed).
        auto override_callback = [this, captured_pub](OverrideRCFlagSummary::SharedPtr msg) -> void
                {
                    auto pub_ptr = captured_pub.lock();
                    if (!pub_ptr) {
                        return;
                    }
                    auto rc_flag_summary = std::make_unique<RCFlagSummary>();
//                    TODO clean this out.
                    *rc_flag_summary = std::move(*msg);

                    if (this->listen_to_override)
                        pub_ptr->publish(std::move(rc_flag_summary));
                };

        // Handle race flags from race control
        auto nominal_callback = [this, captured_pub](RCFlagSummary::SharedPtr msg) -> void
                {
                    auto pub_ptr = captured_pub.lock();
                    if (!pub_ptr) {
                        return;
                    }
                    auto rc_flag_summary = std::make_unique<RCFlagSummary>();
                    *rc_flag_summary = std::move(*msg);

                    if (this->listen_to_nominal)
                        pub_ptr->publish(std::move(rc_flag_summary));
                };

        this->override_msg_sub =
                this->create_subscription<OverrideRCFlagSummary>(
                        "rc_to_ct/flag_summary", nif::common::constants::QOS_RACE_CONTROL,
                        override_callback);

        this->nominal_msg_sub =
                this->create_subscription<RCFlagSummary>(
                        "raptor_dbw_interface/flag_summary", nif::common::constants::QOS_RACE_CONTROL,
                        nominal_callback);
    }

private:
    bool listen_to_override;
    bool listen_to_nominal;

    rclcpp::Subscription<OverrideRCFlagSummary>::SharedPtr override_msg_sub;
    rclcpp::Subscription<RCFlagSummary>::SharedPtr nominal_msg_sub;

    rclcpp::Publisher<RCFlagSummary>::SharedPtr rc_flag_summary_pub;

};

}
#endif //ROS2MASTER_RC_INTERFACE_NODE_H
