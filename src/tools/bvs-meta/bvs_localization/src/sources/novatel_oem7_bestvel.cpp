#include "bvs_localization/sources/novatel_oem7_bestvel.h"

namespace bvs_localization {
namespace sources {

NovatelOem7BESTVEL::NovatelOem7BESTVEL(
    rclcpp::Node::SharedPtr node,
    std::string source_name,
    std::string topic_bestvel,
    std::string topic_inspva,
    bool orient_to_inspva,
    double health_lim_age,
    bvs_utils::GeodeticConverter::SharedPtr frame_converter
) : EstimateSource(node, source_name),
    message_sync_(0.01),
    health_lim_age_(static_cast<uint32_t>(health_lim_age * 1e9)),
    orient_to_inspva_(orient_to_inspva)
{
    frame_converter_ = frame_converter;
    bestvel_sub_ = node->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(topic_bestvel,
                    1, std::bind(&NovatelOem7BESTVEL::bestvelCB, this, std::placeholders::_1));
    if(orient_to_inspva) {
        inspva_sub_ = node->create_subscription<novatel_oem7_msgs::msg::INSPVA>(topic_inspva,
                        1, std::bind(&NovatelOem7BESTVEL::inspvaCB, this, std::placeholders::_1));
    
        message_sync_.registerCallback(std::bind(&NovatelOem7BESTVEL::syncedCB, this, std::placeholders::_1, std::placeholders::_2));
    }

    // Set which fields this estimate can provide
    latest_estimate_.valid_fields = 
        estimate::Estimate::Fields::VELOCITY_X
        | estimate::Estimate::Fields::VELOCITY_Y
        | estimate::Estimate::Fields::VELOCITY_Z;
}

void 
NovatelOem7BESTVEL::bestvelCB(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr message) {
    auto stamp = frame_converter_->weekMSToEpoch(message->nov_header.gps_week_number, message->nov_header.gps_week_milliseconds);
    if(orient_to_inspva_) {
        message_sync_.addMessage(stamp, *message);
    } else {
        received_time_ = rclcpp::Clock().now();
        latest_estimate_.velocity.linear_x = message->hor_speed;
        latest_estimate_.velocity.linear_y = 0.;
        latest_estimate_.velocity.linear_z = message->ver_speed;
    }
}

void 
NovatelOem7BESTVEL::inspvaCB(const novatel_oem7_msgs::msg::INSPVA::SharedPtr message) {
    auto stamp = frame_converter_->weekMSToEpoch(message->nov_header.gps_week_number, message->nov_header.gps_week_milliseconds);
    message_sync_.addMessage(stamp, *message);
}

void
NovatelOem7BESTVEL::syncedCB(
    const novatel_oem7_msgs::msg::BESTVEL& vel,
    const novatel_oem7_msgs::msg::INSPVA& pva
) {
    received_time_ = rclcpp::Clock().now();
    double yaw = frame_converter_->deg2Rad(vel.trk_gnd - pva.azimuth);
    latest_estimate_.velocity.linear_x = vel.hor_speed * std::cos(-yaw);
    latest_estimate_.velocity.linear_y = vel.hor_speed * std::sin(-yaw);
    latest_estimate_.velocity.linear_z = vel.ver_speed;
}

estimate::EstimateStatus
NovatelOem7BESTVEL::getEstimate(estimate::Estimate& estimate) {
    estimate = latest_estimate_;
    auto now = rclcpp::Clock().now();
    if (now - received_time_ > health_lim_age_) {
        return estimate::EstimateStatus::STALE;
    }
    return estimate::EstimateStatus::GOOD;
}

} /* namespace sources */
} /* namespace bvs_localization */