#include "bvs_localization/sources/novatel_oem7_inspva.h"

namespace bvs_localization {
namespace sources {

NovatelOem7INSPVA::NovatelOem7INSPVA(
    rclcpp::Node::SharedPtr node,
    std::string source_name,
    std::string gps_topic_name,
    double health_lim_age,
    bvs_utils::GeodeticConverter::SharedPtr frame_converter
) : EstimateSource(node, source_name),
    health_lim_age_(static_cast<uint32_t>(health_lim_age * 1e9))
{
    frame_converter_ = frame_converter;
    inspva_sub_ = node->create_subscription<novatel_oem7_msgs::msg::INSPVA>(gps_topic_name,
                    1, std::bind(&NovatelOem7INSPVA::inspvaCB, this, std::placeholders::_1));
                
    // Set which fields this estimate can provide
    latest_estimate_.valid_fields = 
        estimate::Estimate::Fields::POSITION 
        | estimate::Estimate::Fields::ORIENTATION
        | estimate::Estimate::Fields::VELOCITY_X
        | estimate::Estimate::Fields::VELOCITY_Y
        | estimate::Estimate::Fields::VELOCITY_Z;
}

void 
NovatelOem7INSPVA::inspvaCB(const novatel_oem7_msgs::msg::INSPVA::SharedPtr message) {
    received_time_ = rclcpp::Clock().now();
    // https://docs.novatel.com/OEM7/Content/SPAN_Logs/INSPVA.htm
    bvs_utils::GeodeticConverter::GeoRef ref;
    ref.latitude = message->latitude;
    ref.longitude = message->longitude;
    // Currently ignore altitude for the most part and just track x/y
    ref.altitude = 0.;

    bvs_utils::GeodeticConverter::CartesianPoint ned_pt;
    frame_converter_->geodetic2Ned(ref, ned_pt);

    latest_estimate_.estimate_time = frame_converter_->weekMSToEpoch(message->nov_header.gps_week_number, message->nov_header.gps_week_milliseconds);
    latest_estimate_.position.x = ned_pt.x;
    latest_estimate_.position.y = -ned_pt.y;
    latest_estimate_.position.z = -ned_pt.z;
    latest_estimate_.orientation.x = 0.;
    latest_estimate_.orientation.y = 0.;
    double yaw = frame_converter_->deg2Rad(-message->azimuth);
    latest_estimate_.orientation.z = std::sin(yaw / 2.);
    latest_estimate_.orientation.w = std::cos(yaw / 2.);
    latest_estimate_.velocity.linear_x = message->north_velocity * std::cos(-yaw) + message->east_velocity * std::sin(-yaw);
    latest_estimate_.velocity.linear_y = message->north_velocity * std::sin(-yaw) - message->east_velocity * std::cos(-yaw);
    latest_estimate_.velocity.linear_z = message->up_velocity;
}

estimate::EstimateStatus
NovatelOem7INSPVA::getEstimate(estimate::Estimate& estimate) {
    estimate = latest_estimate_;
    auto now = rclcpp::Clock().now();
    if (now - received_time_ > health_lim_age_) {
        return estimate::EstimateStatus::STALE;
    }
    return estimate::EstimateStatus::GOOD;
}

} /* namespace sources */
} /* namespace bvs_localization */