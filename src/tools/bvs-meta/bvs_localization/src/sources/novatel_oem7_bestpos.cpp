#include "bvs_localization/sources/novatel_oem7_bestpos.h"

namespace bvs_localization {
namespace sources {

NovatelOem7BESTPOS::NovatelOem7BESTPOS(
    rclcpp::Node::SharedPtr node,
    std::string source_name,
    std::string topic_bestpos,
    double health_lim_stddev,
    double health_lim_age,
    bvs_utils::GeodeticConverter::SharedPtr frame_converter
) : EstimateSource(node, source_name),
    health_lim_stddev_(health_lim_stddev),
    health_lim_age_(static_cast<uint32_t>(health_lim_age * 1e9))
{
    frame_converter_ = frame_converter;
    bestpos_sub_ = node->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(topic_bestpos,
                    1, std::bind(&NovatelOem7BESTPOS::bestposCB, this, std::placeholders::_1));

    // Set which fields this estimate can provide
    latest_estimate_.valid_fields = estimate::Estimate::Fields::POSITION;
}

void 
NovatelOem7BESTPOS::bestposCB(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr message) {
    received_time_ = rclcpp::Clock().now();
    // https://docs.novatel.com/OEM7/Content/Logs/BESTPOS.htm
    bvs_utils::GeodeticConverter::GeoRef ref;
    ref.latitude = message->lat;
    ref.longitude = message->lon;
    // Currently ignore altitude for the most part and just track x/y
    ref.altitude = 0.;

    bvs_utils::GeodeticConverter::CartesianPoint ned_pt;
    frame_converter_->geodetic2Ned(ref, ned_pt);

    lat_stddev_ = message->lat_stdev; 
    lon_stddev_ = message->lon_stdev; 
    latest_estimate_.estimate_time = frame_converter_->weekMSToEpoch(message->nov_header.gps_week_number, message->nov_header.gps_week_milliseconds);
    latest_estimate_.position.x = ned_pt.x;
    latest_estimate_.position.y = -ned_pt.y;
    latest_estimate_.position.z = -ned_pt.z;
}

estimate::EstimateStatus
NovatelOem7BESTPOS::getEstimate(estimate::Estimate& estimate) {
    auto now = rclcpp::Clock().now();
    estimate = latest_estimate_;

    if(now - received_time_ > health_lim_age_) {
        return estimate::EstimateStatus::STALE;
    }
    if(lat_stddev_ > health_lim_stddev_ || lon_stddev_ > health_lim_stddev_) {
        return estimate::EstimateStatus::BAD;
    }
    return estimate::EstimateStatus::GOOD;
}

} /* namespace sources */
} /* namespace bvs_localization */