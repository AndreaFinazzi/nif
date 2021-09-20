#include "bvs_localization/sources/front_wheel_velocity.h"

namespace bvs_localization {
namespace sources {

FrontWheelVelocity::FrontWheelVelocity(
    rclcpp::Node::SharedPtr node,
    std::string source_name,
    std::string topic_name
) : EstimateSource(node, source_name, EstimateSource::PubType::Twist) {
    wheel_speed_sub_ = node->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(topic_name,
                    1, std::bind(&FrontWheelVelocity::wheelVelocityCB, this, std::placeholders::_1));
                
    latest_estimate_.valid_fields = estimate::Estimate::Fields::VELOCITY_X;
}

void
FrontWheelVelocity::wheelVelocityCB(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
    latest_estimate_.velocity.linear_x = (msg->front_left + msg->front_right)*0.5*kph2ms;
}

estimate::EstimateStatus
FrontWheelVelocity::getEstimate(estimate::Estimate& estimate) {
    estimate = latest_estimate_;
    return estimate::EstimateStatus::GOOD;
}

} /* namespace sources */
} /* namespace bvs_localization */