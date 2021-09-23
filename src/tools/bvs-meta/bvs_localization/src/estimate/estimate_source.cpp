#include "bvs_localization/estimate/estimate_source.h"

namespace bvs_localization {
namespace estimate {

EstimateSource::EstimateSource(
    rclcpp::Node::SharedPtr node,
    std::string source_name,
    EstimateSource::PubType pub_type
) : pub_type_(pub_type) {

    if(pub_type_ == PubType::Odom)
        odom_pub_ = node->create_publisher<nav_msgs::msg::Odometry>("bvs_localization/" + source_name + "/odom", 1);
    else if(pub_type_ == PubType::Twist)
        twist_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("bvs_localization/" + source_name + "/twist", 1);
    status_pub_ = node->create_publisher<std_msgs::msg::UInt8>("bvs_localization/" + source_name + "/status", 1);

    ltp_frame_ = node->get_parameter("ltp_frame").as_string();
    vehicle_frame_ = node->get_parameter("vehicle_frame").as_string();
}

void
EstimateSource::publish(Estimate estimate, EstimateStatus status) {
    std_msgs::msg::UInt8 status_msg;
    status_msg.data = status;
    status_pub_->publish(status_msg);

    auto odom = estimate.toOdom();
    odom.header.frame_id = ltp_frame_;
    odom.child_frame_id = vehicle_frame_;

    if(pub_type_ == PubType::Odom) {
        odom_pub_->publish(odom);
    }
    else if(pub_type_ == PubType::Twist) {
        twist_pub_->publish(odom.twist.twist);
    }
}


} /* namespace estimate */
} /* namespace bvs_localization */