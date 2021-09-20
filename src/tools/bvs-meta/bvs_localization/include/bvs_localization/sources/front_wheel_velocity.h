/**
 * @brief provides an interface for wheel speed
 **/
#ifndef BVS_LOCALIZATION_SOURCES_FRONT_WHEEL_VELOCITY_H_
#define BVS_LOCALIZATION_SOURCES_FRONT_WHEEL_VELOCITY_H_

// Application
#include "bvs_localization/estimate/estimate.h"
#include "bvs_localization/estimate/estimate_source.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"

namespace bvs_localization {
namespace sources {

using namespace estimate;

/**
 * @brief the interface for novatel oem7 inspva messages
 **/
class FrontWheelVelocity : public estimate::EstimateSource {
public:
    static constexpr double kph2ms = 1.0/3.6;
    
    /**
     * @brief create an source using novatel oem7 inspva messages
     **/
    FrontWheelVelocity(
        rclcpp::Node::SharedPtr node,
        std::string source_name,
        std::string topic_name
    );

    /**
     * @brief callback for new INSPVA messages
     **/
    void wheelVelocityCB(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr message);

    /**
     * @brief provide the latest estimate and its status
     **/
    estimate::EstimateStatus getEstimate(estimate::Estimate& estimate);

private:
    rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr wheel_speed_sub_;
    estimate::Estimate latest_estimate_;

};


} /* namespace sources */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_SOURCES_WHEEL_VELOCITY_H_ */