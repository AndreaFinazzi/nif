/**
 * @brief provides an interface for novatel oem7 INSPVA
 **/
#ifndef BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_INSPVA_H_
#define BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_INSPVA_H_

// Application
#include "bvs_utils/geodetic_conv.h"
#include "bvs_localization/estimate/estimate_source.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"

namespace bvs_localization {
namespace sources {

/**
 * @brief the interface for novatel oem7 inspva messages
 **/
class NovatelOem7INSPVA : public estimate::EstimateSource {
public:
    const estimate::EstimateSource::PubType pub_type 
        = estimate::EstimateSource::PubType::Odom;

    /**
     * @brief create an source using novatel oem7 inspva messages
     **/
    NovatelOem7INSPVA(
        rclcpp::Node::SharedPtr node,
        std::string source_name,
        std::string gps_topic_name,
        double health_lim_age,
        bvs_utils::GeodeticConverter::SharedPtr frame_converter
    );

    /**
     * @brief callback for new INSPVA messages
     **/
    void inspvaCB(const novatel_oem7_msgs::msg::INSPVA::SharedPtr message);

    /**
     * @brief provide the latest estimate and its status
     **/
    estimate::EstimateStatus getEstimate(estimate::Estimate& estimate);

private:
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr inspva_sub_;
    bvs_utils::GeodeticConverter::SharedPtr frame_converter_;
    estimate::Estimate latest_estimate_;

    rclcpp::Duration health_lim_age_;
    rclcpp::Time received_time_;
};


} /* namespace sources */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_INSPVA_H_ */