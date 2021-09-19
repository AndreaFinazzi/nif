/**
 * @brief provides an interface for novatel oem7 odometry
 **/
#ifndef BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_BESTPOS_H_
#define BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_BESTPOS_H_

// Application
#include "bvs_utils/geodetic_conv.h"
#include "bvs_utils/message_sync.h"
#include "bvs_localization/estimate/estimate_source.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/heading2.hpp"

namespace bvs_localization {
namespace sources {

/**
 * @brief the interface for novatel oem7 bestpos / bestvel messages
 **/
class NovatelOem7BESTPOS : public estimate::EstimateSource {
public:
    const estimate::EstimateSource::PubType pub_type 
        = estimate::EstimateSource::PubType::Odom;

    /**
     * @brief create an source using novatel oem7 inspva messages
     **/
    NovatelOem7BESTPOS(
        rclcpp::Node::SharedPtr node,
        std::string source_name,
        std::string topic_bestpos,
        double health_lim_stddev,
        double health_lim_age,
        bvs_utils::GeodeticConverter::SharedPtr frame_converter
    );

    /**
     * @brief callback for new BESTPOS messages
     **/
    void bestposCB(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr message);

    /**
     * @brief provide the latest estimate and its status
     **/
    estimate::EstimateStatus getEstimate(estimate::Estimate& estimate);

private:
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr bestpos_sub_;
    bvs_utils::GeodeticConverter::SharedPtr frame_converter_;
    estimate::Estimate latest_estimate_;

    double health_lim_stddev_;
    rclcpp::Duration health_lim_age_;

    double lat_stddev_;
    double lon_stddev_;
    rclcpp::Time received_time_;

};


} /* namespace sources */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_BESTPOS_H_ */