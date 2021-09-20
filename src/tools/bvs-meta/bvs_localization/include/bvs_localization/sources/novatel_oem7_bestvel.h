/**
 * @brief provides an interface for novatel oem7 BESTVEL messages
 * 
 * @note the BESTVEL message requires orientation to be put in the
 *  body frame - we use INSPVA to get this orientation
 **/
#ifndef BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_BESTVEL_H_
#define BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_BESTVEL_H_

// Application
#include "bvs_utils/geodetic_conv.h"
#include "bvs_utils/message_sync.h"
#include "bvs_localization/estimate/estimate_source.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/inspva.hpp"

namespace bvs_localization {
namespace sources {

/**
 * @brief the interface for novatel oem7 bestpos / bestvel messages
 **/
class NovatelOem7BESTVEL : public estimate::EstimateSource {
public:
    const estimate::EstimateSource::PubType pub_type 
        = estimate::EstimateSource::PubType::Odom;

    /**
     * @brief create an source using novatel oem7 inspva messages
     **/
    NovatelOem7BESTVEL(
        rclcpp::Node::SharedPtr node,
        std::string source_name,
        std::string topic_bestvel,
        std::string topic_inspva,
        bool orient_to_inspva,
        double health_lim_age,
        bvs_utils::GeodeticConverter::SharedPtr frame_converter
    );

    /**
     * @brief callback for new BESTVEL messages
     **/
    void bestvelCB(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr message);

    /**
     * @brief callback for new BESTPOS messages
     **/
    void inspvaCB(const novatel_oem7_msgs::msg::INSPVA::SharedPtr message);

    /**
     * @brief callback for synced pos / vel messages
     * @param pos the position message
     * @param vel the velocity message
     **/
    void syncedCB(
        const novatel_oem7_msgs::msg::BESTVEL& pos,
        const novatel_oem7_msgs::msg::INSPVA& vel
    );

    /**
     * @brief provide the latest estimate and its status
     **/
    estimate::EstimateStatus getEstimate(estimate::Estimate& estimate);

private:
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr bestvel_sub_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSPVA>::SharedPtr inspva_sub_;
    bvs_utils::GeodeticConverter::SharedPtr frame_converter_;
    estimate::Estimate latest_estimate_;
    bvs_utils::MessageSync<novatel_oem7_msgs::msg::BESTVEL, novatel_oem7_msgs::msg::INSPVA> message_sync_;

    rclcpp::Duration health_lim_age_;
    rclcpp::Time received_time_;
    bool orient_to_inspva_;

};


} /* namespace sources */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_SOURCES_NOVATEL_OEM7_BESTPOS_H_ */