#ifndef BVS_LOCALIZATION_ESTIMATE_ESTIMATE_SOURCE_H_
#define BVS_LOCALIZATION_ESTIMATE_ESTIMATE_SOURCE_H_

// Application
#include "bvs_localization/estimate/estimate.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/u_int8.hpp"

// STD
#include <cstdint>

namespace bvs_localization {
namespace estimate {

class EstimateSource {
public:
    //! used to determine which type to publish
    enum PubType : uint8_t {
        Odom = 0,
        Twist = 1
    };
    // const PubType pub_type = PubType::Odom;

    typedef std::shared_ptr<EstimateSource> SharedPtr;

    /**
     * Creates the interface needed for an estimate source
     **/
    EstimateSource(
        rclcpp::Node::SharedPtr node,
        std::string source_name,
        PubType pub_type = PubType::Odom
    );
    
    /**
     * @brief publishes out state and status for given interfaces
     **/
    void publish(Estimate estimate, EstimateStatus status);

    virtual EstimateStatus getEstimate(Estimate& estimate) = 0;

protected:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr status_pub_;
    std::string ltp_frame_;
    std::string vehicle_frame_;

    PubType pub_type_;

};

} /* namespace estimate */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_ESTIMATE_ESTIMATE_SOURCE_H_ */