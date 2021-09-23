/**
 * @brief defines the standard interface for an estimate
 **/
#ifndef BVS_LOCALIZATION_ESTIMATE_ESTIMATE_H_
#define BVS_LOCALIZATION_ESTIMATE_ESTIMATE_H_

// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

// STD
#include <memory>
#include <cstdint>


namespace bvs_localization {
namespace estimate {

//! Possible values for estimate status
enum EstimateStatus : uint8_t {
    GOOD = 0,
    BAD = 1,
    STALE = 2,
    UNKNOWN = 100
};

struct Estimate {
    //! Encode position (xyz)
    struct Point {
        double x = 0.;
        double y = 0.;
        double z = 0.;
    };
    //! Encodes orientation (quaternion expression)
    struct Quaternion {
        double x = 0.;
        double y = 0.;
        double z = 0.;
        double w = 1.;
    };
    //! Velocity Estimate (linear velocity only)
    struct Velocity {
        double linear_x = 0.;
        double linear_y = 0.;
        double linear_z = 0.;
    };
    
    //! The invidiual fields that can be estimated by different sources
    //! EG: We can take position from one source and orientation from another
    //!     but we can't take position.x from one and position.y from another
    enum Fields : uint8_t {
        POSITION    = 0b00000001,
        ORIENTATION = 0b00000010,
        VELOCITY_X  = 0b00000100,
        VELOCITY_Y  = 0b00001000,
        VELOCITY_Z  = 0b00010000
    };
    //! Which fields are valid (uses above Fields bitmapping)
    unsigned int valid_fields;
    //! Time of the estimate
    double estimate_time;

    //! Field values
    Point position;
    Quaternion orientation;
    Velocity velocity;

    nav_msgs::msg::Odometry toOdom() {
        nav_msgs::msg::Odometry result;
        
        result.header.stamp = rclcpp::Time(static_cast<uint64_t>(estimate_time * 1e9));
        result.pose.pose.position.x = position.x;
        result.pose.pose.position.y = position.y;
        result.pose.pose.position.z = position.z;
        result.pose.pose.orientation.x = orientation.x;
        result.pose.pose.orientation.y = orientation.y;
        result.pose.pose.orientation.z = orientation.z;
        result.pose.pose.orientation.w = orientation.w;
        result.twist.twist.linear.x = velocity.linear_x;
        result.twist.twist.linear.y = velocity.linear_y;
        result.twist.twist.linear.z = velocity.linear_z;
        return result;
    };
};

} /* namespace sources */
} /* namespace bvs_localization */

#endif /* BVS_LOCALIZATION_ESTIMATE_ESTIMATE_H_ */