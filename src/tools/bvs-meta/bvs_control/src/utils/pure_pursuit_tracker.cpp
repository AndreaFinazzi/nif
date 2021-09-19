#include "bvs_control/utils/pure_pursuit_tracker.h"

#include <cmath>

namespace bvs_control {
namespace utils {

template<> double
pursuit_dist<nav_msgs::msg::Odometry, nav_msgs::msg::Odometry>(
    nav_msgs::msg::Odometry& point_a,
    nav_msgs::msg::Odometry& point_b
) {
    return std::sqrt(
        std::pow(point_a.pose.pose.position.x - point_b.pose.pose.position.x, 2)
        + std::pow(point_a.pose.pose.position.y - point_b.pose.pose.position.y, 2)
    );
}

template<> double
pursuit_dist<geometry_msgs::msg::PoseStamped, nav_msgs::msg::Odometry>(
    geometry_msgs::msg::PoseStamped& point_a,
    nav_msgs::msg::Odometry& point_b
) {
    return std::sqrt(
        std::pow(point_a.pose.position.x - point_b.pose.pose.position.x, 2)
        + std::pow(point_a.pose.position.y - point_b.pose.pose.position.y, 2)
    );
}

template<> double
pursuit_dist<geometry_msgs::msg::PoseStamped, geometry_msgs::msg::PoseStamped>(
    geometry_msgs::msg::PoseStamped& point_a,
    geometry_msgs::msg::PoseStamped& point_b
) {
    return std::sqrt(
        std::pow(point_a.pose.position.x - point_b.pose.position.x, 2)
        + std::pow(point_a.pose.position.y - point_b.pose.position.y, 2)
    );
}

double smoothSignal(
    double current_signal,
    double target_signal, 
    double delta_dt,
    double dt
) {
    if(target_signal > current_signal && target_signal > current_signal + delta_dt * dt) {
        return current_signal + delta_dt * dt;
    }
    if(target_signal < current_signal && target_signal < current_signal - delta_dt * dt) {
        return current_signal - delta_dt * dt;
    }
    return target_signal;
}

} /* namespace utils */
} /* namespace bvs_control */