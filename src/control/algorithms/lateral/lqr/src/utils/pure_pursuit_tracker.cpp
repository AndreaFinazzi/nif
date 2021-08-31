#include "utils/pure_pursuit_tracker.h"

namespace nif {
    namespace control {
        namespace lateral {
            namespace lqr {
                namespace utils {
                    double pursuit_dist(const nav_msgs::msg::Odometry &point_a,
                                        const nav_msgs::msg::Odometry &point_b) {
                        return std::sqrt(
                                std::pow(point_a.pose.pose.position.x - point_b.pose.pose.position.x, 2) +
                                std::pow(point_a.pose.pose.position.y - point_b.pose.pose.position.y, 2));
                    }

                    double pursuit_dist(
                            const geometry_msgs::msg::PoseStamped &point_a,
                            const nav_msgs::msg::Odometry &point_b) {
                        return std::sqrt(
                                std::pow(point_a.pose.position.x - point_b.pose.pose.position.x, 2) +
                                std::pow(point_a.pose.position.y - point_b.pose.pose.position.y, 2));
                    }

                    double pursuit_dist(
                            const geometry_msgs::msg::PoseStamped &point_a,
                            const geometry_msgs::msg::PoseStamped &point_b) {
                        return std::sqrt(
                                std::pow(point_a.pose.position.x - point_b.pose.position.x, 2) +
                                std::pow(point_a.pose.position.y - point_b.pose.position.y, 2));
                    }

                    double smoothSignal(
                            double current_signal,
                            double target_signal,
                            double delta_dt,
                            double dt
                    ) {
                        if (target_signal > current_signal && target_signal > current_signal + delta_dt * dt) {
                            return current_signal + delta_dt * dt;
                        }
                        if (target_signal < current_signal && target_signal < current_signal - delta_dt * dt) {
                            return current_signal - delta_dt * dt;
                        }
                        return target_signal;
                    }
                }
            }
        }
    }
}
