//
// Created by usrg on 6/22/21.
//

#ifndef NIF_COMMON_NODES_UTILS_GEOMETRY_H
#define NIF_COMMON_NODES_UTILS_GEOMETRY_H

#include <rclcpp/parameter.hpp>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"

namespace nif {
    namespace common {
        namespace utils {
            namespace geometry {

                const float MPH_TO_KPH_FACTOR = 1.60934;
                const float KPH_TO_MPH_FACTOR = 0.621371;

                /**
                 * Calculate Euclidean distance between Point/Pose/PoseStamped
                 * @param a Point/Pose/PoseStamped
                 * @param b Point/Pose/PoseStamped
                 * @return Euclidean distance between Point/Pose/PoseStamped
                 */
                double calEuclideanDistance(const geometry_msgs::msg::PoseStamped &a,
                                            const geometry_msgs::msg::PoseStamped &b);

                /**
                 * Calculate Euclidean distance between Point/Pose/PoseStamped
                 * @param a Point/Pose/PoseStamped
                 * @param b Point/Pose/PoseStamped
                 * @return Euclidean distance between Point/Pose/PoseStamped
                 */
                double calEuclideanDistance(const geometry_msgs::msg::Pose &a, const geometry_msgs::msg::Pose &b);

                /**
                 * Calculate Euclidean distance between Point/Pose/PoseStamped
                 * @param a Point/Pose/PoseStamped
                 * @param b Point/Pose/PoseStamped
                 * @return Euclidean distance between Point/Pose/PoseStamped
                 */
                double calEuclideanDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b);

//    TODO: fix static override problems
//    double calEuclideanDistance(const std::vector<double,double>& a, const std::vector<double,double>& b);

                /**
                 * Convert linear velocity from miles-per-hour to kilometers-per-hour
                 * @param mph Linear velocity in miles-per-hour
                 * @return Linear velocity in kilometers-per-hour
                 */
                double mph2kph(double mph);

                /**
                 * Convert linear velocity from kilometers-per-hour to miles-per-hour
                 * @param mph Linear velocity in kilometers-per-hour
                 * @return Linear velocity in miles-per-hour
                 */
                double kph2mph(double kph);


            }

            namespace io {
//  TODO: define precisely (do we need this?)
                std::vector<rclcpp::Parameter> &readConfig(std::string &file_name);

            }

// TODO: numeric is an awful name, needs something better.
            namespace numeric {

//    TODO: provide exaustive description of these two functions, along wth teir params' description.
//    std::tuple<min_value, min_value_idx> findMinValueNIdx(std::vector<double>& vec);
//    std::tuple<max_value, max_value_idx> findMaxValueNIdx(std::vector<double>& vec);

//  TODO: define precisely
                double clip(double min, double max, double target);
            }

        }
    }
}

#endif // NIF_COMMON_NODES_UTILS_GEOMETRY_H
