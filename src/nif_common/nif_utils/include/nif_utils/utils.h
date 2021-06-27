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

    double calEuclideanDistance(const geometry_msgs::msg::PoseStamped& a, const geometry_msgs::msg::PoseStamped& b);

    double calEuclideanDistance(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b);

    double calEuclideanDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

    double calEuclideanDistance(const std::vector<double,double>& a, const std::vector<double,double>& b);

    double mph2kph(double mph);

    double kph2mph(double kph);


}

namespace io {
//  TODO: define precisely (do we need this?)
    std::vector<rclcpp::Parameter> & readConfig(std::string& file_name);

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
