#include "bvs_utils/path_util.h"

#include <stdexcept>
#include <fstream>
#include <limits>
#include <cmath>

#include <iomanip>
#include <iostream>

namespace bvs_utils {

PathUtil::PathUtil()
{}

void
PathUtil::setConverter(GeodeticConverter& converter) {
    converter_ = converter;
}

void
PathUtil::loadPath(std::string csv_path, std::string frame_id, bool transform_to_ltp) {
    if(transform_to_ltp && !converter_.isInitialised()) {
        throw std::runtime_error("Unable to transform to ltp when converter_ is not properly initialized");
    }

    // First read in all the points from the CSV
    std::ifstream ifs(csv_path, std::ifstream::in);
    nav_msgs::msg::Path new_path;

    if(ifs.is_open()) {
        std::string line;
        while(std::getline(ifs, line)) {
            std::istringstream iss(line);
            double x, y, z = 0;
            char comma;
            if(!(iss >> x >> comma >> y)) {
                break;
            }

            if(transform_to_ltp) {
                // Transform to NED first
                GeodeticConverter::GeoRef r;
                GeodeticConverter::CartesianPoint p;
                r.latitude = x;
                r.longitude = y;
                r.altitude = z;
                converter_.geodetic2Ned(r, p);
                // Tranform to FLU
                x = p.x;
                y = -p.y;
                z = -p.z;
            }
            geometry_msgs::msg::PoseStamped new_pt;
            new_pt.header.frame_id = frame_id;
            new_pt.pose.position.x = x;
            new_pt.pose.position.y = y;
            new_pt.pose.position.z = z;
            new_path.poses.push_back(new_pt);
        }
    }

    // After all the points are loaded we go back and compute yaw
    for(unsigned int i = 0; i < new_path.poses.size(); ++i) {
        new_path.poses[i].pose.orientation = computePointOrientation(new_path, i);
    }
    path_ = new_path;
}

void PathUtil::smoothPath() {

}

nav_msgs::msg::Path
PathUtil::wrapPathTo(geometry_msgs::msg::Pose pose, int pose_limit) {
    auto idx = getClosestPoint(pose);

    nav_msgs::msg::Path result;
    unsigned int pose_count = path_.poses.size();
    if(pose_limit > 0) pose_count = static_cast<unsigned int>(pose_limit);
    if(pose_limit < 0) pose_count-= 100;
    for(unsigned int i = 0; i < pose_count; ++i) {
        unsigned int cidx = (i + idx) % (path_.poses.size());
        result.poses.push_back(path_.poses[cidx]);
    }
    return result;
}

nav_msgs::msg::Path&
PathUtil::getPath() {
    return path_;
}

unsigned int
PathUtil::getClosestPoint(geometry_msgs::msg::Pose pose) {
    // This could be better? (do it like pure pursuit)
    double dist = std::numeric_limits<double>::max();
    unsigned int cidx = 0;
    for(unsigned int i = 0; i < path_.poses.size(); ++i) {
        double dist_p = std::sqrt(
            std::pow(path_.poses[i].pose.position.x - pose.position.x, 2)
            + std::pow(path_.poses[i].pose.position.y - pose.position.y, 2)
        );
        if(dist_p < dist) {
            dist = dist_p;
            cidx = i;
        }
    }
    return cidx;
}

geometry_msgs::msg::Quaternion
PathUtil::computePointOrientation(nav_msgs::msg::Path& path, unsigned int idx) {
    double yaw;
    // try to smooth out the yaw (this path is really rough)
    unsigned int c = 10;
    unsigned int idx_up = (idx + c)%path.poses.size();
    unsigned int idx_down = (path.poses.size() + idx - c)%path.poses.size();

    yaw = std::atan2(
        path.poses[idx_up].pose.position.y - path.poses[idx_down].pose.position.y,
        path.poses[idx_up].pose.position.x - path.poses[idx_down].pose.position.x
    );

    geometry_msgs::msg::Quaternion result;
    result.x = 0.;
    result.y = 0.;
    result.z = std::sin(yaw / 2.);
    result.w = std::cos(yaw / 2.);
    return result;
}

} /* namespace bvs_utils */
