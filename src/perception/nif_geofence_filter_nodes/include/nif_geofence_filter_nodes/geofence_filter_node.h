
/*
 * geofence_filter_node.cpp
 *
 *  Created on: Aug 10, 2021
 *      Author: Andrea Finazzi for USRG @ KAIST
 */


#ifndef NIF_GEOFENCE_FILTER_NODE_H
#define NIF_GEOFENCE_FILTER_NODE_H

#include <numeric>
#include "nif_utils/utils.h"
#include "nif_common/types.h"
#include "nif_waypoint_manager_minimal/waypoint_manager_minimal.h"
#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace nif {
namespace perception {

class GeofenceFilterNode : public nif::common::IBaseNode {

public:

    GeofenceFilterNode(const std::string &node_name_);

    ~GeofenceFilterNode() {}

private:

    void perceptionArrayCallback(
        const nif_msgs::msg::Perception3DArray::SharedPtr msg);
    void radarTrackCallback(
        const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg);

    bool poseInBodyIsValid(
        const geometry_msgs::msg::Pose &point_in_body);

    rclcpp::Subscription<nif_msgs::msg::Perception3DArray>::SharedPtr sub_perception_array;
    rclcpp::Publisher<nif_msgs::msg::Perception3DArray>::SharedPtr pub_filtered_perception_array;

    rclcpp::Subscription<delphi_esr_msgs::msg::EsrTrack>::SharedPtr sub_radar_track;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrTrack>::SharedPtr pub_filtered_radar_track;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_filtered_radar_track_vis;

    std::unique_ptr<WaypointManagerMinimal> wpt_manager_inner;
    std::unique_ptr<WaypointManagerMinimal> wpt_manager_outer;

    bool distance_filter_active; 
    double distance_filter_threshold_m; 
    double distance_filter_threshold_m_squared; 

    bool boundaries_filter_active; 
    double boundaries_filter_threshold_m; 
    double boundaries_filter_threshold_m_squared; 

};

} // namespace perception
} // namespace nif

#endif // NIF_GEOFENCE_FILTER_NODE_H