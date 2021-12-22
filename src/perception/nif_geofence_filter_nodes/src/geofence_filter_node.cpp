
/*
 * geofence_filter_node.cpp
 *
 *  Created on: Aug 10, 2021
 *      Author: Andrea Finazzi for USRG @ KAIST
 */

#include "nif_geofence_filter_nodes/geofence_filter_node.h"

using nif::perception::GeofenceFilterNode;

GeofenceFilterNode::GeofenceFilterNode(const std::string &node_name_)
    // IBaseNode with no SSM subscription
    : IBaseNode(node_name_, nif::common::NodeType::PERCEPTION, rclcpp::NodeOptions{}, false) { 

    this->declare_parameter<std::string>("file_path_inner_line", "");
    this->declare_parameter<std::string>("file_path_outer_line", "");

    this->declare_parameter<bool>("distance_filter_active", true);
    this->declare_parameter<double>("distance_filter_threshold_m", 0.5);

    this->declare_parameter<bool>("boundaries_filter_active", true);

    auto file_path_inner_line = this->get_parameter("file_path_inner_line").as_string();
    auto file_path_outer_line = this->get_parameter("file_path_outer_line").as_string();

    this->distance_filter_active = this->get_parameter("distance_filter_active").as_bool();
    this->distance_filter_threshold_m = this->get_parameter("distance_filter_threshold_m").as_double();
    this->distance_filter_threshold_m_squared = pow(this->distance_filter_threshold_m, 2);

    this->boundaries_filter_active = this->get_parameter("boundaries_filter_active").as_bool();

    sub_perception_array = this->create_subscription<nif_msgs::msg::Perception3DArray>(
        "in_perception_array", nif::common::constants::QOS_SENSOR_DATA,
        std::bind(&GeofenceFilterNode::perceptionArrayCallback, this,
                  std::placeholders::_1));

    pub_filtered_perception_array = this->create_publisher<nif_msgs::msg::Perception3DArray>(
        "out_filtered_perception_array", nif::common::constants::QOS_SENSOR_DATA);

    sub_radar_track = this->create_subscription<delphi_esr_msgs::msg::EsrTrack>(
        "in_radar_track", nif::common::constants::QOS_SENSOR_DATA,
        std::bind(&GeofenceFilterNode::radarTrackCallback, this,
                  std::placeholders::_1));

    pub_filtered_radar_track = this->create_publisher<delphi_esr_msgs::msg::EsrTrack>(
        "out_filtered_radar_track", nif::common::constants::QOS_SENSOR_DATA);

    pub_filtered_radar_track_vis = this->create_publisher<visualization_msgs::msg::Marker>(
        "out_filtered_radar_track_vis", nif::common::constants::QOS_SENSOR_DATA);

    
    int spline_interval = 1;

    // Load waypoints
    std::vector<std::string> inner_line_path_vector { file_path_inner_line }; // Pass single path
    this->wpt_manager_inner = std::make_unique<WaypointManagerMinimal>(
                        inner_line_path_vector,
                        this->getBodyFrameId(),
                        this->getGlobalFrameId(),
                        spline_interval);

    std::vector<std::string> outer_line_path_vector { file_path_outer_line }; // Pass single path
    this->wpt_manager_outer = std::make_unique<WaypointManagerMinimal>(
                        outer_line_path_vector,
                        this->getBodyFrameId(),
                        this->getGlobalFrameId(),
                        spline_interval);
}

void GeofenceFilterNode::perceptionArrayCallback(
    const nif_msgs::msg::Perception3DArray::SharedPtr msg) {

  nif_msgs::msg::Perception3DArray msg_out{};
  msg_out.header = msg->header;

  for (auto&& object : msg->perception_list) {
    if (this->poseInBodyIsValid(object.detection_result_3d.center))
      msg_out.perception_list.push_back(object);

  }
  
  this->pub_filtered_perception_array->publish(msg_out);
}

void GeofenceFilterNode::radarTrackCallback(
    const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg) {
      geometry_msgs::msg::Pose pose_in_body{};

      pose_in_body.position.x = msg->track_range * cos( 2 * M_PI * msg->track_angle / 360);
      pose_in_body.position.y = msg->track_range * sin( 2 * M_PI * msg->track_angle / 360);

      if (this->poseInBodyIsValid(pose_in_body))
      {
        visualization_msgs::msg::Marker track_vis{};
        track_vis.header = msg->header;
        track_vis.id = msg->track_id;
        track_vis.lifetime = rclcpp::Duration(0, 0.1e+9);

        track_vis.pose = pose_in_body;

        track_vis.scale.x = 1.0;
        track_vis.scale.y = 1.0;
        track_vis.scale.z = 3.0;

        float colorcode = ((track_vis.id - 0x500) / (0x53F - 0x500)); // [0, 1]
        track_vis.color.b = 1 - colorcode;
        track_vis.color.g = colorcode;
        track_vis.color.r = 0.0;
        track_vis.color.a = 1;

        this->pub_filtered_radar_track->publish(*msg);
        this->pub_filtered_radar_track_vis->publish(track_vis);
      }
    }


bool GeofenceFilterNode::poseInBodyIsValid(
  const geometry_msgs::msg::Pose &point_in_body)
{
  auto object_pose_in_global = nif::common::utils::coordination::getPtBodytoGlobal(
    this->getEgoOdometry(),
    point_in_body
  ).pose;

  this->wpt_manager_inner->setCurrentOdometry(object_pose_in_global);
  this->wpt_manager_outer->setCurrentOdometry(object_pose_in_global);

  auto current_idx_inner_in_global = this->wpt_manager_inner->getCurrentIdx(
    object_pose_in_global);
  auto current_idx_outer_in_global = this->wpt_manager_outer->getCurrentIdx(
    object_pose_in_global);

  auto current_wpt_pose_inner_in_global = this->wpt_manager_inner->getPoseStampedAtIndex(current_idx_inner_in_global).pose;
  auto current_wpt_pose_outer_in_global = this->wpt_manager_outer->getPoseStampedAtIndex(current_idx_outer_in_global).pose;

  // filter by distance
  if (this->distance_filter_active) 
  {
    // Use squared because we're not interested in the actual distance, save a sqrt() call.
    if (
      nif::common::utils::geometry::calEuclideanDistanceSquared(
        object_pose_in_global.position, 
        current_wpt_pose_inner_in_global.position) < this->distance_filter_threshold_m_squared ||
      nif::common::utils::geometry::calEuclideanDistanceSquared(
        object_pose_in_global.position, 
        current_wpt_pose_outer_in_global.position) < this->distance_filter_threshold_m_squared)
    {
      // Element should be filtered out
      return false;
    }
  }

  // filter by boundaries in/out
  if (this->boundaries_filter_active) 
  {
      double sign;
      // Use squared because we're not interested in the actual distance, save a sqrt() call.
      auto cp_inner_in_global = nif::common::utils::algebra::calCrossProduct(
        object_pose_in_global.position, 
        current_wpt_pose_inner_in_global.position);

      auto cp_outer_in_global = nif::common::utils::algebra::calCrossProduct(
        object_pose_in_global.position, 
        current_wpt_pose_outer_in_global.position);

      // TODO Handle case CP ~= 0. It happens twice on an oval track.
      if (abs(cp_outer_in_global.z) <= 0.05 || abs(cp_inner_in_global.z) <= 0.05)
      {
        // Transform to body, very unlikely to have zero in both references
        auto current_wpt_pose_inner_in_body = nif::common::utils::coordination::getPtGlobaltoBody(
          this->getEgoOdometry(),
          current_wpt_pose_inner_in_global
        ).pose;
        auto current_wpt_pose_outer_in_body = nif::common::utils::coordination::getPtGlobaltoBody(
          this->getEgoOdometry(),
          current_wpt_pose_outer_in_global
        ).pose;

        // Use squared because we're not interested in the actual distance, save a sqrt() call.
        auto cp_inner_in_body = nif::common::utils::algebra::calCrossProduct(
          point_in_body.position, 
          current_wpt_pose_inner_in_body.position);

        auto cp_outer_in_body = nif::common::utils::algebra::calCrossProduct(
          point_in_body.position, 
          current_wpt_pose_outer_in_body.position);

        if (cp_outer_in_body.z == 0.0 || cp_inner_in_body.z == 0.0)
          return false; // At this point just continue

        sign = cp_inner_in_body.z * cp_outer_in_body.z;

      } else {
        
        // TODO remove 2D assumption! using vector_c.z sign is valid only if vector_a.z = vector_b.z = 0
        // If the two CP have the same sign the point is outside the boundaries
        sign = cp_inner_in_global.z * cp_outer_in_global.z;
      }


      if (sign > 0.0) {
        // Point is out, element should be filtered out
        return false;
      }
  }

  return true;
}