
/*
 * geofence_filter_node.cpp
 *
 *  Created on: Aug 10, 2021
 *      Author: Andrea Finazzi for USRG @ KAIST
 */

#include "nif_geofence_filter_nodes/geofence_filter_node.h"

using nif::perception::GeofenceFilterNode;
using nif::common::constants::DEG2RAD;

GeofenceFilterNode::GeofenceFilterNode(const std::string &node_name_)
    // IBaseNode with no SSM subscription
    : IBaseNode(node_name_, nif::common::NodeType::PERCEPTION, rclcpp::NodeOptions{}, false) { 

    this->declare_parameter<std::string>("file_path_inner_line", "");
    this->declare_parameter<std::string>("file_path_outer_line", "");

    this->declare_parameter<bool>("distance_filter_active", true);
    this->declare_parameter<double>("distance_filter_threshold_m", 0.5);

    this->declare_parameter<bool>("boundaries_filter_active", true);

    this->declare_parameter<bool>("range_rate_filter_active", true);
    this->declare_parameter<double>("range_rate_filter_threshold_mps", 1.0);

    this->declare_parameter<bool>("track_angle_filter_active", true);
    this->declare_parameter<double>("track_angle_filter_threshold_deg", 15.0);
    
    this->declare_parameter<bool>("track_range_filter_active", true);
    this->declare_parameter<double>("track_range_filter_threshold_min_m", 20.0);

    auto file_path_inner_line = this->get_parameter("file_path_inner_line").as_string();
    auto file_path_outer_line = this->get_parameter("file_path_outer_line").as_string();

    this->distance_filter_active = this->get_parameter("distance_filter_active").as_bool();
    this->distance_filter_threshold_m = this->get_parameter("distance_filter_threshold_m").as_double();
    this->distance_filter_threshold_m_squared = pow(this->distance_filter_threshold_m, 2);

    this->boundaries_filter_active = this->get_parameter("boundaries_filter_active").as_bool();

    this->range_rate_filter_active = this->get_parameter("range_rate_filter_active").as_bool();
    this->range_rate_filter_threshold_mps = this->get_parameter("range_rate_filter_threshold_mps").as_double();

    this->track_angle_filter_active = this->get_parameter("track_angle_filter_active").as_bool();
    this->track_angle_filter_threshold_deg = this->get_parameter("track_angle_filter_threshold_deg").as_double();
    
    this->track_range_filter_active = this->get_parameter("track_range_filter_active").as_bool();
    this->track_range_filter_threshold_min_m = this->get_parameter("track_range_filter_threshold_min_m").as_double();

    sub_perception_array = this->create_subscription<nif_msgs::msg::Perception3DArray>(
        "in_perception_array", nif::common::constants::QOS_SENSOR_DATA,
        std::bind(&GeofenceFilterNode::perceptionArrayCallback, this,
                  std::placeholders::_1));

    pub_filtered_perception_array = this->create_publisher<nif_msgs::msg::Perception3DArray>(
        "out_filtered_perception_array", nif::common::constants::QOS_SENSOR_DATA);

    //sw's option
    pub_filtered_perception_array_vis = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "out_filtered_perception_array_vis", nif::common::constants::QOS_SENSOR_DATA); 
    ///////////////////////////////////////////////////////////

    sub_radar_track = this->create_subscription<delphi_esr_msgs::msg::EsrTrack>(
        "in_radar_track", nif::common::constants::QOS_SENSOR_DATA,
        std::bind(&GeofenceFilterNode::radarTrackCallback, this,
                  std::placeholders::_1));

    pub_filtered_radar_track = this->create_publisher<delphi_esr_msgs::msg::EsrTrack>(
        "out_filtered_radar_track", nif::common::constants::QOS_SENSOR_DATA);

    pub_filtered_radar_track_vis = this->create_publisher<visualization_msgs::msg::Marker>(
        "out_filtered_radar_track_vis", nif::common::constants::QOS_SENSOR_DATA);

    pub_filtered_radar_perception_list = this->create_publisher<nif::common::msgs::PerceptionResultList>(
        "out_filtered_radar_perception_list", nif::common::constants::QOS_SENSOR_DATA);

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

    this->parameters_callback_handle = this->add_on_set_parameters_callback(
        std::bind(&GeofenceFilterNode::parametersCallback, this, std::placeholders::_1));

    this->alternative_orig_frame.position.x = 1000.0;
}

// void GeofenceFilterNode::perceptionArrayCallback(
//     const nif_msgs::msg::Perception3DArray::SharedPtr msg) {

//   nif_msgs::msg::Perception3DArray msg_out{};
//   msg_out.header = msg->header;

//   for (auto&& object : msg->perception_list) {
//     if (this->poseInBodyIsValid(object.detection_result_3d.center, object.obj_velocity_in_local.linear.x))
//       msg_out.perception_list.push_back(object);
//   }
  
//   this->pub_filtered_perception_array->publish(msg_out);

// }

//sw's for visualize
void GeofenceFilterNode::perceptionArrayCallback(
    const nif_msgs::msg::Perception3DArray::SharedPtr msg) {

  nif_msgs::msg::Perception3DArray msg_out{};
  msg_out.header = msg->header;

  visualization_msgs::msg::MarkerArray array_vis;

  int i = 0;

  for (auto&& object : msg->perception_list) {
    // visualization_msgs::msg::MarkerArray array_vis;

    if (this->poseInBodyIsValid(object.detection_result_3d.center, object.obj_velocity_in_local.linear.x)){
      msg_out.perception_list.push_back(object);

      visualization_msgs::msg::Marker buff;
      buff.header = object.header;
      buff.type = visualization_msgs::msg::Marker::CUBE;
      buff.action = visualization_msgs::msg::Marker::ADD;
      buff.lifetime = rclcpp::Duration(2, 0.3);
      buff.id = i++;

      buff.pose = object.detection_result_3d.center;
      // buff.scale = object.detection_result_3d.size;

      buff.scale.x = 7.0;
      buff.scale.y = 7.0;
      buff.scale.z = 7.0;

      
      buff.color.b = 0.0;
      buff.color.g = 1.0;
      buff.color.r = 0.0;
      buff.color.a = 1.0;

      array_vis.markers.push_back(buff);
      
    }
      
  }

  this->pub_filtered_perception_array_vis->publish(array_vis);
  this->pub_filtered_perception_array->publish(msg_out);

}
//////////////////////////////////////////////////////////////////////////

void GeofenceFilterNode::radarTrackCallback(
    const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg) {

      if (this->trackIsValid(msg)) {

        geometry_msgs::msg::Pose pose_in_body{};
        auto theta_rad = DEG2RAD * msg->track_angle;
        auto& r = msg->track_range;

        pose_in_body.position.x = r * cos(theta_rad);
        pose_in_body.position.y = r * sin(theta_rad);

        // @DEBUG filter on inner fence only if angle is smaller than 10.0
        if (
          this->poseInBodyIsValid(pose_in_body, msg->track_range_rate) || 
          (abs(msg->track_angle) < 10.0 && this->poseInBodyIsValid(pose_in_body, msg->track_range_rate, false, true))
        )
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

          nif::common::msgs::PerceptionResultList track_prl{};
          nif::common::msgs::PerceptionResult track_pr{};
          track_prl.header = msg->header;
          track_pr.header = msg->header;
          // @DEBUG 
          // TODO convert frame and use constant for naming
          track_prl.header.frame_id = "base_link";
          track_pr.header.frame_id = "base_link";
          track_pr.id = msg->track_id;
          track_pr.obj_velocity_in_local.linear.x = msg->track_range_rate;
          track_pr.detection_result_3d.center = pose_in_body;
          track_prl.perception_list.push_back(track_pr);


          this->pub_filtered_radar_track->publish(*msg);
          this->pub_filtered_radar_track_vis->publish(track_vis);
          this->pub_filtered_radar_perception_list->publish(track_prl);
        }
      }
    }


bool GeofenceFilterNode::poseInBodyIsValid(
  const geometry_msgs::msg::Pose &point_in_body, 
  double r_dot,
  bool filter_on_inner_fence, bool filter_on_outer_fence)
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
      (filter_on_inner_fence &&
      nif::common::utils::geometry::calEuclideanDistanceSquared(
        object_pose_in_global.position, 
        current_wpt_pose_inner_in_global.position) < this->distance_filter_threshold_m_squared) ||
      (filter_on_outer_fence &&
      nif::common::utils::geometry::calEuclideanDistanceSquared(
        object_pose_in_global.position, 
        current_wpt_pose_outer_in_global.position) < this->distance_filter_threshold_m_squared))
    {
      // Element should be filtered out
      return false;
    }
  }

  // filter by boundaries in/out
  if (this->boundaries_filter_active) 
  {

    auto prev_wpt_pose_inner_in_global = this->wpt_manager_inner->getPoseStampedAtIndex(current_idx_inner_in_global - 1).pose;
    auto prev_wpt_pose_outer_in_global = this->wpt_manager_outer->getPoseStampedAtIndex(current_idx_outer_in_global - 1).pose;
    
    auto current_wpt_pose_inner_in_prev = nif::common::utils::coordination::getPtGlobaltoBody(
      prev_wpt_pose_inner_in_global, current_wpt_pose_inner_in_global);
    auto current_wpt_pose_outer_in_prev = nif::common::utils::coordination::getPtGlobaltoBody(
      prev_wpt_pose_outer_in_global, current_wpt_pose_outer_in_global);
  
    auto object_pose_in_inner_prev = nif::common::utils::coordination::getPtGlobaltoBody(
      prev_wpt_pose_inner_in_global, object_pose_in_global);
    auto object_pose_in_outer_prev = nif::common::utils::coordination::getPtGlobaltoBody(
      prev_wpt_pose_outer_in_global, object_pose_in_global);

    // Use squared because we're not interested in the actual distance, save a sqrt() call.
    auto cp_inner_in_prev = nif::common::utils::algebra::calCrossProduct(
      object_pose_in_inner_prev.position, 
      current_wpt_pose_inner_in_prev.position);

    auto cp_outer_in_prev = nif::common::utils::algebra::calCrossProduct(
      object_pose_in_outer_prev.position, 
      current_wpt_pose_outer_in_prev.position);

    if (
      (filter_on_inner_fence && cp_inner_in_prev.z < 0) || // object on the left of inner wpt
      (filter_on_outer_fence && cp_outer_in_prev.z > 0)  // object on the right of outer wpt
    ) return false;

  }

  // filter by range rate
  // TODO update considering external info abou t the speed of the desired target
  // !!!!!!! WARNING this can be very dangerous, it filters out  static obstacles! !!!!!!!!!
  if (this->range_rate_filter_active) 
  {
    if (this->getEgoOdometry().twist.twist.linear.x + r_dot < this->range_rate_filter_threshold_mps )
      return false; // Likely a wall point
  }

  return true;
}

bool GeofenceFilterNode::trackIsValid(
  const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg) {
  if (this->track_angle_filter_active) 
  {
    if (abs(msg->track_angle) > this->track_angle_filter_threshold_deg)
      return false;
  }

  if (this->track_range_filter_active)
  {
    if (msg->track_range < this->track_range_filter_threshold_min_m)
      return false;
  }

  return true;
}


rcl_interfaces::msg::SetParametersResult
nif::perception::GeofenceFilterNode::parametersCallback(
    const std::vector<rclcpp::Parameter> &vector) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "";
  for (const auto &param : vector) {
    if (param.get_name() == "distance_filter_active") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (true) // TODO implement switching policy, if needed
        {
          this->distance_filter_active = param.as_bool();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "boundaries_filter_active") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (true) {
          this->boundaries_filter_active = param.as_bool();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "range_rate_filter_active") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
        if (true) {
          this->range_rate_filter_active = param.as_bool();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "distance_filter_threshold_m") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->distance_filter_threshold_m = param.as_double();
          this->distance_filter_threshold_m_squared = pow(this->distance_filter_threshold_m, 2);
          result.successful = true;
        }
      }
    } else if (param.get_name() == "boundaries_filter_threshold_m") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->boundaries_filter_threshold_m = param.as_double();
          result.successful = true;
        }
      }
    } else if (param.get_name() == "range_rate_filter_threshold_mps") {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
        if (true) {
          this->range_rate_filter_threshold_mps = param.as_double();
          result.successful = true;
        }
      }
    }
  }
  return result;
}
