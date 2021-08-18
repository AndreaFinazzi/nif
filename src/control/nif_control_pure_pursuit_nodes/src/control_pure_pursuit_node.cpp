#include "nif_control_pure_pursuit_nodes/control_pure_pursuit_node.hpp"

using nif::control::ControlPurePursuitNode;

ControlPurePursuitNode::ControlPurePursuitNode(const std::string &node_name)
    : IControllerNode(node_name) {

  // Subscribers
  m_map_track_sub = this->create_subscription<nav_msgs::msg::Path>(
      "target_path", 1,
      std::bind(&ControlPurePursuitNode::mapTrackCallback,
                this,
                std::placeholders::_1));

  // Publishers
  m_lookahead_pt_in_global_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "lookahead_point", 1);

  steer_control_cmd_msg = std::make_shared<nif::common::msgs::ControlCmd>();

  // Create Timer
//  TODO period here SHOULD (really) be parametric
  m_ego_update_timer = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ControlPurePursuitNode::egoUpdateTimerCallback, this));

  try {
    // Assign parameter to the member variable
    this->declare_parameter("invert_steer_direction", true);
    this->get_parameter("invert_steer_direction", m_param_is_steer_sign_inver);

  } catch (const std::exception & e ) {
//    TODO deciding whether or not to abort in this situation is critical
    RCLCPP_ERROR(this->get_logger(), "Failed to declare or get parameters. Setting defaults values.");
    RCLCPP_ERROR(this->get_logger(), "What: %s", e.what());

    this->node_status_manager.update(common::NodeStatusCode::NODE_FATAL_ERROR);

    throw std::runtime_error("Critical error during initialization. ABORT.");
  }

  m_pure_pursuit_handler_ptr = std::make_shared<PurePursuit>(
      m_param_min_lookahead_dist,
      m_param_max_lookahead_dist,
      m_param_lookahead_speed_ratio,
      m_param_use_lpf_flg,
      m_param_lfp_gain,
      m_param_is_steer_sign_inver);
}

// TODO there's a storeTraj function that is called every time we receive a new plan
void ControlPurePursuitNode::mapTrackCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  m_maptrack = *msg;
  m_pure_pursuit_handler_ptr->setMapTrack(m_maptrack);
  this->maptrack_recv_time_ = this->now();
}

void ControlPurePursuitNode::initParameters() {}
void ControlPurePursuitNode::getParameters() {}

void ControlPurePursuitNode::egoUpdateTimerCallback() const {
  m_pure_pursuit_handler_ptr->setVehicleStatus(this->getEgoOdometry());
//  m_ego_status_first_run = true;
}

/**
 * Publish lookahead marker for visualization purposes.
 */
void ControlPurePursuitNode::publishLookAheadPointMarker() {
  visualization_msgs::msg::Marker look_ahead_marker;
  look_ahead_marker.header.stamp = this->get_clock()->now();
  look_ahead_marker.header.frame_id = this->getBodyFrameId();
  look_ahead_marker.type = visualization_msgs::msg::Marker::SPHERE;
  look_ahead_marker.action = visualization_msgs::msg::Marker::ADD;
  look_ahead_marker.pose = m_control_pt_in_body.pose;
  double scale = 0.3;
  look_ahead_marker.scale.x = scale;
  look_ahead_marker.scale.y = scale;
  look_ahead_marker.scale.z = scale;
  look_ahead_marker.color.r = 0.5;
  look_ahead_marker.color.a = 0.5;
  look_ahead_marker.frame_locked = true;
  // look_ahead_marker.points.append(m_lookahead_pt_in_global.pose.position);
  m_lookahead_pt_in_global_pub->publish(look_ahead_marker);
}

/**
 * solve() is called every time-step, and its result is published as output of the controller.
 *
 * @return reference to ControlCmd
 */
nif::common::msgs::ControlCmd &ControlPurePursuitNode::solve() {

  if (!this->getReferenceTrajectory()->points.empty())
  {
    auto pure_pursuit_steer_cmd = m_pure_pursuit_handler_ptr->getSteerCmd();
    steer_control_cmd_msg->steering_control_cmd.data = pure_pursuit_steer_cmd;

    m_lookahead_pt_in_global = m_pure_pursuit_handler_ptr->getLookaheadPointInGlobal();
    m_control_pt_in_body = m_pure_pursuit_handler_ptr->getControlPointInBody();

    return *steer_control_cmd_msg;
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "%s\n",
                 "PURE PURSUIT : Ego status and Map track are not updated yet. "
                 "Send ZERO "
                 "steering command.");

    steer_control_cmd_msg->steering_control_cmd.data = 0.0;
    return *steer_control_cmd_msg;

  }
}