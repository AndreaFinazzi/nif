#include "nif_control_pure_pursuit_nodes/control_pure_pursuit_node.hpp"
#include "std_srvs/srv/set_bool.hpp"

using nif::control::ControlPurePursuitNode;

ControlPurePursuitNode::ControlPurePursuitNode(const std::string &node_name)
    : IControllerNode(node_name) {

  // Publishers
  m_lookahead_pt_in_global_pub = this->create_publisher<visualization_msgs::msg::Marker>(
      "control_pure_pursuit/lookahead_point", 1);

  control_command = std::make_shared<nif::common::msgs::ControlCmd>();

  try {
    // Assign parameter to the member variable
    this->declare_parameter("invert_steer_direction", false);
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

/**
 * Publish lookahead marker for visualization purposes.
 */
void ControlPurePursuitNode::publishLookAheadPointMarker() {
  visualization_msgs::msg::Marker look_ahead_marker;
  look_ahead_marker.header.stamp = this->now();
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
nif::common::msgs::ControlCmd::SharedPtr ControlPurePursuitNode::solve() {
//TODO update to comply with IControllerNode last version (embedded path subscriber and has...() methods)
  if  (
          (this->getReferenceTrajectory() && !(this->getReferenceTrajectory()->points.empty()))
          ||
          (this->hasReferencePath() && !(this->getReferencePath()->poses.empty()))
      )
  {
    auto pure_pursuit_steer_cmd_rad = m_pure_pursuit_handler_ptr->getSteerCmd();
    control_command->steering_control_cmd.data =
        pure_pursuit_steer_cmd_rad * nif::common::constants::RAD2DEG;

    m_lookahead_pt_in_global = m_pure_pursuit_handler_ptr->getLookaheadPointInGlobal();
    m_control_pt_in_body = m_pure_pursuit_handler_ptr->getControlPointInBody();

    this->publishLookAheadPointMarker();

    return control_command;
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "%s\n",
                 "PURE PURSUIT : Ego status and Map track are not updated yet. "
                 "Send ZERO "
                 "steering command.");

    control_command->steering_control_cmd.data = 0.0;
    return control_command;

  }
}
void nif::control::ControlPurePursuitNode::afterReferencePathCallback() {
  m_pure_pursuit_handler_ptr->setMapTrack(*(this->getReferencePath()));
}
void nif::control::ControlPurePursuitNode::afterEgoOdometryCallback() {
  m_pure_pursuit_handler_ptr->setVehicleStatus(this->getEgoOdometry());
}
