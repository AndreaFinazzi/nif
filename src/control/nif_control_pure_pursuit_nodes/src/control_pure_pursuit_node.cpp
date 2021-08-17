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

  steer_control_cmd_msg = std::make_shared<nif::common::msgs::ControlCmd>();

  // Create Timer
  m_ego_update_timer = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&ControlPurePursuitNode::egoUpdateTimerCallback, this));

  // Assign parameter to the member variable
  initParameters();
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
}

/**
 * solve() is called every time-step, and its result is published as output of the controller.
 *
 * @return reference to ControlCmd
 */
nif::common::msgs::ControlCmd &ControlPurePursuitNode::solve() {
//  TODO move the check to path length
  if (!this->getReferenceTrajectory()->points.empty())
  {
    auto pure_pursuit_steer_cmd = m_pure_pursuit_handler_ptr->getSteerCmd();
    steer_control_cmd_msg->steering_control_cmd = pure_pursuit_steer_cmd;

    return *steer_control_cmd_msg;
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "%s\n",
                 "PURE PURSUIT : Ego status and Map track are not updated yet. "
                 "Send ZERO "
                 "steering command.");

    steer_control_cmd_msg->steering_control_cmd = 0.0;
    return *steer_control_cmd_msg;

  }
}