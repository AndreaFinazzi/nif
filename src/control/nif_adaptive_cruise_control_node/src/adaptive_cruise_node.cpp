//
// Created by usrg on 30/11/21.
//

#include "../include/adaptive_cruise_node.hpp"

using namespace nif::control;

IDMACCNode::IDMACCNode(const std::string& node_name_)
  : IBaseNode(node_name_, common::NodeType::CONTROL) {
  std::string package_share_directory;

  try {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_adaptive_cruise_control_node");
  } catch (std::exception e) {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }

  this->declare_parameter("input_perception_msg_topic_name",
                          "/perception/tracked/msg");
  m_input_perception_topic_name =
      this->get_parameter("input_perception_msg_topic_name").as_string();
  this->declare_parameter("idm_acc_config_file", "idm_acc_config.yaml");
  m_config_file = this->get_parameter("idm_acc_config_file").as_string();
  this->declare_parameter("input_maptrack_body_topic_name",
                          "/manager/maptrack_body/msg");
  m_input_maptrack_body_topic_name =
      this->get_parameter("input_maptrack_body_topic_name").as_string();
  this->declare_parameter("output_acc_cmd_topic_name", "/control/acc/cmd");
  m_output_acc_cmd_topic_name =
      this->get_parameter("output_acc_cmd_topic_name").as_string();

  // publisher
  m_acc_cmd_publisher = this->create_publisher<std_msgs::msg::Float32>(
      m_output_acc_cmd_topic_name, nif::common::constants::QOS_CONTROL_CMD);

  // m_detection_subscriber =
  //     this->create_subscription<nif_msgs::msg::DetectedObjectArray>(
  //         m_input_perception_topic_name,
  //         nif::common::constants::QOS_CONTROL_CMD,
  //         std::bind(
  //             &IDMACCNode::detectionCallback, this, std::placeholders::_1));
  m_perception_subscriber =
      this->create_subscription<nif_msgs::msg::Perception3DArray>(
          m_input_perception_topic_name,
          nif::common::constants::QOS_DEFAULT,
          std::bind(
              &IDMACCNode::perceptionCallback, this, std::placeholders::_1));
  m_maptrack_body_subscriber = this->create_subscription<nav_msgs::msg::Path>(
      m_input_maptrack_body_topic_name,
      nif::common::constants::QOS_DEFAULT,
      std::bind(
          &IDMACCNode::maptrackBodyCallback, this, std::placeholders::_1));

  // IDM LIB initialize
  // 1. with defualt config
  // this->m_idm_prt = std::make_shared<IDM>();
  // 2. with specific config
  this->m_idm_prt = std::make_shared<IDM>(m_config_file);
}

void IDMACCNode::maptrackBodyCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  m_maptrack_body = *msg;
}

void IDMACCNode::detectionCallback(
    const nif_msgs::msg::DetectedObjectArray::SharedPtr det_msg) {
  m_det_result = *det_msg;

  // Calc acc cmd
  m_veh_speed_mps = this->getEgoPowertrainState().vehicle_speed_kmph / 3.6;
  m_ego_odom = this->getEgoOdometry();

  // TODO : Assigning CIPV
  // NOTE : In CES, there is only one opponent on the track.
  // At the moment, we assumed that there is no false positive.

  // First approach
  // 1. Convert opponent's position to the global
  // 2. Project to the ego's racing line to calculate the progress
  // 3. Calculate the progress gap
  // * double check that the detection message is represent in body frame (if
  // no, converte to the body frame)

  int cipv_idx = 0;

  if (this->hasEgoOdometry() && this->hasEgoPowertrainState()) {
    if (det_msg->objects.size() != 0) {
      // NOTE : This is just for the test. Proecssing of CIPC data should be
      // done.

      // TODO : The way to calculate the progress gap btw the ego and opponent.
      double progress_gap = 0.0;

      // Approach 1. (longitudinal-wise distance directly from the perception
      // result)
      progress_gap = det_msg->objects[cipv_idx].pose.position.x;
      if (progress_gap < 0.0) {
        // when the car is behind us, don't care about the ACC. Set the progress
        // gap as INF
        progress_gap = nif::common::constants::numeric::INF;
      }

      // Approach 2. (Based on our future trajectory, calculate the progress.
      // But when the case that we want to overtake, the progress gap might be
      // wrong.)
      m_idm_prt->calcAccel(m_veh_speed_mps,
                           progress_gap,
                           det_msg->objects[cipv_idx].velocity.linear.x);

      m_acc_cmd = m_idm_prt->getACCCmd();
    } else {
      m_acc_cmd = m_idm_prt->getParamAccelMax();
    }
  } else {
    // abnormal situation
    m_acc_cmd = 0.0;
  }

  std_msgs::msg::Float32 out;
  out.data = m_acc_cmd;
  m_acc_cmd_publisher->publish(out);
}

void IDMACCNode::perceptionCallback(
    const nif_msgs::msg::Perception3DArray::SharedPtr msg) {
  m_perception_result = *msg;

  // Calc acc cmd
  m_veh_speed_mps = this->getEgoPowertrainState().vehicle_speed_kmph / 3.6;
  m_ego_odom = this->getEgoOdometry();

  // TODO : Assigning CIPV
  // NOTE : In CES, there is only one opponent on the track.
  // At the moment, we assumed that there is no false positive.

  // First approach
  // 1. Convert opponent's position to the global
  // 2. Project to the ego's racing line to calculate the progress
  // 3. Calculate the progress gap
  // * double check that the detection message is represent in body frame (if
  // no, converte to the body frame)

  int cipv_idx = 0;

  if (this->hasEgoOdometry() && this->hasEgoPowertrainState()) {
    if (msg->perception_list.size() != 0) {
      // NOTE : This is just for the test. Proecssing of CIPC data should be
      // done.

      // TODO : The way to calculate the progress gap btw the ego and opponent.
      double progress_gap = 0.0;

      // Approach 1. (longitudinal-wise distance directly from the perception
      // result)
      progress_gap =
          msg->perception_list[cipv_idx].detection_result_3d.center.position.x;
      if (progress_gap < 0.0) {
        // when the car is behind us, don't care about the ACC. Set the progress
        // gap as INF
        progress_gap = nif::common::constants::numeric::INF;
      }

      // Approach 2. (Based on our future trajectory, calculate the progress.
      // But when the case that we want to overtake, the progress gap might be
      // wrong.)
      m_idm_prt->calcAccel(
          m_veh_speed_mps,
          progress_gap,
          msg->perception_list[cipv_idx].obj_velocity_in_local.linear.x);

      m_acc_cmd = m_idm_prt->getACCCmd();
    } else {
      m_acc_cmd = m_idm_prt->getParamAccelMax();
    }
  } else {
    // abnormal situation
    m_acc_cmd = 0.0;
  }

  std_msgs::msg::Float32 out;
  out.data = m_acc_cmd;
  m_acc_cmd_publisher->publish(out);
}