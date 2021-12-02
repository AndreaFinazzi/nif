//
// Created by usrg on 30/11/21.
//

#include "../include/adaptive_cruise_node.hpp"

using namespace nif::control;

IDMACCNode::IDMACCNode(const std::string &node_name_)
    : IBaseNode(node_name_, common::NodeType::CONTROL)
{
  std::string package_share_directory;

  try
  {
    // This value shouldn't be used, it's as a backup if a config param is
    // missing.
    package_share_directory = ament_index_cpp::get_package_share_directory(
        "nif_adaptive_cruise_control_node");
  }
  catch (std::exception e)
  {
    RCLCPP_FATAL(this->get_logger(), "Can't get package_share_directory");
  }

  m_detection_subscriber =
      this->create_subscription<nif_msgs::msg::DetectedObjectArray>(
          "/input_topic_name", nif::common::constants::QOS_CONTROL_CMD,
          std::bind(&IDMACCNode::detectionCallback,
                    this, std::placeholders::_1));

  this->declare_parameter("idm_acc_config_file", "idm_acc_config.yaml");
  m_config_file = this->get_parameter("idm_acc_config_file").as_string();

  // IDM LIB initialize
  // 1. with defualt config
  // this->m_idm_prt = std::make_shared<IDM>();
  // 2. with specific config
  this->m_idm_prt = std::make_shared<IDM>(m_config_file);
}

void IDMACCNode::detectionCallback(const nif_msgs::msg::DetectedObjectArray::SharedPtr det_msg)
{
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
  // * double check that the detection message is represent in body frame (if no, converte to the body frame)

  int cipv_idx = 0;

  if (this->hasEgoOdometry() && this->hasEgoPowertrainState() && (det_msg->objects.size() != 0))
  {
    // NOTE : This is just for the test. Proecssing of CIPC data should be done.
    m_idm_prt->calcAccel(m_veh_speed_mps, det_msg->objects[cipv_idx].pose.position.x, det_msg->objects[cipv_idx].velocity.linear.x);
    m_acc_cmd = m_idm_prt->getACCCmd();
  }
  else
  {
    // abnormal situation
  }
}