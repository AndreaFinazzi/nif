//
// Created by usrg on 30/11/21.
//

#ifndef IDM_BASED_ACC_NODE_H
#define IDM_BASED_ACC_NODE_H

#include "idm.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_msgs/msg/detected_object.hpp"
#include "nif_msgs/msg/detected_object_array.hpp"
#include "nif_msgs/msg/dynamic_trajectory.hpp"
#include "nif_msgs/msg/perception3_d.hpp"
#include "nif_msgs/msg/perception3_d_array.hpp"
#include "nif_utils/utils.h"
#include "std_msgs/msg/float32.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <rclcpp/rclcpp.hpp>

namespace nif {
namespace control {
class IDMACCNode : public nif::common::IBaseNode {
public:
  IDMACCNode(const std::string &node_name_);
  //   IDMACCNode(const std::string &node_name_,
  //              const std::shared_ptr<IDM> idm_prt_);
  ~IDMACCNode() {}

  void detectionCallback(
      const nif_msgs::msg::DetectedObjectArray::SharedPtr det_msg);
  void
  perceptionCallback(const nif_msgs::msg::Perception3DArray::SharedPtr det_msg);
  void egoTrajectoryCallback(
      const nif_msgs::msg::DynamicTrajectory::SharedPtr traj_msg);
  void maptrackBodyCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void
  predictionCallback(const nif_msgs::msg::DynamicTrajectory::SharedPtr msg);

private:
  /* data */
  //   IDMACCNode();
  double m_acc_cmd;
  double m_veh_speed_mps;

  std::string m_config_file;

  nif_msgs::msg::DetectedObjectArray m_det_result;
  nif_msgs::msg::Perception3DArray m_perception_result;
  nif_msgs::msg::DynamicTrajectory m_prediction_result;
  nif_msgs::msg::DynamicTrajectory m_last_prediction_result;

  rclcpp::Time m_prev_oppo_pred_last_update;
  bool m_oppo_pred_callback_first_run;

  nav_msgs::msg::Odometry m_ego_odom;
  nav_msgs::msg::Path m_maptrack_body;
  nif_msgs::msg::DynamicTrajectory m_ego_trajectory;

  std::string m_input_perception_topic_name;
  std::string m_input_maptrack_body_topic_name;
  std::string m_output_acc_cmd_topic_name;

  std::shared_ptr<IDM> m_idm_prt;

  // detection result subscriber
  rclcpp::Subscription<nif_msgs::msg::Perception3DArray>::SharedPtr
      m_perception_subscriber;
  rclcpp::Subscription<nif_msgs::msg::DynamicTrajectory>::SharedPtr
      m_prediction_subscriber;
  rclcpp::Subscription<nif_msgs::msg::DynamicTrajectory>::SharedPtr
      m_ego_traj_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
      m_maptrack_body_subscriber;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_acc_cmd_publisher;
};
} // namespace control
} // namespace nif

#endif // IDM_BASED_ACC_NODE_H
