//
// Created by usrg on 30/11/21.
//

// #include "adaptive_cruise_node.hpp"
#include "nif_adaptive_cruise_control_node/adaptive_cruise_node.hpp"

using namespace nif::control;

IDMACCNode::IDMACCNode(const std::string &node_name_)
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

  this->declare_parameter("idm_acc_config_file", "idm_acc_config.yaml");
  m_config_file = this->get_parameter("idm_acc_config_file").as_string();

  // publisher
  m_acc_cmd_publisher = this->create_publisher<std_msgs::msg::Float32>(
      "out_acc_acceleration", nif::common::constants::QOS_CONTROL_CMD);

  // m_perception_subscriber =
  //     this->create_subscription<nif_msgs::msg::Perception3DArray>(
  //         "in_perception_array", nif::common::constants::QOS_SENSOR_DATA,
  //         std::bind(&IDMACCNode::perceptionCallback, this,
  //                   std::placeholders::_1));

  m_prediction_subscriber =
      this->create_subscription<nif_msgs::msg::DynamicTrajectory>(
          "in_prediction_array", nif::common::constants::QOS_SENSOR_DATA,
          std::bind(&IDMACCNode::predictionCallback, this,
                    std::placeholders::_1));

  // m_maptrack_body_subscriber =
  // this->create_subscription<nav_msgs::msg::Path>(
  //     "in_maptrack_in_body", nif::common::constants::QOS_PLANNING,
  //     std::bind(&IDMACCNode::maptrackBodyCallback, this,
  //               std::placeholders::_1));

  // IDM LIB initialize
  // 1. with defualt config
  // this->m_idm_prt = std::make_shared<IDM>();
  // 2. with specific config
  m_oppo_pred_callback_first_run = true;
  this->m_idm_prt = std::make_shared<IDM>(m_config_file);
}

void IDMACCNode::egoTrajectoryCallback(
    const nif_msgs::msg::DynamicTrajectory::SharedPtr traj_msg) {

  m_ego_trajectory = *traj_msg;

  m_ego_odom = this->getEgoOdometry();
  m_veh_speed_mps = m_ego_odom.twist.twist.linear.x;

  // prediction health check
  if (!m_oppo_pred_callback_first_run) {
    if (this->now() - m_prev_oppo_pred_last_update > rclcpp::Duration(2, 0)) {
      m_prediction_result.trajectory_path.poses.clear();
      m_prediction_result.trajectory_velocity.clear();
      m_prediction_result.trajectory_timestamp_array.clear();
      m_prediction_result.trajectory_global_progress.clear();
    }
  }

  if (m_prediction_result.trajectory_path.poses.empty()) {
    std_msgs::msg::Float32 out;
    out.data = nif::common::constants::numeric::INF;
    m_acc_cmd_publisher->publish(out);
  } else {
    std_msgs::msg::Float32 out;

    auto naive_gap = sqrt(
        pow(m_ego_odom.pose.pose.position.x -
                m_prediction_result.trajectory_path.poses[0].pose.position.x,
            2) +
        pow(m_ego_odom.pose.pose.position.y -
                m_prediction_result.trajectory_path.poses[0].pose.position.y,
            2));

    m_idm_prt->calcAccel(m_veh_speed_mps, naive_gap,
                         m_prediction_result.trajectory_velocity[0]);

    out.data = m_idm_prt->getACCCmd();
    m_acc_cmd_publisher->publish(out);
  }
}

void IDMACCNode::predictionCallback(
    const nif_msgs::msg::DynamicTrajectory::SharedPtr msg) {

  if (m_oppo_pred_callback_first_run) {
    m_oppo_pred_callback_first_run = false;
  }
  m_prev_oppo_pred_last_update = this->now(); // TODO msg->header.stamp;
  m_prediction_result = *msg;
}

void IDMACCNode::maptrackBodyCallback(
    const nav_msgs::msg::Path::SharedPtr msg) {
  m_maptrack_body = *msg;
}

// void IDMACCNode::detectionCallback(
//     const nif_msgs::msg::DetectedObjectArray::SharedPtr det_msg) {
//   m_det_result = *det_msg;
//   int cipv_idx = 0;

//   // Calc acc cmd
//   // m_veh_speed_mps = this->getEgoPowertrainState().vehicle_speed_kmph
//   / 3.6; m_ego_odom = this->getEgoOdometry(); m_veh_speed_mps =
//   m_ego_odom.twist.twist.linear.x;

//   // TODO : Assigning CIPV
//   // NOTE : In CES, there is only one opponent on the track.
//   // At the moment, we assumed that there is no false positive.

//   // First approach
//   // 1. Convert opponent's position to the global
//   // 2. Project to the ego's racing line to calculate the progress
//   // 3. Calculate the progress gap
//   // * double check that the detection message is represent in body frame (if
//   // no, converte to the body frame)

//   if (this->hasEgoOdometry() && this->hasEgoPowertrainState()) {
//     if (det_msg->objects.size() != 0) {
//       // NOTE : This is just for the test. Proecssing of CIPC data should be
//       // done.

//       // TODO : The way to calculate the progress gap btw the ego and
//       opponent. double progress_gap = 0.0;

//       // Approach 1. (longitudinal-wise distance directly from the perception
//       // result)
//       progress_gap = det_msg->objects[cipv_idx].pose.position.x;
//       if (progress_gap < 0.0) {
//         // when the car is behind us, don't care about the ACC. Set the
//         progress
//         // gap as INF
//         progress_gap = nif::common::constants::numeric::INF;
//       }

//       // Approach 2. (Based on our future trajectory, calculate the progress.
//       // But when the case that we want to overtake, the progress gap might
//       be
//       // wrong.)
//       m_idm_prt->calcAccel(m_veh_speed_mps,
//                            progress_gap,
//                            det_msg->objects[cipv_idx].velocity.linear.x);

//       m_acc_cmd = m_idm_prt->getACCCmd();
//     } else {
//       m_acc_cmd = m_idm_prt->getParamAccelMax();
//     }
//   } else {
//     // abnormal situation
//     m_acc_cmd = 0.0;
//   }

//   std_msgs::msg::Float32 out;
//   out.data = m_acc_cmd;
//   m_acc_cmd_publisher->publish(out);
// }

void IDMACCNode::perceptionCallback(
    const nif_msgs::msg::Perception3DArray::SharedPtr msg) {
  m_perception_result = *msg;

  // TODO : ASSUMING THAT ONLY CIPV INFORMATION IS IN THE MESSAGE
  int cipv_idx = 0;

  // std::cout << "body cipv x : " <<
  // m_perception_result.perception_list[cipv_idx].detection_result_3d.center.position.x
  // << std::endl;

  if (sqrt(pow(m_perception_result.perception_list[cipv_idx]
                   .detection_result_3d.center.position.x,
               2) +
           pow(m_perception_result.perception_list[cipv_idx]
                   .detection_result_3d.center.position.y,
               2)) > 100.0) {
    // opponent is behind us. dont care about the ACC
    m_acc_cmd = m_idm_prt->getParamAccelMax();
    std_msgs::msg::Float32 out;
    out.data = m_acc_cmd;
    m_acc_cmd_publisher->publish(out);
    return;
  }

  // Calc acc cmd
  m_veh_speed_mps = this->getEgoOdometry().twist.twist.linear.x;
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
          m_veh_speed_mps, progress_gap,
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