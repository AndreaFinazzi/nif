#ifndef FRENET_BASED_OPPONENT_PREDICTOR_H
#define FRENET_BASED_OPPONENT_PREDICTOR_H

#include "nif_common/types.h"
// #include "nif_utils/amathutils_lib/amathutils.hpp"
#include <assert.h>
#include <float.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include "nif_msgs/msg/dynamic_trajectory.hpp"

#include <fstream>
#include <iostream>
#include <vector>

#include "cubic_spliner_2D.h"
#include "frenet_path.h"
#include "frenet_path_generator.h"
#include "nif_common/constants.h"
#include "nif_frame_id/frame_id.h"
#include "nif_utils/utils.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "rcl_interfaces/msg/set_parameters_result.hpp"

using namespace std;

namespace nif {
namespace perception {
class FrenetBasedOpponentPredictor : public rclcpp::Node {
public:
FrenetBasedOpponentPredictor(const string& node_name);
  ~FrenetBasedOpponentPredictor() {}

  void setOpponentStatus(const nif_msgs::msg::Perception3D& oppo_status_) {
    m_opponent_status = oppo_status_;
  }

  void setEgoStatus(const nav_msgs::msg::Odometry& ego_status_) {
    m_ego_status = ego_status_;
  }

  void predict();
  void calcOpponentProgress();
  double calcProgress(geometry_msgs::msg::PoseStamped& pt_);

  tuple<vector<double>, vector<double>>
  loadCSVFile(const string wpt_file_path_);

  nif_msgs::msg::DynamicTrajectory getPredictiveTrajectoryInGlobal() {
    return m_predicted_output_in_global;
  }
  nif_msgs::msg::DynamicTrajectory getPredictiveTrajectoryInBody() {
    return m_predicted_output_in_local;
  }

  void opponentStatusCallback(const nif_msgs::msg::Perception3D::SharedPtr msg);
  void egoStatusCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void defenderVelCallback(const std_msgs::msg::Float32::SharedPtr msg);

private:
  nav_msgs::msg::Path m_opponent_target_path;         // without orientation
  nav_msgs::msg::Path m_opponent_splined_target_path; // with orientation

  string m_centerline_ref_file_path, m_prediction_config_file_path;

  double m_opponent_global_progress; // progress indicator ragarding to the
                                     // opponent_target_path path
  double m_opponent_cte;

  vector<double> m_progress_vec;

  vector<double> m_centerline_path_x,
      m_centerline_path_y; // full path without splining
  vector<double> m_opponent_local_maptrack_path_x,
      m_opponent_local_maptrack_path_y; // local path without splining

  vector<double> m_splined_center_path_x, m_splined_center_path_y,
      m_splined_center_path_yaw,
      m_splined_center_path_curvature; // full path with splining
  vector<double> m_opponent_splined_local_maptrack_path_x,
      m_opponent_splined_local_maptrack_path_y; // local path with splining
  std::shared_ptr<CubicSpliner2D> m_centerline_splined_model;

  nif_msgs::msg::Perception3D m_opponent_status; // in global
  nav_msgs::msg::Odometry m_ego_status;

  double m_config_path_spline_interval_m;
  double m_config_prediction_horizon_s;
  double m_config_prediction_sampling_time_s;
  double m_config_oppo_vel_bias_mps = 0.5; // mps
  double m_defender_vel_mps; // assume that official provides the speed of
                             // defender

  nif_msgs::msg::DynamicTrajectory m_predicted_output_in_global,
      m_predicted_output_in_local;
  nav_msgs::msg::Path m_predicted_output_in_global_vis;

  shared_ptr<FrenetPathGenerator> m_frenet_generator_ptr;

  bool m_prediction_valid_flg;
  bool m_opponent_target_path_valid_flg;
  bool m_config_valid_flg;
  bool m_initialize_done_flg;

  // Subscribers & topic information
  rclcpp::Subscription<nif_msgs::msg::Perception3D>::SharedPtr
      m_sub_opponent_status;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_ego_status;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_sub_defender_vel;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle;

  rcl_interfaces::msg::SetParametersResult parametersCallback(
          const std::vector<rclcpp::Parameter> &vector);

  string m_opponent_status_topic_name, m_ego_status_topic_name,
      m_defender_vel_topic_name;

  // Publisher & topic name
  rclcpp::Publisher<nif_msgs::msg::DynamicTrajectory>::SharedPtr
      m_pub_predicted_trajectory;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
      m_pub_predicted_trajectory_vis;
  string m_predicted_trajectory_topic_name;
  string m_predicted_trajectory_vis_topic_name;
};
} // namespace perception
} // namespace nif

#endif // FRENET_BASED_OPPONENT_PREDICTOR_H
