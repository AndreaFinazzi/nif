#ifndef FRENET_BASED_OPPONENT_PREDICTOR_H
#define FRENET_BASED_OPPONENT_PREDICTOR_H

#include <assert.h>
#include <float.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "nif_common/types.h"
#include "nif_utils/amathutils_lib/amathutils.hpp"

#include <iostream>
#include <vector>
#include <fstream>
#include <iostream>

#include "cubic_spliner_2D.h"
#include "frenet_path.h"
#include "frenet_path_generator.h"
#include <rclcpp/rclcpp.hpp>
#include "nif_frame_id/frame_id.h"

using namespace std;

namespace nif
{
  namespace perception
  {
    class FrenetBasedOpponentPredictor : public rclcpp::Node
    {
    public:
      FrenetBasedOpponentPredictor(const string &target_ref_file_path_,
                                   const string &prediction_config_file_path_);
      ~FrenetBasedOpponentPredictor() {}

      void setOpponentStatus(const nif::common::msgs::PerceptionResult &oppo_status_)
      {
        m_opponent_status = oppo_status_;
      }

      void setEgoStatus(const nav_msgs::msg::Odometry &ego_status_) { m_ego_status = ego_status_; }

      void predict();
      void calcOpponentProgress();

      tuple<vector<double>, vector<double>> loadCSVFile(const string wpt_file_path_);

      nif::common::msgs::NIF_Trajectory getPredictiveTrajectoryInGlobal() { return m_predicted_output_in_global; }
      nif::common::msgs::NIF_Trajectory getPredictiveTrajectoryInBody() { return m_predicted_output_in_local; }

      void opponentStatusCallback(
          const nif::common::msgs::PerceptionResult::SharedPtr msg);
      void egoStatusCallback(
          const nav_msgs::msg::Odometry::SharedPtr msg);

    private:
      nav_msgs::msg::Path m_opponent_target_path;         // without orientation
      nav_msgs::msg::Path m_opponent_splined_target_path; // with orientation

      string m_target_ref_file_path, m_prediction_config_file_path;

      double m_opponent_global_progress; // progress indicator ragarding to the opponent_target_path path
      double m_opponent_cte;

      vector<double> m_opponent_target_path_x, m_opponent_target_path_y;                 // full path without splining
      vector<double> m_opponent_local_maptrack_path_x, m_opponent_local_maptrack_path_y; // local path without splining

      vector<double> m_opponent_splined_target_path_x, m_opponent_splined_target_path_y, m_opponent_splined_target_path_yaw, m_opponent_splined_target_path_curvature; // full path with splining
      vector<double> m_opponent_splined_local_maptrack_path_x, m_opponent_splined_local_maptrack_path_y;                                                               // local path with splining
      std::shared_ptr<CubicSpliner2D> m_splined_model;

      nif::common::msgs::PerceptionResult m_opponent_status; // in global
      nav_msgs::msg::Odometry m_ego_status;

      double m_config_target_path_sline_interval;
      double m_config_prediction_horizon;
      double m_config_prediction_sampling_time;
      double m_config_oppo_vel_bias_mps;

      nif::common::msgs::NIF_Trajectory m_predicted_output_in_global, m_predicted_output_in_local;
      nav_msgs::msg::Path m_predicted_output_in_global_vis;

      shared_ptr<FrenetPathGenerator>
          m_frenet_generator_ptr;

      bool m_prediction_valid_flg;
      bool m_opponent_target_path_valid_flg;
      bool m_config_valid_flg;
      bool m_initialize_done_flg;

      // Subscribers & topic information
      rclcpp::Subscription<nif::common::msgs::PerceptionResult>::SharedPtr m_sub_opponent_status;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_sub_ego_status;
      string m_opponent_status_topic_name, m_ego_status_topic_name;

      // Publisher & topic name
      rclcpp::Publisher<nif::common::msgs::NIF_Trajectory>::SharedPtr m_pub_predicted_trajectory;
      string m_predicted_trajectory_topic_name;
    };
  }
}

#endif // FRENET_BASED_OPPONENT_PREDICTOR_H
