//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/24/21.
//

#ifndef ROS2MASTER_DYNAMIC_PLANNER_NODE_H
#define ROS2MASTER_DYNAMIC_PLANNER_NODE_H

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_common/vehicle_model.h"
#include "nif_common_nodes/i_base_node.h"
#include "nif_frame_id/frame_id.h"
#include "nif_msgs/msg/dynamic_trajectory.hpp"
#include "nif_msgs/msg/perception3_d.hpp"
#include "nif_opponent_prediction_nodes/cubic_spliner_2D.h"
#include "nif_opponent_prediction_nodes/frenet_path.h"
#include "nif_opponent_prediction_nodes/frenet_path_generator.h"
#include "nif_utils/utils.h"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/error_handling.h"
#include "std_msgs/msg/float32.hpp"
#include <math.h>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

// pcl
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

using namespace std;

namespace nif {
namespace planning {

class AdaptiveDynamicPlannerNode : public nif::common::IBaseNode {
  enum PLANNING_DECISION_TYPE {
    STRAIGHT, // follow the original racing line
    FOLLOW,   // stay behind the opponent
    RIGHT,    // overtake to the right
    LEFT,     // overtake to the left
    ESTOP     // emergency stop
  };
  enum PLANNING_ACTION_TYPE {
    DRIVING,           // Without overtaking, driving
    START_OVERTAKING,  // Start overtaking to the right or left. (Before merging
                       // to the another line)
    SIDE_BY_SIDE,      // Merged to the another line and driving side-by-side
    FINISH_OVERTAKING, // Ego vehicle is infront of the opponent and merging
                       // back to the original racing line
    ABORT_OVERTAKING // Tried to overtake but somehow it failed. Merging back to
                     // the original racing line
  };

public:
  AdaptiveDynamicPlannerNode(const std::string& node_name_);

  void
  detectionResultCallback(const nif_msgs::msg::Perception3D::SharedPtr msg);
  void mapTrackBodyCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void mapTrackGlobalCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void predictionResultCallback(
      const nif_msgs::msg::DynamicTrajectory::SharedPtr msg);

  void loadConfig(const std::string& planning_config_file_);

  tuple<vector<double>, vector<double>>
  loadCSVfile(const std::string& wpt_file_path_);

  void timer_callback();
  void timer_callback_v2();
  void publishTrajectory();
  void initOutputTrajectory();
  bool setDrivingMode();

  bool checkOverTake();

  double getProgress(const geometry_msgs::msg::Pose& pt_global_,
                     pcl::KdTreeFLANN<pcl::PointXY>& target_tree_);

  double getProgress(const double& pt_x_,
                     const double& pt_y_,
                     pcl::KdTreeFLANN<pcl::PointXY>& target_tree_);

  double getCurIdx(const double& pt_x_,
                   const double& pt_y_,
                   pcl::KdTreeFLANN<pcl::PointXY>& target_tree_);

  double calcCTE(const geometry_msgs::msg::Pose& pt_global_,
                 pcl::KdTreeFLANN<pcl::PointXY>& target_tree_,
                 pcl::PointCloud<pcl::PointXY>::Ptr& pc_);

  tuple<double, double>
  calcProgressNCTE(const geometry_msgs::msg::Pose& pt_global_,
                   pcl::KdTreeFLANN<pcl::PointXY>& target_tree_,
                   pcl::PointCloud<pcl::PointXY>::Ptr& pc_);

  double calcProgressDiff(
      const geometry_msgs::msg::Pose& ego_pt_global_,
      const geometry_msgs::msg::Pose& target_pt_global_,
      pcl::KdTreeFLANN<pcl::PointXY>&
          target_tree_); // negative : ego vehicle is in front of the vehicle
                         // positive : target is in front of the ego vehicle

  nav_msgs::msg::Path xyyawVec2Path(std::vector<double>& x_,
                                    std::vector<double>& y_,
                                    std::vector<double>& yaw_rad_);

  pcl::PointCloud<pcl::PointXY>::Ptr genPointCloudFromVec(vector<double>& x_,
                                                          vector<double>& y_);

  int calcCurIdxFromDynamicTraj(const nif_msgs::msg::DynamicTrajectory& msg);

  void registerPath(nav_msgs::msg::Path& frenet_segment_,
                    nav_msgs::msg::Path& origin_path_);

  std::shared_ptr<FrenetPath> getFrenetToRacingLine();

  double calcProgressDiff(const nif_msgs::msg::DynamicTrajectory& ego_traj_,
                          const nif_msgs::msg::DynamicTrajectory& oppo_traj_,
                          pcl::KdTreeFLANN<pcl::PointXY>& target_tree_);

  bool collisionCheckBTWtrajs(
      const nif_msgs::msg::DynamicTrajectory& ego_traj_,
      const nif_msgs::msg::DynamicTrajectory& oppo_traj_,
      const double collision_dist_boundary,
      const double
          collision_time_boundary); // if there is collision, return true.

  bool collisionCheckBTWtrajsNFrenet(
      std::shared_ptr<FrenetPath> ego_frenet_traj_,
      const nif_msgs::msg::DynamicTrajectory& oppo_traj_,
      const double collision_dist_boundary,
      const double
          collision_time_boundary); // if there is collision, return true.

  nif_msgs::msg::DynamicTrajectory
  stitchFrenetToPath(std::shared_ptr<FrenetPath>& frenet_segment_,
                     pcl::KdTreeFLANN<pcl::PointXY>& target_tree_,
                     nav_msgs::msg::Path& target_path_);

private:
  // Opponent perception result (not prediction result)
  rclcpp::Subscription<nif_msgs::msg::Perception3D>::SharedPtr m_det_sub;
  // Map track from wpt manager
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_maptrack_body_sub;
  // Map track from wpt manager
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_maptrack_global_sub;
  // Opponent's future DynamicTrajectory result
  rclcpp::Subscription<nif_msgs::msg::DynamicTrajectory>::SharedPtr
      m_oppo_pred_sub;

  // Ego's planned output DynamicTrajectory
  rclcpp::Publisher<nif_msgs::msg::DynamicTrajectory>::SharedPtr
      m_ego_traj_body_pub;
  rclcpp::Publisher<nif_msgs::msg::DynamicTrajectory>::SharedPtr
      m_ego_traj_global_pub;
  // output timer
  rclcpp::TimerBase::SharedPtr m_planner_timer;
  bool m_timer_callback_first_run;

  nif_msgs::msg::Perception3D m_cur_det;
  nif_msgs::msg::Perception3D m_prev_det;
  bool m_det_callback_first_run;

  nif_msgs::msg::DynamicTrajectory m_cur_oppo_pred_result;
  nif_msgs::msg::DynamicTrajectory m_prev_oppo_pred_result;
  bool m_oppo_pred_callback_first_run;

  nav_msgs::msg::Path m_maptrack_body, m_maptrack_global;

  nif_msgs::msg::DynamicTrajectory m_cur_ego_planned_result_body;
  nif_msgs::msg::DynamicTrajectory m_prev_ego_planned_result_body;
  nif_msgs::msg::DynamicTrajectory m_cur_ego_planned_result_global;
  nif_msgs::msg::DynamicTrajectory m_prev_ego_planned_result_global;

  double m_ego_progress, m_oppo_progress;           // [m]
  double m_ego_cur_speed_mps, m_oppo_cur_speed_mps; // [mps]

  // TODO : check this unit (it can be used as a term for consistency of path
  // planning) --> positive : left / negative : right
  double m_cur_steer_angle;

  bool m_overtake_allowed_flg; // Set by "system status manager". default : true
  bool m_emergency_flg;        // TODO : If emergency flag is true, then what?

  PLANNING_DECISION_TYPE m_cur_decision;
  PLANNING_DECISION_TYPE m_prev_decision;

  PLANNING_ACTION_TYPE m_cur_overtaking_action;
  PLANNING_ACTION_TYPE m_prev_overtaking_action;

  int m_num_overtaking_candidates; // number of lines for overtaking
  std::vector<nav_msgs::msg::Path> m_overtaking_candidates_path_vec;
  std::vector<nif_msgs::msg::DynamicTrajectory>
      m_overkaing_candidates_dtraj_vec;
  std::vector<pcl::PointCloud<pcl::PointXY>::Ptr>
      m_overtaking_candidates_path_pc_vec; // for kdtree search
  std::vector<pcl::KdTreeFLANN<pcl::PointXY>>
      m_overtaking_candidates_path_kdtree_vec;
  std::vector<std::string> m_overtaking_candidates_file_path_vec;
  std::vector<std::string> m_overtaking_candidates_alias_vec;
  std::vector<FrenetPathGenerator::CubicSpliner2DResult>
      m_overtaking_candidates_spline_data_vec;
  std::vector<std::shared_ptr<CubicSpliner2D>>
      m_overtaking_candidates_spline_model_vec;

  std::string m_racingline_file_path;
  std::vector<double> m_racingline_x_vec, m_racingline_y_vec;
  nav_msgs::msg::Path m_racingline_path;
  nif_msgs::msg::DynamicTrajectory m_racingline_dtraj;
  pcl::PointCloud<pcl::PointXY>::Ptr m_racingline_path_pc;
  pcl::KdTreeFLANN<pcl::PointXY> m_racineline_path_kdtree;
  FrenetPathGenerator::CubicSpliner2DResult m_racingline_spline_data;

  // OUTPUT
  int m_planned_traj_len;
  int m_ego_cur_idx_in_planned_traj;
  nif_msgs::msg::DynamicTrajectory m_cur_planned_traj; // full path

  std::string m_planning_config_file_path;
  std::string m_tracking_topic_name;
  std::string m_prediction_topic_name;
  std::string m_map_root_path;

  bool m_config_load_success;
  // configuration param for planning
  double m_config_planning_horizon;
  double m_config_planning_dt;
  double m_config_max_accel;
  double m_config_overtaking_longitudinal_margin; // [m] longitudinal wise
                                                  // distance margin to obey
                                                  // when starts overtaking
  double m_config_overtaking_lateral_margin; // [m] lateral wise distance margin
                                             // to obey
  double m_config_merging_longitudinal_margin; // [m] longitudinal wise distance
                                               // margin to obey when starts
                                               // merging
  double m_config_follow_enable_dist;          // [m]
  double m_config_spline_interval;             // [m]
  double m_config_merge_allow_dist;            // [m]
  double m_config_overlap_checking_dist_bound; // [m]
  double m_config_overlap_checking_time_bound; // [sec]

  shared_ptr<FrenetPathGenerator> m_frenet_generator_ptr;

  nav_msgs::msg::Odometry m_ego_odom;
};

} // namespace planning
} // namespace nif

#endif // ROS2MASTER_DYNAMIC_PLANNER_NODE_H
