#ifndef __VELOCITY_PROFILER_H__d
#define __VELOCITY_PROFILER_H__d

#include "nif_common/constants.h"
#include "nif_vehicle_dynamics_manager/tire_manager.hpp"
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <math.h>
#include "nif_opponent_prediction_nodes/frenet_path_generator.h"
#include <yaml-cpp/yaml.h>
#include "nif_msgs/msg/dynamic_trajectory.hpp"

#define MIN_SPEED_MPS 1.0

class velocity_profiler
{
public:
  velocity_profiler();
  ~velocity_profiler() {}

  velocity_profiler(std::string config_file_path_);

  void setConfigUseVehModel(bool value_);
  void setConfigUseACCModel(bool value_);
  void setConfigUseCurvatureMode(bool value_);

  bool setConstraintMaxT(double value_);      // [sec]
  bool setConstraintMinT(double value_);      // [sec]
  bool setConstraintMaxAccel(double value_);  // [mpss]
  bool setConstraintMaxDeccel(double value_); // [mpss]
  bool setConstraintMaxVel(double value_);    // [mps]

  bool checkConfig();
  bool parseConfig_(std::string & config_file_path_);

  nif_msgs::msg::DynamicTrajectory
  velProfile(
    const nav_msgs::msg::Odometry & odom_,
    const nav_msgs::msg::Path & target_path_,
    const double & spline_interval_);

  nif_msgs::msg::DynamicTrajectory
  velProfilewithDynamics(
    const nav_msgs::msg::Odometry & odom_,
    const nav_msgs::msg::Path & target_path_,
    const double & spline_interval_);

  nif_msgs::msg::DynamicTrajectory
  velProfileForAcc(
    const nav_msgs::msg::Odometry & odom_,
    const nif_msgs::msg::DynamicTrajectory & cipv_predicted_traj_,
    const double & cipv_vel_abs_,
    const nav_msgs::msg::Path & target_path_,
    const double & spline_interval_);

private:
  /* data */

  std::string m_config_path;

  // velocity profiling related parmas
  double m_constraint_max_t;
  double m_constraint_min_t;
  double m_constraint_max_accel;
  double m_constraint_max_lat_accel;
  double m_constraint_max_deccel;
  double m_constraint_max_vel;

  bool m_config_use_veh_model = false;
  bool m_config_use_acc_model = false;
  bool m_config_use_curvature_model = false;
  double m_config_dt;

  // ACC related parmas
  double m_acc_config_s0;
  double m_acc_config_s1;
  double m_acc_config_v_desired;
  double m_acc_config_time_headway;
  double m_acc_config_accel_max;
  double m_acc_config_decel_desired;
  double m_acc_config_delta;
  double m_acc_config_veh_l;

  nav_msgs::msg::Odometry m_cur_odom;
  nav_msgs::msg::Path m_profile_target_path;

  nif_msgs::msg::DynamicTrajectory m_profiled_traj;
  nif_msgs::msg::DynamicTrajectory m_predicted_cipv_traj;

  // Tire dynamics
  double m_lat_tire_factor = 1.0; // "Parameter lateral_tire_model_factor must
                                  // be lower or equal than 1.0 "
  TireManager m_tire_manager;
};

#endif // __VELOCITY_PROFILER_H__
