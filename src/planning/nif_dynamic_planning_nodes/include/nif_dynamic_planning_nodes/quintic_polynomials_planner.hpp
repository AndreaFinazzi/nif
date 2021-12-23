#ifndef __QUINTIC_POLYNOMIALS_PLANNER_H__
#define __QUINTIC_POLYNOMIALS_PLANNER_H__

#include "nif_opponent_prediction_nodes/quintic_polynomial.h"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nif_msgs/msg/dynamic_trajectory.hpp>
#include <nif_utils/utils.h>

static bool abs_compare(int a, int b) { return (std::abs(a) < std::abs(b)); }

class quintic_polynomials_planner {
private:
  /* data */

  nav_msgs::msg::Odometry m_plan_start_odom; // position,orientation, speed and
                                             // acceleration are needed.
  nav_msgs::msg::Odometry m_plan_goal_odom;  // position,orientation, speed and
                                             // acceleration are needed.

  nif_msgs::msg::DynamicTrajectory m_planeed_traj;

  bool m_goal_reach_flg;

  double m_constraint_max_accel;
  double m_constraint_max_jerk;
  double m_constraint_min_t;
  double m_constraint_max_t;
  double m_config_dt;

public:
  quintic_polynomials_planner();
  quintic_polynomials_planner(double constraint_min_t, double constraint_max_t,
                              double constraint_max_accel,
                              double constraint_max_jerk, double config_dt);
  ~quintic_polynomials_planner();

  void setStartOdom(nav_msgs::msg::Odometry &m_start_odom_);
  void setGoalOdom(nav_msgs::msg::Odometry &m_goal_odom_);
  nif_msgs::msg::DynamicTrajectory getPlaneedTraj();
  void setPlaneedTraj(nif_msgs::msg::DynamicTrajectory &m_planeed_traj_);
  double getConstraintMaxAccel();
  void setConstraintMaxAccel(double constraint_max_accel_);
  double getConstraintMaxJerk();
  void setConstraintMaxJerk(double constraint_max_jerk_);
  double getConfigDt();
  void setConfigDt(double config_dt_);
  double getConstraintMinT();
  void setConstraintMinT(double m_constraint_min_t);
  double getConstraintMaxT();
  void setConstraintMaxT(double m_constraint_max_t);

  void configCheck();

  nif_msgs::msg::DynamicTrajectory solve();
};

#endif // __QUINTIC_POLYNOMIALS_PLANNER_H__