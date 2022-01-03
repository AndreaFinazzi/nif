#include "nif_dynamic_planning_nodes/quintic_polynomials_planner.hpp"

quintic_polynomials_planner::quintic_polynomials_planner(/* args */) {
  // laod default constraints and config.

  double default_constraint_min_t = 1.0;
  double default_constraint_max_t = 4.0;
  double default_constraint_max_accel = 2.0;
  double default_constraint_max_jerk = 5.0;
  double default_config_dt = 0.2;

  quintic_polynomials_planner(default_constraint_min_t,
                              default_constraint_max_t,
                              default_constraint_max_accel,
                              default_constraint_max_jerk, default_config_dt);
}

quintic_polynomials_planner::~quintic_polynomials_planner() {}

quintic_polynomials_planner::quintic_polynomials_planner(
    double constraint_min_t, double constraint_max_t,
    double constraint_max_accel, double constraint_max_jerk, double config_dt) {
  // laod default constraints and config.

  m_constraint_min_t = constraint_min_t;
  m_constraint_max_t = constraint_max_t;
  m_constraint_max_accel = constraint_max_accel;
  m_constraint_max_jerk = constraint_max_jerk;
  m_config_dt = config_dt;

  configCheck();
}

nif_msgs::msg::DynamicTrajectory quintic_polynomials_planner::solve() {

  auto sv = m_plan_start_odom.twist.twist.linear.x; // [mps] start velocity
  auto gv = m_plan_goal_odom.twist.twist.linear.x;  // [mps] goal velocity
  auto sa = 0.0;                                    // [mpss] start accel
  auto ga = 0.0;                                    // [mpss] goal accel

  auto syaw = nif::common::utils::coordination::quat2yaw(
      m_plan_start_odom.pose.pose.orientation);
  auto gyaw = nif::common::utils::coordination::quat2yaw(
      m_plan_goal_odom.pose.pose.orientation);

  auto vxs = sv * cos(syaw);
  auto vys = sv * sin(syaw);
  auto vxg = gv * cos(gyaw);
  auto vyg = gv * sin(gyaw);

  auto axs = sa * cos(syaw);
  auto ays = sa * sin(syaw);
  auto axg = ga * cos(gyaw);
  auto ayg = ga * sin(gyaw);

  for (int plan_horizon = m_constraint_min_t;
       plan_horizon <= m_constraint_max_t; plan_horizon = plan_horizon + 1) {
    auto xqp = QuinticPolynomial(m_plan_start_odom.pose.pose.position.x, vxs,
                                 axs, m_plan_goal_odom.pose.pose.position.x,
                                 vxg, axg, plan_horizon);
    auto yqp = QuinticPolynomial(m_plan_start_odom.pose.pose.position.y, vys,
                                 ays, m_plan_goal_odom.pose.pose.position.y,
                                 vyg, ayg, plan_horizon);

    nif_msgs::msg::DynamicTrajectory out;
    std::vector<double> accel_vec;
    std::vector<double> jerk_vec;

    for (double plan_t = 0.0; plan_t < plan_horizon + m_config_dt;
         plan_t = plan_t + m_config_dt) {
      geometry_msgs::msg::PoseStamped ps;
      ps.pose.position.x = xqp.calc_point(plan_t);
      ps.pose.position.y = yqp.calc_point(plan_t);

      auto vx = xqp.calculate_first_derivative(plan_t);
      auto vy = yqp.calculate_first_derivative(plan_t);
      auto v = sqrt(vx * vx + vy * vy);
      auto yaw = atan2(vy, vx);
      ps.pose.orientation =
          nif::common::utils::coordination::euler2quat(yaw, 0.0, 0.0);

      out.trajectory_path.poses.push_back(ps);
      out.trajectory_velocity.push_back(v);
      out.trajectory_timestamp_array.push_back(plan_t);

      auto ax = xqp.calculate_second_derivative(plan_t);
      auto ay = yqp.calculate_second_derivative(plan_t);
      auto a = sqrt(ax * ax + ay * ay);

      if (out.trajectory_velocity.size() >= 2 &&
          (out.trajectory_velocity.rbegin()[0] -
           out.trajectory_velocity.rbegin()[1]) < 0.0) {
        a *= -1;
      }
      accel_vec.push_back(a);

      auto jx = xqp.calculate_third_derivative(plan_t);
      auto jy = yqp.calculate_third_derivative(plan_t);
      auto j = sqrt(jx * jx + jy * jy);

      if (accel_vec.size() >= 2 &&
          (accel_vec.rbegin()[0] - accel_vec.rbegin()[1]) < 0.0) {
        j *= -1;
      }
      jerk_vec.push_back(j);
    }

    auto max_abs_a = *std::max_element(
        accel_vec.begin(), accel_vec.end(),
        [](const int &a, const int &b) { return abs(a) < abs(b); });
    auto max_abs_jerk = *std::max_element(
        jerk_vec.begin(), jerk_vec.end(),
        [](const int &a, const int &b) { return abs(a) < abs(b); });

    if (max_abs_a < m_constraint_max_accel &&
        max_abs_jerk < m_constraint_max_jerk) {
      // found path
      return out;
    }
  }

  // Not reached
  // return empty traj
  nif_msgs::msg::DynamicTrajectory empty_out;
  return empty_out;
}

void quintic_polynomials_planner::configCheck() {

  if (m_constraint_min_t <= 0) {
    std::runtime_error(
        "[QUINTIC_POLYNOMIALS_PLANNER] : m_constraint_min_t can not be less "
        "than zero. Check the configuration.");
  }

  if (m_constraint_max_t <= 0) {
    std::runtime_error(
        "[QUINTIC_POLYNOMIALS_PLANNER] : m_constraint_max_t can not be less "
        "than zero. Check the configuration.");
  }

  if (m_constraint_max_t <= m_constraint_min_t) {
    std::runtime_error(
        "[QUINTIC_POLYNOMIALS_PLANNER] : m_constraint_max_t can not be less "
        "than m_constraint_min_t. Check the configuration.");
  }

  if (m_config_dt <= 0) {
    std::runtime_error("[QUINTIC_POLYNOMIALS_PLANNER] : dt can not be less "
                       "than zero. Check the configuration.");
  }
}

void quintic_polynomials_planner::setStartOdom(
    nav_msgs::msg::Odometry &m_start_odom_) {
  this->m_plan_start_odom = m_start_odom_;
}

void quintic_polynomials_planner::setGoalOdom(
    nav_msgs::msg::Odometry &m_goal_odom_) {
  this->m_plan_goal_odom = m_goal_odom_;
}

nif_msgs::msg::DynamicTrajectory quintic_polynomials_planner::getPlaneedTraj() {
  return this->m_planeed_traj;
}

void quintic_polynomials_planner::setPlaneedTraj(
    nif_msgs::msg::DynamicTrajectory &m_planeed_traj_) {
  this->m_planeed_traj = m_planeed_traj_;
}

double quintic_polynomials_planner::getConstraintMaxAccel() {
  return this->m_constraint_max_accel;
}

void quintic_polynomials_planner::setConstraintMaxAccel(
    double constraint_max_accel_) {
  this->m_constraint_max_accel = constraint_max_accel_;
}

double quintic_polynomials_planner::getConstraintMaxJerk() {
  return this->m_constraint_max_jerk;
}

void quintic_polynomials_planner::setConstraintMaxJerk(
    double constraint_max_jerk_) {
  this->m_constraint_max_jerk = constraint_max_jerk_;
}

double quintic_polynomials_planner::getConfigDt() { return this->m_config_dt; }

void quintic_polynomials_planner::setConfigDt(double config_dt_) {
  this->m_config_dt = config_dt_;
}

double quintic_polynomials_planner::getConstraintMinT() {
  return this->m_constraint_min_t;
}
void quintic_polynomials_planner::setConstraintMinT(double m_constraint_min_t) {
  this->m_constraint_min_t = m_constraint_min_t;
}

double quintic_polynomials_planner::getConstraintMaxT() {
  return this->m_constraint_max_t;
}

void quintic_polynomials_planner::setConstraintMaxT(double m_constraint_max_t) {
  this->m_constraint_max_t = m_constraint_max_t;
}
