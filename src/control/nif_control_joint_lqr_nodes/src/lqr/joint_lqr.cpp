#include "nif_control_joint_lqr_nodes/lqr/joint_lqr.h"
#include "nif_control_joint_lqr_nodes/lqr/joint_dynamics.h"
#include "nif_control_joint_lqr_nodes/lqr/solver.h"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

#include <rclcpp/rclcpp.hpp>

namespace joint_lqr {
namespace lqr {

JointLQR::JointLQR(std::string config_file) { loadConfig(config_file); }

void JointLQR::loadConfig(std::string config_file) {
  YAML::Node config = YAML::LoadFile(config_file);

  //! A few checks for a valid yaml, let YAML throw the rest if any values are
  //! missing
  if (!config["model_params"]) {
    throw std::runtime_error("model_params not defined in config file.");
  }
  if (!config["velocity_brackets"]) {
    throw std::runtime_error("velocity_brackets not defined in config file.");
  }
  //! Load model parameters
  YAML::Node model_params = config["model_params"];
  //! apply model parameters to member variable, model.
  // JointDynamics::ModelParams model;
  model.cornering_stiffness_front =
      model_params["cornering_stiffness_front"].as<double>();
  model.cornering_stiffness_rear =
      model_params["cornering_stiffness_rear"].as<double>();
  model.mass = model_params["mass"].as<double>();
  model.length_front = model_params["length_front"].as<double>();
  model.length_rear = model_params["length_rear"].as<double>();
  model.yaw_moment_inertia = model_params["yaw_moment_inertia"].as<double>();
  model.drag_coeff = model_params["drag_coeff"].as<double>();

  //! Use those params to generate the model and generate feedback matrices
  JointDynamics bicycle_model(model);
  KMatrices new_matrices;
  YAML::Node velocity_brackets = config["velocity_brackets"];
  for (std::size_t i = 0; i < velocity_brackets.size(); ++i) {
    //! Get A and B matrices
    double velocity = velocity_brackets[i]["velocity"].as<double>();
    auto AB = bicycle_model.linearize(velocity);

    //! Generate Q and R matrices
    YAML::Node q_trace = velocity_brackets[i]["q_trace"];
    YAML::Node r_trace = velocity_brackets[i]["r_trace"];
    JointDynamics::QMatrix Q = JointDynamics::QMatrix::Zero();
    JointDynamics::RMatrix R = JointDynamics::RMatrix::Zero();
    for (std::size_t i = 0; i < q_trace.size(); ++i) {
      Q(i, i) = q_trace[i].as<double>();
    }
    for (std::size_t i = 0; i < r_trace.size(); ++i) {
      R(i, i) = r_trace[i].as<double>();
    }

    //! Calculate feedback matrix with lqr
    Eigen::MatrixXd K;
    lqr(AB.first, AB.second, Q, R, K);

    new_matrices.push_back({velocity, K});
  }
  //! Make sure the config is valid before setting matrices_
  matrices_ = new_matrices;
} /* loadConfig() */

JointLQR::ErrorMatrix JointLQR::computeError(JointLQR::StateMatrix state,
                                             JointLQR::GoalMatrix goal) {
  //! Do some frame shifting
  double x_goal_shift = goal(0, 0) - state(0, 0);
  double y_goal_shift = goal(1, 0) - state(1, 0);
  double y_goal = x_goal_shift * std::sin(-state(4, 0)) +
                  y_goal_shift * std::cos(-state(4, 0));
  double yaw_error = wrap_angle(state(4, 0) - goal(2, 0));
  double v_error = state(2, 0) - goal(3, 0);

  //! Assign results and return
  JointLQR::ErrorMatrix result;
  result(0, 0) = -y_goal;
  result(1, 0) = state(3, 0) + state(2, 0) * yaw_error;
  result(2, 0) = yaw_error;
  //! This could be calculated better based on curvature..
  //! But we have no decent yaw rate estimate any :shrug:
  result(3, 0) = state(5, 0);
  result(4, 0) = v_error;
  return result;
}

JointLQR::ControlVector
JointLQR::computeFFControl(JointLQR::ErrorMatrix error,
                           JointLQR::StateMatrix state) {
  //! Compute feedforward control for the compensation of linearization
  JointLQR::ControlVector FFControl;
  // Feedforward control
  //  xdot = Ax + Bu + g
  //  xdot = Ax + B * (-Kx - uff) + g
  //  xdot = (A-BK)x - B*uff + g

  // Feedforward for lateral model
  // TODO: curvature input is needed
  // steer_ff =
  //  L * kappa + Kv * ay
  //  where, Kv = mf / (2*Cf) - mr / (s*Cr)
  double steer_ff = 0.;

  // Feedforward for longitudinal model
  // accel_ff =
  //  2./m * Cd*vx*error_v - Cr/mass - 1./mass * Cd*vx**2 - vy*yaw_rate;
  // TODO: Rolling resistance should be considered as Fz-dependent term.
  double accel_ff =
      2. / model.mass * model.drag_coeff * state(2, 0) * error(4, 0) -
      1. / model.mass * model.drag_coeff * std::pow(state(2, 0), 2) -
      state(3, 0) * state(5, 0);

  // Negative as 'u = -Kx - uff'
  FFControl(0, 0) = -steer_ff;
  FFControl(1, 0) = -accel_ff;

  // std::cout << "Feedforward - steer : " << FFControl(0, 0);
  // std::cout << "\nFeedforward - accel : " << FFControl(1, 0) << std::endl;

  return FFControl;
}

JointLQR::ControlVector JointLQR::process(JointLQR::ErrorMatrix error,
                                          double x_vel_body) {
  JointLQR::KMatrix matrix;
  // Find the tier where the upper limit is higher than the current velocity
  // If no tier contains this velocity it will use the last tier
  for (const auto &matrice : matrices_) {
    matrix = matrice.second;
    if (matrice.first > x_vel_body) {
      break;
    }
  }
  // Compute feedback control 2x1, (steer, accel)
  auto FBControl = -matrix * error;

  RCLCPP_DEBUG(rclcpp::get_logger("joint_lqr_solver"),
               "matrix K e y        : %f", matrix(0, 0));
  RCLCPP_DEBUG(rclcpp::get_logger("joint_lqr_solver"),
               "\nmatrix K e ydot   : %f", matrix(0, 1));
  RCLCPP_DEBUG(rclcpp::get_logger("joint_lqr_solver"),
               "\nmatrix K e yaw    : %f", matrix(0, 2));
  RCLCPP_DEBUG(rclcpp::get_logger("joint_lqr_solver"),
               "\nmatrix K e yawdot : %f", matrix(0, 3));
  RCLCPP_DEBUG(rclcpp::get_logger("joint_lqr_solver"),
               "\nmatrix K e vx     : %f", matrix(1, 4));

  RCLCPP_DEBUG(rclcpp::get_logger("joint_lqr_solver"),
               "\nFeedback    - steer : %f", FBControl(0, 0));
  RCLCPP_DEBUG(rclcpp::get_logger("joint_lqr_solver"),
               "\nFeedback    - accel : %f", FBControl(1, 0));

  return FBControl;
}

JointLQR::ControlVector JointLQR::process(JointLQR::StateMatrix state,
                                          JointLQR::GoalMatrix goal) {
  // Convert to error matrix, x vel body
  // and call the other process function
  auto error = computeError(state, goal);
  auto feedback_cmd = process(error, state(2, 0));
  auto feedforward_cmd = computeFFControl(error, state);

  return feedback_cmd + feedforward_cmd;
}

JointLQR::KMatrices &JointLQR::getKMatrices() { return matrices_; }

double JointLQR::wrap_angle(double theta) {
  while (theta >= M_PI) {
    theta -= 2.0 * M_PI;
  }
  while (theta < -M_PI) {
    theta += 2.0 * M_PI;
  }
  return theta;
}

} /* namespace lqr */
} /* namespace joint_lqr */
