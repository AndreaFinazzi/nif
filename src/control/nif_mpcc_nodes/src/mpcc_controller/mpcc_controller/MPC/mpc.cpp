// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "mpc.h"

namespace mpcc {
MPC::MPC() : Ts_(1.0) {
  std::cout
      << "(mpc) default constructor, not everything is initialized properly"
      << std::endl;
}

MPC::MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts,
         const PathToJson &path)
    : Ts_(Ts), valid_initial_guess_(false),
      solver_interface_(new HpipmInterface()), param_(Param(path.param_path)),
      normalization_param_(NormalizationParam(path.normalization_path)),
      bounds_(BoundsParam(path.bounds_path)),
      constraints_(Constraints(Ts, path)), cost_(Cost(path)),
      integrator_(Integrator(Ts, path)), model_(Model(Ts, path)),
      track_(ArcLengthSpline(path)), track_i(ArcLengthSpline(path)),
      track_o(ArcLengthSpline(path)) {
  std::cout << "(mpc) Openning params at " << path.param_path << std::endl;
  n_sqp_ = n_sqp;
  sqp_mixing_ = sqp_mixing;
  n_non_solves_ = 0;
  n_no_solves_sqp_ = 0;
  n_reset_ = n_reset;
  cost_original_ = cost_;
  std::vector<double> temp(N, 0);
  centerline_yaw_error_ = temp;
  Cr2_default_ = param_.Cr2; // 0.542445 = 0.5*1.2*1.247*0.725
  driven_dist_ = 0;
  std::cout << "Initialized mpc" << std::endl;
}

void MPC::setOvertakeCost(const PathToJson &path) {
  cost_overtake_ = Cost(path);
}

void MPC::setSecondCost(const PathToJson &path) { cost_follow_ = Cost(path); }

void MPC::setOppoMode(bool is_oppo_mpc) {
  constraints_.is_oppo_mpc_ = is_oppo_mpc;
  is_oppo_mpc_ = is_oppo_mpc;
}

void MPC::setMPCProblem() {
  pos_outer_.clear();
  pos_inner_.clear();
  for (int i = 0; i <= N; i++) {
    if (i != N) {
      // 0 ~ N-1th stage
      setStage(initial_guess_[i].xk, initial_guess_[i].uk,
               initial_guess_[i + 1].xk, i);
    } else {
      // Nth stage
      State xk1;
      propagateOneStep(initial_guess_[i].xk, initial_guess_[i].uk, xk1);
      setStage(initial_guess_[i].xk, initial_guess_[i].uk, xk1, i);
    }
    pos_outer_.push_back(constraints_.pos_outer_xy_);
    pos_inner_.push_back(constraints_.pos_inner_xy_);
    // update pos_center
    std::vector<double> pos_center_element;
    pos_center_element = {
        0.5 * (constraints_.pos_inner_xy_[0] + constraints_.pos_outer_xy_[0]),
        0.5 * (constraints_.pos_inner_xy_[1] + constraints_.pos_outer_xy_[1])};
    pos_center_.push_back(pos_center_element);
  }
  // update centerline
  updateCenterlineYawError();
}

void MPC::setStage(State &xk, const Input &uk, State &xk1,
                   const int time_step) {
  stages_[time_step].nx = NX;
  stages_[time_step].nu = NU;

  if (time_step == 0) {
    stages_[time_step].ng = 0;
    stages_[time_step].ns = 0;
  } else {
    stages_[time_step].ng = NPC;
    stages_[time_step].ns = NS;
  }

  // Update centerline yaw error
  model_.center_yaw_error_ = centerline_yaw_error_[time_step];
  if (time_step == 0) {
    std::cout << "Bank angle" << xk.B << std::endl;
  }
  // TODO : issue case 1
  double initial_race_time = 2.0;
  // if (initial_raceline_ == 1 && this->time_elapsed_ < initial_race_time)
  // {
  //   constraints_.amount_modulation = 3.0;
  // }
  // else
  // {
  //   constraints_.amount_modulation = 0.0;
  // }

  constraints_.amount_modulation = 0.0;

  // Update model parameters
  // Get Model param
  // [Bf, Cf, Df, Br, Cr, Dr, Cm1, Cm1_brake]
  double vel = sqrt(pow(xk.vx, 2.0) + pow(xk.vy, 2.0));
  // std::vector<double> model_params =
  //     model_manager_->getModelParam(vel, driven_dist_);
  // param_.Bf = model_params[0];
  // param_.Cf = model_params[1];
  // param_.Df = model_params[2];
  // param_.Br = model_params[3];
  // param_.Cr = model_params[4];
  // param_.Dr = model_params[5];
  // param_.Cm1 = model_params[6];
  // param_.Cm1_brake = model_params[7];

  // Aero param
  State xk_oppo;
  double delta_x = 0, delta_y = 0;

  param_.Cr2 = Cr2_default_;
  for (int oppo_idx = 0; oppo_idx < considering_paths_.size(); oppo_idx++) {
    xk_oppo.X = (double)considering_paths_[oppo_idx].path_data_.X(time_step);
    xk_oppo.Y = (double)considering_paths_[oppo_idx].path_data_.Y(time_step);
    xk_oppo.phi = (double)considering_paths_phi_[oppo_idx](time_step);
    getPosDiffAtOppoFrame(xk, xk_oppo, delta_x, delta_y);

    param_.Cr2 = std::max(0.0, param_.Cr2);
  }

  State xk_nz = xk;
  xk_nz.vxNonZero(param_.vx_zero);
  State xk1_nz = xk1;
  xk1_nz.vxNonZero(param_.vx_zero);

  stages_[time_step].cost_mat = normalizeCost(
      cost_.getCost(num_of_opponents_, tracks_, considering_paths_phi_, track_,
                    xk_nz, uk, time_step));
  stages_[time_step].lin_model =
      normalizeDynamics(model_.getLinModel(xk_nz, uk, xk1_nz));
  stages_[time_step].constrains_mat = normalizeCon(
      constraints_.getConstraints(track_, track_i, track_o, xk_nz, uk));

  stages_[time_step].l_bounds_x =
      normalization_param_.T_x_inv * bounds_.getBoundsLX(xk_nz);
  stages_[time_step].u_bounds_x =
      normalization_param_.T_x_inv * bounds_.getBoundsUX(xk_nz);
  stages_[time_step].l_bounds_u =
      normalization_param_.T_u_inv * bounds_.getBoundsLU(uk);
  stages_[time_step].u_bounds_u =
      normalization_param_.T_u_inv * bounds_.getBoundsUU(uk);
  stages_[time_step].l_bounds_s =
      normalization_param_.T_s_inv * bounds_.getBoundsLS();
  stages_[time_step].u_bounds_s =
      normalization_param_.T_s_inv * bounds_.getBoundsUS();

  stages_[time_step].l_bounds_x(si_index.s) =
      normalization_param_.T_x_inv(si_index.s, si_index.s) *
      (-param_.s_trust_region); //*initial_guess_[time_step].xk.vs;
  stages_[time_step].u_bounds_x(si_index.s) =
      normalization_param_.T_x_inv(si_index.s, si_index.s) *
      (param_.s_trust_region); //*initial_guess_[time_step].xk.vs;
}

CostMatrix MPC::normalizeCost(const CostMatrix &cost_mat) {
  const Q_MPC Q =
      normalization_param_.T_x * cost_mat.Q * normalization_param_.T_x;
  const R_MPC R =
      normalization_param_.T_u * cost_mat.R * normalization_param_.T_u;
  const q_MPC q = normalization_param_.T_x * cost_mat.q;
  const r_MPC r = normalization_param_.T_u * cost_mat.r;
  const Z_MPC Z =
      normalization_param_.T_s * cost_mat.Z * normalization_param_.T_s;
  const z_MPC z = normalization_param_.T_s * cost_mat.z;
  return {Q, R, S_MPC::Zero(), q, r, Z, z};
}

LinModelMatrix MPC::normalizeDynamics(const LinModelMatrix &lin_model) {
  const A_MPC A =
      normalization_param_.T_x_inv * lin_model.A * normalization_param_.T_x;
  const B_MPC B =
      normalization_param_.T_x_inv * lin_model.B * normalization_param_.T_u;
  const g_MPC g = normalization_param_.T_x_inv * lin_model.g;
  return {A, B, g};
}

ConstrainsMatrix MPC::normalizeCon(const ConstrainsMatrix &con_mat) {
  const C_MPC C = con_mat.C * normalization_param_.T_x;
  const D_MPC D = con_mat.D * normalization_param_.T_u;
  const d_MPC dl = con_mat.dl;
  const d_MPC du = con_mat.du;
  return {C, D, dl, du};
}

std::array<OptVariables, N + 1>
MPC::deNormalizeSolution(const std::array<OptVariables, N + 1> &solution) {
  std::array<OptVariables, N + 1> denormalized_solution;
  StateVector updated_x_vec;
  InputVector updated_u_vec;
  for (int i = 0; i <= N; i++) {
    updated_x_vec = normalization_param_.T_x * stateToVector(solution[i].xk);
    updated_u_vec = normalization_param_.T_u * inputToVector(solution[i].uk);

    denormalized_solution[i].xk = vectorToState(updated_x_vec);
    denormalized_solution[i].uk = vectorToInput(updated_u_vec);
  }
  return denormalized_solution;
}

void MPC::updateInitialGuess(const State &x0) {
  for (int i = 1; i < N; i++)
    initial_guess_[i - 1] = initial_guess_[i];

  initial_guess_[0].xk = x0;
  // initial_guess_[0].uk.setZero();

  // initial_guess_[N - 1].xk = initial_guess_[N - 2].xk;
  // initial_guess_[N - 1].uk.setZero(); // = initial_guess_[N-2].uk;

  initial_guess_[N].xk =
      integrator_.RK4(initial_guess_[N - 1].xk, initial_guess_[N - 1].uk, Ts_);
  initial_guess_[N].uk.setZero();

  unwrapInitialGuess();
}

void MPC::updateInitialGuess2(const State &x0) {
  for (int i = 1; i < N; i++) {
    initial_guess_[i - 1] = initial_guess_[i];
    if (i == 1) {
      initial_guess_[i - 1].xk = x0;
    }
    propagateOneStep(initial_guess_[i - 1].xk, initial_guess_[i - 1].uk,
                     initial_guess_[i - 1].xk);
  }

  initial_guess_[N].xk =
      integrator_.RK4(initial_guess_[N - 1].xk, initial_guess_[N - 1].uk, Ts_);
  // initial_guess_[N].uk.setZero();

  unwrapInitialGuess();
}

void MPC::propagateOneStep(const State &xk, const Input &uk, State &xk1) {
  xk1 = integrator_.RK4(xk, uk, Ts_);
}

// alternatively OptVariables MPC::unwrapInitialGuess(const OptVariables
// &initial_guess)
void MPC::unwrapInitialGuess() {
  double L = track_.getLength();
  for (int i = 1; i <= N; i++) {
    if ((initial_guess_[i].xk.phi - initial_guess_[i - 1].xk.phi) < -M_PI) {
      initial_guess_[i].xk.phi += 2. * M_PI;
    }
    if ((initial_guess_[i].xk.phi - initial_guess_[i - 1].xk.phi) > M_PI) {
      initial_guess_[i].xk.phi -= 2. * M_PI;
    }

    // if ((initial_guess_[i].xk.s - initial_guess_[i - 1].xk.s) > L / 2.) {
    //   initial_guess_[i].xk.s -= L;
    // }
    if ((initial_guess_[i].xk.s - initial_guess_[i - 1].xk.s) < -L / 2.) {
      initial_guess_[i].xk.s += L;
    }
  }
}

double MPC::unwrapProgressDifference(double s_ego, double s_oppo) {
  double L = track_.getLength();
  double sr_oppo = s_oppo - s_ego;
  if (sr_oppo > L / 2.) {
    sr_oppo -= L;
  } else if (sr_oppo < -L / 2.) {
    sr_oppo += L;
  }
  return sr_oppo;
}

void MPC::generateNewInitialGuess(const State &x0) {
  initial_guess_[0].xk = x0;
  initial_guess_[0].uk.setZero();

  for (int i = 1; i <= N; i++) {
    initial_guess_[i].xk.setZero();
    initial_guess_[i].uk.setZero();

    initial_guess_[i].xk.s =
        initial_guess_[i - 1].xk.s + Ts_ * param_.initial_velocity;
    Eigen::Vector2d track_pos_i = track_.getPostion(initial_guess_[i].xk.s);
    Eigen::Vector2d track_dpos_i = track_.getDerivative(initial_guess_[i].xk.s);
    initial_guess_[i].xk.X = track_pos_i(0);
    initial_guess_[i].xk.Y = track_pos_i(1);
    initial_guess_[i].xk.phi = atan2(track_dpos_i(1), track_dpos_i(0));
    initial_guess_[i].xk.vx = param_.initial_velocity;
    initial_guess_[i].xk.vs = param_.initial_velocity;
  }
  unwrapInitialGuess();
  valid_initial_guess_ = true;
}

// @brief sw: Generate initail guess such that vehicle is assumed to maintain
// current state using Pure Pursuit.
void MPC::generateNewInitialGuess2(const State &x0) {
  initial_guess_[0].xk = x0;

  double lookahead = 65.0; // 35 m
  double car_length = 3.0; // 3 m
  double steer_max = 0.25;

  for (int i = 1; i <= N; i++) {
    initial_guess_[i].xk.setZero();
    Eigen::Vector2d ego_pos = {initial_guess_[i - 1].xk.X,
                               initial_guess_[i - 1].xk.Y};

    Eigen::Vector2d track_pos_goal =
        track_.getPostion(initial_guess_[i - 1].xk.s + lookahead);

    double alpha =
        atan2(track_pos_goal(1) - ego_pos(1), track_pos_goal(0) - ego_pos(0)) -
        initial_guess_[i - 1].xk.phi;
    double delta = atan2(2.0 * car_length * sin(alpha) / lookahead, 1.0);

    delta = std::min(std::max(delta, -steer_max), steer_max);

    double vx = initial_guess_[i - 1].xk.vx;
    double vy = initial_guess_[i - 1].xk.vy *
                0.99; // assume vy converges to zero exponentially
    double dphi = vx / car_length * tan(delta);
    double phi = initial_guess_[i - 1].xk.phi + dphi * Ts_;
    Eigen::Vector2d vel_vector = {vx * cos(phi) - vy * sin(phi),
                                  vx * sin(phi) + vy * cos(phi)};
    Eigen::Vector2d next_ego_pos = ego_pos + Ts_ * vel_vector;

    initial_guess_[i].uk.dDelta =
        0.0; // (delta - initial_guess_[i - 1].xk.delta) / Ts_;

    initial_guess_[i].xk.s =
        initial_guess_[i - 1].xk.s + Ts_ * initial_guess_[i - 1].xk.vs;
    initial_guess_[i].xk.X = next_ego_pos(0);
    initial_guess_[i].xk.Y = next_ego_pos(1);
    initial_guess_[i].xk.phi = phi;
    initial_guess_[i].xk.vx = vx;
    initial_guess_[i].xk.vy = vy;
    initial_guess_[i].xk.vs =
        (initial_guess_[i].xk.s - initial_guess_[i - 1].xk.s) / Ts_;
  }
  unwrapInitialGuess();
  valid_initial_guess_ = true;
}

std::array<OptVariables, N + 1> MPC::sqpSolutionUpdate(
    const std::array<OptVariables, N + 1> &last_solution,
    const std::array<OptVariables, N + 1> &current_solution) {
  // TODO use line search and merit function
  std::array<OptVariables, N + 1> updated_solution;
  StateVector updated_x_vec;
  InputVector updated_u_vec;
  for (int i = 0; i <= N; i++) {
    updated_x_vec = sqp_mixing_ * (stateToVector(current_solution[i].xk) +
                                   stateToVector(last_solution[i].xk)) +
                    (1.0 - sqp_mixing_) * stateToVector(last_solution[i].xk);
    updated_u_vec = sqp_mixing_ * (inputToVector(current_solution[i].uk) +
                                   inputToVector(last_solution[i].uk)) +
                    (1.0 - sqp_mixing_) * inputToVector(last_solution[i].uk);

    updated_solution[i].xk = vectorToState(updated_x_vec);
    updated_solution[i].uk = vectorToInput(updated_u_vec);
  }

  return updated_solution;
}

MPCReturn MPC::runMPC(State &x0) {
  cost_ = cost_original_;

  auto t1 = std::chrono::high_resolution_clock::now();
  int solver_status = -1;

  double vel = sqrt(pow(x0.vx, 2.0) + pow(x0.vy, 2.0));
  // Set cost weight varying by speed
  double qC_min = 0.04;
  double qC_max = 0.06;
  double speed_min = 0.0 / 3.6;
  double speed_max = 270.0 / 3.6;
  double qC_norm = cost_.cost_param_.q_c_original;
  double qVS_origin = cost_.cost_param_.q_vs_original;

  // TODO: Check this
  bool is_track_straight = true;
  // if (is_track_straight && considering_paths_.size() != 0)
  // {
  //   cost_.cost_param_.q_c = qC_norm * 0.8;
  // }
  // else
  // {
  //   cost_.cost_param_.q_c = qC_norm * 1.1;
  //   cost_.cost_param_.q_vs = qVS_origin * 0.9;
  // }

  cost_.cost_param_.q_c = qC_norm * 1.1;
  cost_.cost_param_.q_vs = qVS_origin * 0.9;

  // Set progress weight by number of opponents and time
  double qVs_min = 0.0003;
  double qVs_max = 3;

  x0.s = track_.projectOnSpline(x0);
  x0.unwrap(track_.getLength());
  generateNewInitialGuess2(x0);

  // TODO: this is one approach to handle solver errors, works well in
  // simulation
  n_no_solves_sqp_ = 0;
  int solved_successively = 0;
  int exit_thres_successively = 4;
  State x0_normalized;
  for (int i = 0; i < n_sqp_; i++) {
    setMPCProblem();
    x0_normalized =
        vectorToState(normalization_param_.T_x_inv *
                      (stateToVector(x0) - 1.0 * stateToVector(x0)));
    optimal_solution_ =
        solver_interface_->solveMPC(stages_, x0_normalized, &solver_status);
    optimal_solution_ = deNormalizeSolution(optimal_solution_);
    if (solver_status != 0) {
      n_no_solves_sqp_++;
      solved_successively = 0;
    } else {
      if (++solved_successively > exit_thres_successively)
        break;
    }
    if (solver_status <= 1)
      initial_guess_ = sqpSolutionUpdate(initial_guess_, optimal_solution_);
    if (solver_status == 2 && i > n_reset_) {
      for (int j = 0; j <= N - 1; j++) {
        initial_guess_[j] = initial_guess_[j + 1];
      }
      break;
    }
  }
  const int max_error = std::max(n_sqp_ - 1, 1);
  if (n_no_solves_sqp_ >= max_error)
    n_non_solves_++;
  else
    n_non_solves_ = 0;

  if (n_non_solves_ >= n_reset_) {
    valid_initial_guess_ = false;
  }

  solver_status_ = solver_status;

  auto t2 = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> time_span =
      std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
  double time_nmpc = time_span.count();

  // update driven_distance for tire
  driven_dist_ += 0.01 * vel;

  return {initial_guess_[0].uk, initial_guess_, time_nmpc};
}

void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y) {
  track_.gen2DSpline(X, Y);
}

void MPC::setTrack(const nav_msgs::msg::Path &planned_path) {
  if (planned_path.poses.size() != 0) {
    Eigen::VectorXd X(planned_path.poses.size());
    Eigen::VectorXd Y(planned_path.poses.size());
    Eigen::VectorXd X_i(planned_path.poses.size());
    Eigen::VectorXd Y_i(planned_path.poses.size());
    Eigen::VectorXd X_o(planned_path.poses.size());
    Eigen::VectorXd Y_o(planned_path.poses.size());

    X(0) = planned_path.poses[0].pose.position.x;
    Y(0) = planned_path.poses[0].pose.position.y;

    double r = 2.0;
    double slope = 0.0;

    for (int i = 1; i < planned_path.poses.size(); i++) {
      X(i) = planned_path.poses[i].pose.position.x;
      Y(i) = planned_path.poses[i].pose.position.y;

      slope = atan2(Y(i) - Y(i - 1), X(i) - X(i - 1));
      X_i(i - 1) = planned_path.poses[i].pose.position.x - r * sin(slope);
      Y_i(i - 1) = planned_path.poses[i].pose.position.y + r * cos(slope);
      X_o(i - 1) = planned_path.poses[i].pose.position.x + r * sin(slope);
      Y_o(i - 1) = planned_path.poses[i].pose.position.y - r * cos(slope);
    }

    X_i(planned_path.poses.size() - 1) =
        planned_path.poses[planned_path.poses.size() - 1].pose.position.x -
        r * sin(slope + 0.000001);
    Y_i(planned_path.poses.size() - 1) =
        planned_path.poses[planned_path.poses.size() - 1].pose.position.y +
        r * cos(slope + 0.000001);
    X_o(planned_path.poses.size() - 1) =
        planned_path.poses[planned_path.poses.size() - 1].pose.position.x +
        r * sin(slope + 0.000001);
    Y_o(planned_path.poses.size() - 1) =
        planned_path.poses[planned_path.poses.size() - 1].pose.position.y -
        r * cos(slope + 0.000001);

    track_.gen2DSpline(X, Y);
    track_i.gen2DSpline(X_i, Y_i);
    track_o.gen2DSpline(X_o, Y_o);
  }
}

void MPC::setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y,
                   const Eigen::VectorXd &X_i, const Eigen::VectorXd &Y_i,
                   const Eigen::VectorXd &X_o, const Eigen::VectorXd &Y_o) {
  track_.gen2DSpline(X, Y);
  track_i.gen2DSpline(X_i, Y_i);
  track_o.gen2DSpline(X_o, Y_o);
  track_fast_line_ = track_;
  track_fast_line_.gen2DSpline(X, Y);
}

void MPC::setTrack(const Eigen::VectorXd &X_i, const Eigen::VectorXd &Y_i,
                   const Eigen::VectorXd &X_o, const Eigen::VectorXd &Y_o) {
  track_i.gen2DSpline(X_i, Y_i);
  track_o.gen2DSpline(X_o, Y_o);
}

void MPC::setInitialTrackLeftRight(const Eigen::VectorXd &X_left,
                                   const Eigen::VectorXd &Y_left,
                                   const Eigen::VectorXd &X_right,
                                   const Eigen::VectorXd &Y_right) {
  track_initial_left_.gen2DSpline(X_left, Y_left);
  track_initial_right_.gen2DSpline(X_right, Y_right);
}

void MPC::setm25m40(const Eigen::VectorXd &X_left,
                    const Eigen::VectorXd &Y_left,
                    const Eigen::VectorXd &X_right,
                    const Eigen::VectorXd &Y_right) {
  track_m25_.gen2DSpline(X_left, Y_left);
  track_m40_.gen2DSpline(X_right, Y_right);
}

void MPC::changeTrack(int initial_raceline) {
  if (initial_raceline == 0) {
    track_ = track_fast_line_;
  } else if (initial_raceline == 1) {
    track_ = track_initial_left_;
  } else if (initial_raceline == 2) {
    track_ = track_initial_right_;
  } else if (initial_raceline == 3) {
    track_ = track_m25_;
  } else if (initial_raceline == 4) {
    track_ = track_m40_;
  }
  initial_raceline_ = initial_raceline;
}

void MPC::setElapsedTime(double time_elapsed) { time_elapsed_ = time_elapsed; }

void MPC::setPredictions(const Eigen::VectorXd &X, const Eigen::VectorXd &Y,
                         const int k) {
  resetPredictions();
  // k: opponent index
  tracks_[k].path_data_.X.resize(X.size());
  tracks_[k].path_data_.Y.resize(Y.size());
  tracks_[k].path_data_.X = X;
  tracks_[k].path_data_.Y = Y;
}

void MPC::setMultiplePredictions(
    const std::vector<ArcLengthSpline> &considering_paths,
    const std::vector<Eigen::VectorXd> &considering_paths_phi) {

  considering_paths_ = considering_paths;
  considering_paths_phi_ = considering_paths_phi;

  resetPredictions(considering_paths);
  int num_oppo = considering_paths.size();
  // i: opponent index
  for (int i = 0; i < num_oppo; i++) {
    tracks_[i].path_data_.X.resize(considering_paths[i].path_data_.X.size());
    tracks_[i].path_data_.Y.resize(considering_paths[i].path_data_.Y.size());
    tracks_[i].path_data_.X = considering_paths[i].path_data_.X;
    tracks_[i].path_data_.Y = considering_paths[i].path_data_.Y;
  }
}

// Find closest front oppo
void MPC::setSinglePredictionFront(
    const State &x0_ego, const std::vector<ArcLengthSpline> &considering_paths,
    const std::vector<Eigen::VectorXd> &considering_paths_phi) {

  int num_oppo = considering_paths.size();
  double s_ego = track_.projectOnSpline(x0_ego);
  int oppo_index = -1;

  for (int i = 0; i < num_oppo; i++) {
    State x_oppo;
    x_oppo.setZero();
    x_oppo.X = considering_paths[i].path_data_.X(0);
    x_oppo.Y = considering_paths[i].path_data_.Y(0);
    x_oppo.phi = considering_paths_phi[i](0);
    double s_oppo = track_.projectOnSpline(x_oppo);

    if (s_ego < s_oppo) {
      oppo_index = i;
      break;
    }
  }

  resetPrediction(considering_paths[oppo_index]);

  considering_paths_.clear();
  considering_paths_.push_back(considering_paths[oppo_index]);
  considering_paths_phi_.clear();
  considering_paths_phi_.push_back(considering_paths_phi[oppo_index]);

  tracks_[0].path_data_.X.resize(
      considering_paths[oppo_index].path_data_.X.size());
  tracks_[0].path_data_.Y.resize(
      considering_paths[oppo_index].path_data_.Y.size());
  tracks_[0].path_data_.X = considering_paths[oppo_index].path_data_.X;
  tracks_[0].path_data_.Y = considering_paths[oppo_index].path_data_.Y;
}

void MPC::resetPredictions() {

  Eigen::VectorXd tmp_x(N);
  Eigen::VectorXd tmp_y(N);

  tmp_x.fill(0);
  tmp_y.fill(0);

  for (int k = 0; k < sizeof(tracks_) / sizeof(tracks_[0]); k++) {
    tracks_[k].path_data_.X = tmp_x;
    tracks_[k].path_data_.Y = tmp_y;
  }
}

void MPC::resetPredictions(
    const std::vector<ArcLengthSpline> &considering_paths_) {
  tracks_.resize(considering_paths_.size());
  for (int k = 0; k < considering_paths_.size(); k++) {
    Eigen::VectorXd tmp_x(considering_paths_[k].path_data_.X.size());
    Eigen::VectorXd tmp_y(considering_paths_[k].path_data_.Y.size());

    tmp_x.fill(0);
    tmp_y.fill(0);

    tracks_[k].path_data_.X = tmp_x;
    tracks_[k].path_data_.Y = tmp_y;
  }
}

void MPC::resetPrediction(const ArcLengthSpline &considering_path) {
  tracks_.resize(1);
  int k = 0;
  Eigen::VectorXd tmp_x(considering_path.path_data_.X.size());
  Eigen::VectorXd tmp_y(considering_path.path_data_.Y.size());

  tmp_x.fill(0);
  tmp_y.fill(0);

  tracks_[k].path_data_.X = tmp_x;
  tracks_[k].path_data_.Y = tmp_y;
}

void MPC::updateCenterlineYawError() {
  // Calc centerline yaw error
  centerline_yaw_error_.clear();
  for (int i = 0; i <= N - 1; i++) {
    // yaw(t) = atan2(y(t+1) - y(t), x(t+1) - x(t))
    double centerline_yaw = atan2(pos_center_[i + 1][1] - pos_center_[i][1],
                                  pos_center_[i + 1][0] - pos_center_[i][0]);
    centerline_yaw_error_.push_back(centerline_yaw - initial_guess_[i].xk.phi);
  }
  centerline_yaw_error_.push_back(
      centerline_yaw_error_[N - 1]); // Nth value == N-1th value
}

void MPC::findClosestOppo(const State &xk, const int time_step,
                          State &xk_oppo) {
  double num_oppo = considering_paths_.size();
  double dist = 100000;
  double temp = 0;
  for (int i = 0; i < num_oppo; i++) {
    temp = pow(xk.X - considering_paths_[i].path_data_.X(time_step), 2.0) +
           pow(xk.Y - considering_paths_[i].path_data_.Y(time_step), 2.0);
    if (temp < dist) {
      dist = temp;
      xk_oppo.X = (double)considering_paths_[i].path_data_.X(time_step);
      xk_oppo.Y = (double)considering_paths_[i].path_data_.Y(time_step);
      xk_oppo.phi = (double)considering_paths_phi_[i](time_step);
    }
  }
}

void MPC::getPosDiffAtOppoFrame(State &xk, State &xk_oppo, double &delta_x,
                                double &delta_y) {
  double relative_heading =
      xk_oppo.phi - xk.phi; // refer to "Drafting model for IAC.pptx"
  // std::cout << "xk_oppo.phi" << xk_oppo.phi << " xk.phi : " << xk.phi <<
  // std::endl;

  double dist = sqrt(pow(xk_oppo.X - xk.X, 2) + pow(xk_oppo.Y - xk.Y, 2));
  // double relative_heading_compensated =
  // atan2(dist*sin(relative_heading),dist*cos(relative_heading)-4.921/2.0);
  double relative_heading_compensated =
      atan2(dist * sin(relative_heading), dist * cos(relative_heading));
  // double cos_yaw = cos(relative_heading_compensated);
  // double sin_yaw = sin(relative_heading_compensated);

  // double p = xk_oppo.X - xk.X;
  // double q = xk_oppo.Y - xk.Y;

  // delta_x = -p * cos_yaw - q * sin_yaw;
  // delta_y = p * sin_yaw - q * cos_yaw;

  double dist_compensated = sqrt(pow(dist * sin(relative_heading), 2) +
                                 pow(dist * cos(relative_heading), 2));
  // delta_x = dist * cos(relative_heading_compensated-relative_heading);
  // delta_y = dist * sin(relative_heading_compensated-relative_heading);

  delta_x = (xk.X - xk_oppo.X) * cos(xk_oppo.phi) +
            (xk.Y - xk_oppo.Y) * sin(xk_oppo.phi);
  delta_y = -(xk.X - xk_oppo.X) * sin(xk_oppo.phi) +
            (xk.Y - xk_oppo.Y) * cos(xk_oppo.phi);
}

void MPC::checkIfOvertake(bool is_track_straight) {

  if (tracks_.size() == 0) {
    /////////////////////////////////////////////////
    //  CASE 1 : WITHOUT OPPONENTS
    /////////////////////////////////////////////////
    is_overtake_mode_ = true;
  } else {
    // THERE IS/ARE OPPONENTS
    double s_ego = 0;
    double s_oppo = 0;
    double s_diff_min = -1000000;
    double s_diff_btw_closest_ahead = 1000000;
    double s_diff = 0;
    bool is_AheadOPPO_exist = false;
    int ahead_closest_tracks_idx = -1;

    State x_ego_current;
    x_ego_current.setZero();
    x_ego_current = initial_guess_[0].xk;
    s_ego = track_.projectOnSpline(x_ego_current);

    for (int i = 0; i < tracks_.size(); i++) {
      State x_oppo_current;
      x_oppo_current.X = tracks_[i].path_data_.X(0);
      x_oppo_current.Y = tracks_[i].path_data_.Y(0);
      s_oppo = track_.projectOnSpline(x_oppo_current);
      double sr_oppo = unwrapProgressDifference(s_ego, s_oppo);

      s_diff = -sr_oppo;

      if (s_diff < 0) {
        //   only for ahead vehicle
        // is_AheadOPPO_exist = true;
        // break;
        is_AheadOPPO_exist += 1;
        if (s_diff > s_diff_min) {
          s_diff_min = s_diff;
          s_diff_btw_closest_ahead = abs(s_diff_min);
          ahead_closest_tracks_idx = i;
        }
      }
    }

    // update index of the ahead closest oppo
    ahead_closest_tracks_idx_ = ahead_closest_tracks_idx;

    if (is_AheadOPPO_exist == false) {
      /////////////////////////////////////////////////
      //   CASE 2 : WHEN ONLY REAR OPPONENTS
      /////////////////////////////////////////////////

      is_overtake_mode_ = true;
      return;
    }
    /////////////////////////////////////////////////
    // CASE 3 : THERE IS ONE OR MORE AHEAD OPPONENTS
    /////////////////////////////////////////////////
    // Compare terminal progress

    const double race_strategy_planner_activate_range = 7.0;
    if (s_diff_btw_closest_ahead < race_strategy_planner_activate_range) {
      /////////////////////////////////////////////////////////////////
      // CASE 3-1 : THERE IS ONE OR MORE AHEAD OPPONENTS WITHIN
      // race_strategy_planner_activate_range meter
      /////////////////////////////////////////////////////////////////

      State x_ego_terminal;
      x_ego_terminal.setZero();
      x_ego_terminal = initial_guess_[N].xk;
      s_ego = track_.projectOnSpline(x_ego_terminal);

      State x_oppo_terminal;
      x_oppo_terminal.setZero();
      // findClosestOppo(x_ego_terminal, N, x_oppo_terminal);
      x_oppo_terminal.X =
          (double)considering_paths_[ahead_closest_tracks_idx].path_data_.X(N);
      x_oppo_terminal.Y =
          (double)considering_paths_[ahead_closest_tracks_idx].path_data_.Y(N);
      x_oppo_terminal.phi =
          (double)considering_paths_phi_[ahead_closest_tracks_idx](N);
      s_oppo = track_.projectOnSpline(x_oppo_terminal);

      double sr_oppo = unwrapProgressDifference(s_ego, s_oppo);
      //   State x_oppo_current;
      //   x_oppo_current.X = tracks_[i].path_data_.X(0);
      //   x_oppo_current.Y = tracks_[i].path_data_.Y(0);
      //   s_oppo = track_.projectOnSpline(x_oppo_current);

      // TODO: check this
      s_diff = 0 - sr_oppo;
      // s_diff = s_diff_btw_closest_ahead;

      double current_throttle = x_ego_current.D;
      double threshold = 2.0;
      // double threshold_straight = 0.4; // 0.8
      // double threshold_straight = 0.8;
      double threshold_straight = 0.25;

      double throttle_upperbound = 1.0;
      double throttle_lowerbound = 0.8;

      double threshold_straight_max = 0.4;
      double threshold_straight_min = -0.0;
      double threshold_curve_max = 4.0;
      double threshold_curve_min = 2.0;

      double throttle_margin1 =
          (threshold_curve_max - threshold_curve_min) /
          (throttle_upperbound -
           throttle_lowerbound); // threshold = -1 m @ D = 0.8
      double throttle_margin2 =
          (threshold_straight_max - threshold_straight_min) /
          (throttle_upperbound -
           throttle_lowerbound); // threshold = - 3 m @ D = 0.8

      // threshold = threshold_curve_max + throttle_margin1 * (current_throttle
      // - throttle_upperbound); threshold_straight = threshold_straight_max +
      // throttle_margin2 * (current_throttle - throttle_upperbound);

      // threshold = std::min(std::max(threshold, threshold_curve_min),
      // threshold_curve_max); threshold_straight =
      // std::min(std::max(threshold_straight, threshold_straight_min),
      // threshold_straight_max);

      bool progress_criteria = false;

      // if (is_track_straight) {
      //   if(s_diff < 0) { // ego is in the rear
      //     if (s_diff < threshold_straight) {
      //       progress_criteria = true;
      //     }
      //     else {
      //       progress_criteria = false;
      //     }
      //   }
      //   else {  // ego is in the front
      //     if (s_diff > threshold_straight) {
      //         progress_criteria = true;
      //     }
      //     else {
      //         progress_criteria = false;
      //     }
      //   }
      // } else {
      //   if (s_diff > threshold) {
      //     progress_criteria = true;
      //   } else {
      //     progress_criteria = false;
      //   }
      // }

      if (is_track_straight) {
        if (s_diff < threshold_straight) {
          progress_criteria = false;
        } else {
          progress_criteria = true;
        }
      } else {
        if (s_diff > threshold) {
          progress_criteria = true;
        } else {
          progress_criteria = false;
        }
      }

      is_overtake_mode_ = progress_criteria;
    } else {
      /////////////////////////////////////////////////////////////////
      // CASE 3-2 : THERE IS ONE OR MORE AHEAD OPPONENTS FAR BEYOND
      // race_strategy_planner_activate_range meter
      /////////////////////////////////////////////////////////////////
      is_overtake_mode_ = true;
    }
  }
}

void MPC::findAheadOppo(std::vector<ArcLengthSpline> considering_paths) {
  ahead_closest_tracks_idx_ = -1;

  if (considering_paths.empty() == true) {
    /////////////////////////////////////////////////
    //  CASE 1 : WITHOUT OPPONENTS
    /////////////////////////////////////////////////
    ahead_closest_tracks_idx_ = -1;
  } else {
    // THERE IS/ARE OPPONENTS
    double s_ego = 0;
    double s_oppo = 0;
    double s_diff_min = -1000000;
    double s_diff_btw_closest_ahead = 1000000;
    double s_diff = 0;
    bool is_AheadOPPO_exist = false;

    State x_ego_current;
    x_ego_current.setZero();
    x_ego_current = initial_guess_[0].xk;
    s_ego = track_.projectOnSpline(x_ego_current);

    for (int i = 0; i < considering_paths.size(); i++) {
      State x_oppo_current;
      x_oppo_current.X = considering_paths[i].path_data_.X(0);
      x_oppo_current.Y = considering_paths[i].path_data_.Y(0);
      s_oppo = track_.projectOnSpline(x_oppo_current);
      double sr_oppo = unwrapProgressDifference(s_ego, s_oppo);

      s_diff = -sr_oppo;

      if (s_diff < 0) {
        //   only for ahead vehicle
        // is_AheadOPPO_exist = true;
        // break;
        is_AheadOPPO_exist += 1;
        if (s_diff > s_diff_min) {
          s_diff_min = s_diff;
          s_diff_btw_closest_ahead = abs(s_diff_min);
          ahead_closest_tracks_idx_ = i;
        }
      }
    }
  }
}

bool MPC::returnOvertakeMode() { return is_overtake_mode_; }

int MPC::returnSolverStatus() { return solver_status_; }
int MPC::returnSolverIter() {
  solver_iter_ = solver_interface_->getSolverIter();
  return solver_iter_;
}

} // namespace mpcc