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

#include "cost.h"
namespace mpcc {
Cost::Cost() {
  std::cout << "(cost) default constructor, not everything is initialized properly"
            << std::endl;
}

Cost::Cost(const PathToJson &path)
    : cost_param_(CostParam(path.cost_path)), param_(Param(path.param_path)) {
    std::cout << "(cost) Openning params at " << path.param_path << std::endl;
}

TrackPoint Cost::getRefPoint(const ArcLengthSpline &track,
                             const State &x) const {
  // compute all the geometry information of the track at a given arc length
  const double s = x.s;

  // X-Y postion of the reference at s
  const Eigen::Vector2d pos_ref = track.getPostion(s);
  const double x_ref = pos_ref(0);
  const double y_ref = pos_ref(1);
  // reference path derivatives
  const Eigen::Vector2d dpos_ref = track.getDerivative(s);
  const double dx_ref = dpos_ref(0);
  const double dy_ref = dpos_ref(1);
  // angle of the reference path
  const double theta_ref = atan2(dy_ref, dx_ref);
  // second order derivatives
  Eigen::Vector2d ddpos_ref = track.getSecondDerivative(s);
  const double ddx_ref = ddpos_ref(0);
  const double ddy_ref = ddpos_ref(1);
  // curvature
  double dtheta_ref_nom = (dx_ref * ddy_ref - dy_ref * ddx_ref);
  double dtheta_ref_denom = (dx_ref * dx_ref + dy_ref * dy_ref);
  // if(std::fabs(dtheta_ref_nom) < 1e-7)
  //     dtheta_ref_nom = 0;
  // if(std::fabs(dtheta_ref_denom) < 1e-7)
  //     dtheta_ref_denom = 1e-7;
  double dtheta_ref = dtheta_ref_nom / dtheta_ref_denom;

  return {x_ref, y_ref, dx_ref, dy_ref, theta_ref, dtheta_ref};
}

ErrorInfo Cost::getErrorInfo(const ArcLengthSpline &track,
                             const State &x) const {
  ErrorInfo error_info;
  // compute error between reference and X-Y position of the car
  const double X = x.X;
  const double Y = x.Y;
  const TrackPoint track_point = getRefPoint(track, x);
  // contouring  error
  Eigen::Matrix<double, 1, 2> contouring_error;
  contouring_error(0) =
      -std::sin(track_point.theta_ref) * (track_point.x_ref - X) +
      std::cos(track_point.theta_ref) * (track_point.y_ref - Y);
  // double contouring_error_max = 1.0;
  // contouring_error(0) = std::min(contouring_error_max,
  // std::max(-contouring_error_max, contouring_error(0))); lag error
  contouring_error(1) =
      std::cos(track_point.theta_ref) * (track_point.x_ref - X) +
      std::sin(track_point.theta_ref) * (track_point.y_ref - Y);
  // partial derivatives of the lag and contouring error with respect to s
  const double dContouringError =
      -track_point.dtheta_ref * std::cos(track_point.theta_ref) *
          (track_point.x_ref - X) -
      track_point.dtheta_ref * std::sin(track_point.theta_ref) *
          (track_point.y_ref - Y) -
      track_point.dx_ref * std::sin(track_point.theta_ref) +
      track_point.dy_ref * std::cos(track_point.theta_ref);
  const double dLagError =
      -track_point.dtheta_ref * std::sin(track_point.theta_ref) *
          (track_point.x_ref - X) +
      track_point.dtheta_ref * std::cos(track_point.theta_ref) *
          (track_point.y_ref - Y) +
      track_point.dx_ref * std::cos(track_point.theta_ref) +
      track_point.dy_ref * std::sin(track_point.theta_ref);

  Eigen::Matrix<double, 2, NX> d_contouring_error =
      Eigen::Matrix<double, 2, NX>::Zero();
  // compute all remaining partial derivatives and store the in dError
  d_contouring_error(0, si_index.X) = std::sin(track_point.theta_ref);
  d_contouring_error(0, si_index.Y) = -std::cos(track_point.theta_ref);
  d_contouring_error(0, si_index.s) = dContouringError;

  d_contouring_error(1, si_index.X) = -std::cos(track_point.theta_ref);
  d_contouring_error(1, si_index.Y) = -std::sin(track_point.theta_ref);
  d_contouring_error(1, si_index.s) = dLagError;

  return {contouring_error, d_contouring_error};
}

CostMatrix Cost::getBetaCost(const State &x) const {
  //    CostMatrix beta_cost;
  const double vx = x.vx;
  const double vy = x.vy;
  // jacobian of beta
  Eigen::Matrix<double, 1, NX> d_beta = Eigen::Matrix<double, 1, NX>::Zero();
  d_beta(si_index.vx) = -vy / (vx * vx + vy * vy);
  d_beta(si_index.vy) = vx / (vx * vx + vy * vy);
  // zero order term of beta approximation
  const double beta_zero = std::atan(vy / vx) - d_beta * stateToVector(x);
  // Q_beta = (qBeta*beta)^2 ~ x^T (qBeta*dBeta^T*dBeta) x +
  // (qBeta*2*BetaZero*qBeta)^ x + const
  const Q_MPC Q_beta = 2.0 * cost_param_.q_beta * d_beta.transpose() * d_beta;
  const q_MPC q_beta =
      cost_param_.q_beta * 2.0 * beta_zero * d_beta.transpose();

  return {Q_beta,        R_MPC::Zero(), S_MPC::Zero(), q_beta,
          r_MPC::Zero(), Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix Cost::getBetaKinCost(const State &x) const {
  const double rel_center = param_.lr / (param_.lf + param_.lr);
  //    CostMatrix beta_cost;
  const double vx = x.vx;
  const double vy = x.vy;

  const double delta = x.delta;
  // jacobian of beta
  Eigen::Matrix<double, 1, NX> d_beta = Eigen::Matrix<double, 1, NX>::Zero();
  d_beta(si_index.vx) = vy / (vx * vx + vy * vy);
  d_beta(si_index.vy) = -vx / (vx * vx + vy * vy);

  d_beta(si_index.delta) =
      (rel_center * (1.0 / std::cos(delta)) * (1.0 / std::cos(delta))) /
      (rel_center * rel_center * std::tan(delta) * std::tan(delta) + 1.0);
  // zero order term of beta approximation
  const double beta_zero = std::atan(std::tan(delta) * rel_center) -
                           std::atan(vy / vx) - d_beta * stateToVector(x);
  // Q_beta = (qBeta*beta)^2 ~ x^T (qBeta*dBeta^T*dBeta) x +
  // (qBeta*2*BetaZero*qBeta)^ x + const
  const Q_MPC Q_beta = 2.0 * cost_param_.q_beta * d_beta.transpose() * d_beta;
  const q_MPC q_beta =
      cost_param_.q_beta * 2.0 * beta_zero * d_beta.transpose();

  return {Q_beta,        R_MPC::Zero(), S_MPC::Zero(), q_beta,
          r_MPC::Zero(), Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix Cost::getContouringCost(const ArcLengthSpline &track, const State &x,
                                   const int k) const {
  // compute state cost, formed by contouring error cost + cost on "real" inputs
  // compute reference information
  const StateVector x_vec = stateToVector(x);
  // compute error and jacobean of error
  const ErrorInfo error_info = getErrorInfo(track, x);
  // contouring cost matrix
  Eigen::Vector2d ContouringCost;
  ContouringCost.setZero(2);
  ContouringCost(0) =
      k < N ? cost_param_.q_c : cost_param_.q_c_N_mult * cost_param_.q_c;
  ContouringCost(1) = cost_param_.q_l;
  // contouring and lag error part
  Q_MPC Q_contouring_cost = Q_MPC::Zero();
  q_MPC q_contouring_cost = q_MPC::Zero();

  Eigen::Matrix<double, 1, NX> d_contouring_error =
      Eigen::Matrix<double, 1, NX>::Zero();
  d_contouring_error = error_info.d_error.row(0);
  const double contouring_error_zero =
      error_info.error(0) - d_contouring_error * stateToVector(x);

  Eigen::Matrix<double, 1, NX> d_lag_error =
      Eigen::Matrix<double, 1, NX>::Zero();
  d_lag_error = error_info.d_error.row(1);
  const double lag_error_zero =
      error_info.error(1) - d_lag_error * stateToVector(x);

  Q_contouring_cost =
      ContouringCost(0) * d_contouring_error.transpose() * d_contouring_error +
      ContouringCost(1) * d_lag_error.transpose() * d_lag_error;
  // regularization cost on yaw rate
  Q_contouring_cost(si_index.r, si_index.r) =
      k < N ? cost_param_.q_r : cost_param_.q_r_N_mult * cost_param_.q_r;
  Q_contouring_cost = 2.0 * Q_contouring_cost;

  q_contouring_cost =
      ContouringCost(0) * 2.0 * contouring_error_zero *
          d_contouring_error.transpose() +
      ContouringCost(1) * 2.0 * lag_error_zero * d_lag_error.transpose();
  // progress maximization part
  q_contouring_cost(si_index.vs) = -cost_param_.q_vs;

  // solver interface expects 0.5 x^T Q x + q^T x
  return {Q_contouring_cost, R_MPC::Zero(), S_MPC::Zero(), q_contouring_cost,
          r_MPC::Zero(),     Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix Cost::getHeadingCost(const ArcLengthSpline &track, const State &x,
                                int k) const {
  // get heading of the track
  const Eigen::Vector2d dpos_ref = track.getDerivative(x.s);
  const double dx_ref = dpos_ref(0);
  const double dy_ref = dpos_ref(1);
  // angle of the reference path
  double theta_ref = atan2(dy_ref, dx_ref);
  theta_ref += 2.0 * M_PI * std::round((x.phi - theta_ref) / (2.0 * M_PI));

  // if(std::fabs(x.phi - theta_ref)>= 1.5){
  //     std::cout <<
  //     "kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk"
  //     << std::endl;
  // }

  Q_MPC Q_heading_cost = Q_MPC::Zero();
  // Q_heading_cost(si_index.phi,si_index.phi) = 2.0*cost_param_.q_mu;
  Q_heading_cost(si_index.phi, si_index.phi) =
      k < N ? 2.0 * cost_param_.q_mu
            : cost_param_.q_c_N_mult * 2.0 * cost_param_.q_mu;
  q_MPC q_heading_cost = q_MPC::Zero();
  // q_heading_cost(si_index.phi) = -2.0*cost_param_.q_mu*theta_ref;
  q_heading_cost(si_index.phi) =
      k < N ? -2.0 * cost_param_.q_mu
            : -cost_param_.q_c_N_mult * 2.0 * cost_param_.q_mu * theta_ref;

  return {Q_heading_cost, R_MPC::Zero(), S_MPC::Zero(), q_heading_cost,
          r_MPC::Zero(),  Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix
Cost::getCollisionCost(const std::vector<ArcLengthSpline> &tracks,
                       const std::vector<Eigen::VectorXd> considering_paths_phi,
                       const State &x, int k) const {
  CostMatrix cost_matrix;
  Q_MPC Q_contouring_cost = Q_MPC::Zero();
  q_MPC q_contouring_cost = q_MPC::Zero();

  int num_of_opponents = tracks.size();

  // std::cout << "k : " << k << " num of oppo : " <<num_of_opponents<<
  // std::endl;
  if (num_of_opponents > 0) {
    // int num_of_opponents = 1;// sizeof(tracks)/sizeof(tracks[0]);
    // std::cout << tracks.size() << " " << tracks[0].path_data_.X.size() <<
    // std::endl; std::cout << "Calculating collision cost with " <<
    // num_of_opponents << " opponents." << std::endl;
    for (int i = 0; i < num_of_opponents; i++) {
      if (k < tracks[i].path_data_.X.size()) {
        cost_matrix =
            getMinusContouringCost(tracks[i], considering_paths_phi[i], x, k);
        Q_contouring_cost += cost_matrix.Q;
        // Q_contouring_cost = Q_contouring_cost*(i-1) + cost_matrix.Q*i;
        q_contouring_cost += cost_matrix.q;
        // q_contouring_cost = q_contouring_cost*(i-1) + cost_matrix.q*i;
      }
    }
  }
  // solver interface expects 0.5 x^T Q x + q^T x
  return {Q_contouring_cost, R_MPC::Zero(), S_MPC::Zero(), q_contouring_cost,
          r_MPC::Zero(),     Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix
Cost::getMinusContouringCost(const ArcLengthSpline &track,
                             const Eigen::VectorXd considering_path_phi,
                             const State &x, const int k) const {
  // compute state cost, get away from opponents' trajectories

  // compute reference information
  const StateVector x_vec = stateToVector(x);
  // compute error and jacobean of error
  const ErrorInfo error_info =
      calcPotentialField(track, considering_path_phi, x, k);
  // const ErrorInfo error_info = calcPredDist(track,x,k);
  // contouring cost matrix
  Eigen::Vector2d ContouringCost;
  ContouringCost.setZero(2);
  // double collision_cost = 2000.;
  double collision_cost = 100.;
  // double discount = 1.;
  // double discount = 0.978;
  // double discount = 0.95;
  // double discount = 0.99;
  double discount = 0.92;
//   ContouringCost(0) = k < 10 ? 10 * collision_cost * pow(discount, (double)k)
//                              : collision_cost * pow(discount, (double)k);
  ContouringCost(0) = collision_cost * pow(discount, (double)k);
  // ContouringCost(0) = k < (int)((double)N/2.) ?
  // collision_cost*pow(discount,(double)k) : 0.0; if(k < 3)
  // {
  //     ContouringCost(0) = collision_cost*pow(discount,(double)k);
  // }
  // else
  // {
  //     ContouringCost(0) = 0.0;
  // }
  // ContouringCost(0) = error_info.error(0) < 25 ? -1000. : 0.;
  // contouring and lag error part
  Q_MPC Q_contouring_cost = Q_MPC::Zero();
  q_MPC q_contouring_cost = q_MPC::Zero();

  Eigen::Matrix<double, 1, NX> d_contouring_error =
      Eigen::Matrix<double, 1, NX>::Zero();
  d_contouring_error = error_info.d_error.row(0);

  const double contouring_error_zero =
      error_info.error(0) - d_contouring_error * stateToVector(x);
  Q_contouring_cost =
      ContouringCost(0) * d_contouring_error.transpose() * d_contouring_error;
  q_contouring_cost = ContouringCost(0) * 2.0 * contouring_error_zero *
                      d_contouring_error.transpose();

  // solver interface expects 0.5 x^T Q x + q^T x
  return {Q_contouring_cost, R_MPC::Zero(), S_MPC::Zero(), q_contouring_cost,
          r_MPC::Zero(),     Z_MPC::Zero(), z_MPC::Zero()};
}

ErrorInfo Cost::calcPredDist(const ArcLengthSpline &track, const State &x,
                             const int k) const {
  Eigen::Matrix<double, 1, 2> distance_to_collision =
      Eigen::Matrix<double, 1, 2>::Zero();
  Eigen::Matrix<double, 2, NX> d_distance_to_collision =
      Eigen::Matrix<double, 2, NX>::Zero();

  int kk = k;

  if (k >= N)
    kk = N - 1;

  double dt = Ts_;
  double x_op = track.path_data_.X(kk);
  double y_op = track.path_data_.Y(kk);
  double x_op_1 = track.path_data_.X(kk + 1);
  double y_op_1 = track.path_data_.Y(kk + 1);
  // Assume circle
  distance_to_collision(0) = pow(x_op - x.X, 2.0) + pow(y_op - x.Y, 2.0);
  distance_to_collision(1) = 0.;

  double dx_op_ds = (x_op - x_op_1) / dt * 1 / x.vs;
  double dy_op_ds = (y_op - y_op_1) / dt * 1 / x.vs;
  // compute all remaining partial derivatives and store the in dError
  d_distance_to_collision(0, si_index.X) = 2 * (x.X - x_op);
  d_distance_to_collision(0, si_index.Y) = 2 * (x.Y - y_op);
  d_distance_to_collision(0, si_index.s) =
      2 * (x.X - x_op) * dx_op_ds + 2 * (x.Y - y_op) * dy_op_ds;

  d_distance_to_collision(1, si_index.X) = 0.;
  d_distance_to_collision(1, si_index.Y) = 0.;
  d_distance_to_collision(1, si_index.s) = 0.;

  return {distance_to_collision, d_distance_to_collision};
}

ErrorInfo Cost::calcPotentialField(const ArcLengthSpline &track,
                                   const Eigen::VectorXd &considering_path_phi,
                                   const State &x, const int k) const {
  /*
      f(d) = exp(-p*d^2) where d = (x_op-x)^2 + (y_op-y)^2
      df/dd = exp(-p*d^2)*(-2*p*d)
      dd/dx = 2*(x-x_op)
      dd/ds = 2*(x_op-x)*dx_op/ds + 2*(y_op-y)*dy_op/ds
      dx_op/ds = dx_op/dt*dt/ds = dot(x_op)/vs
      Thus df/dx = df/dd * dd/dx ...
  */
  Eigen::Matrix<double, 1, 2> potential_field_value =
      Eigen::Matrix<double, 1, 2>::Zero();
  Eigen::Matrix<double, 2, NX> d_potential_field_value =
      Eigen::Matrix<double, 2, NX>::Zero();

  int kk = k;
  if (k >= N)
    kk = N - 1;

  double dt = Ts_;
  double car_length = 5.5 + 0.5;
  double lf = param_.lf;
  double lr = param_.lr;
  double car_width = 1.8;
  // double collision_param = 2.0;
  // double collision_param = 1.85;
  double collision_param = 1.57;
  // double collision_param = 1.2;
  // double collision_param = 0.9;

  double phi_op = considering_path_phi(k);
  double x_op = track.path_data_.X(kk);
  double y_op = track.path_data_.Y(kk);
  double x_op_1 = track.path_data_.X(kk + 1);
  double y_op_1 = track.path_data_.Y(kk + 1);
  // Assume two circles
  double x_op_shift = 0;
  double y_op_shift = 0;
  double x_op_shift_1 = 0;
  double y_op_shift_1 = 0;

  // find closest collision point in ego
  double ego_front_axle_x = x.X + cos(x.phi) * lf;
  double ego_front_axle_y = x.Y + sin(x.phi) * lf;
  double ego_center_x = x.X;
  double ego_center_y = x.Y;
  double ego_rear_axle_x = x.X + cos(x.phi) * (-lr);
  double ego_rear_axle_y = x.Y + sin(x.phi) * (-lr);

  double min_dist = INF;
  double ego_colli_x;
  double ego_colli_y;
  std::vector<double> ego_x_list = {ego_front_axle_x, ego_center_x,
                                    ego_rear_axle_x};
  std::vector<double> ego_y_list = {ego_front_axle_y, ego_center_y,
                                    ego_rear_axle_y};
  // std::vector<double> ego_x_list = {ego_front_axle_x, ego_rear_axle_x};
  // std::vector<double> ego_y_list = {ego_front_axle_y, ego_rear_axle_y};

  for (int i = 0; i < ego_x_list.size(); i++) {
    double dist = pow(ego_x_list[i] - x_op, 2.) + pow(ego_y_list[i] - y_op, 2.);
    if (min_dist > dist) {
      ego_colli_x = ego_x_list[i];
      ego_colli_y = ego_y_list[i];
      min_dist = dist;
    }
  }

  // std::vector<double> dist_ego_op_shift(2);
  double dist_ego_op_min = INF;
  double op_colli_x;
  double op_colli_y;
  double op_colli_x_1;
  double op_colli_y_1;

  double ego_speed = sqrt(pow(x.vx, 2.0)+pow(x.vy, 2.0));
  double oppo_speed = sqrt(pow(x_op_1-x_op, 2.0)+pow(y_op_1-y_op, 2.0)) / dt;
  double relative_speed = -oppo_speed + ego_speed; // roughly
  // relative_speed < 0 : faster than ego
  // relative_speed > 0 : slower than ego
  
  for (int i = 0; i < 3; i++) {
    if (i == 0) {
      // vehicle center
      x_op_shift = cos(phi_op) * car_length / 2.0 + x_op;
      y_op_shift = sin(phi_op) * car_length / 2.0 + y_op;
      x_op_shift_1 = cos(phi_op) * car_length / 2.0 + x_op_1;
      y_op_shift_1 = sin(phi_op) * car_length / 2.0 + y_op_1;
    } else if (i == 1) {
      // vehicle REAR related with relative velocity
      // double shift_gain_rear_max = 2.0;
      double shift_gain_rear_max = 2.5;
      double shift_gain_rear_min = 1.0;
      // double relative_speed_range_kph = 10.0;
      double relative_speed_range_kph = 20.0;
      double shift_gain_rear = 1.0 + (shift_gain_rear_max - shift_gain_rear_min) / (relative_speed_range_kph / 3.6) * relative_speed;
      // TODO: do something
      shift_gain_rear = std::max(shift_gain_rear_min, std::min(shift_gain_rear, shift_gain_rear_max));
      // x_op_shift = cos(phi_op) * (-1.4 * shift_gain_rear) + x_op;
      // y_op_shift = sin(phi_op) * (-1.4 * shift_gain_rear) + y_op;
      // x_op_shift_1 = cos(phi_op) * (-1.4 * shift_gain_rear) + x_op_1;
      // y_op_shift_1 = sin(phi_op) * (-1.4 * shift_gain_rear) + y_op_1;

      if(relative_speed > 0) {
        shift_gain_rear = 1.0 + (relative_speed * 0.3);
      }
      else{
        shift_gain_rear = 1.0;
      }
      x_op_shift = cos(phi_op) * (-1.4 * shift_gain_rear) + x_op;
      y_op_shift = sin(phi_op) * (-1.4 * shift_gain_rear) + y_op;
      x_op_shift_1 = cos(phi_op) * (-1.4 * shift_gain_rear) + x_op_1;
      y_op_shift_1 = sin(phi_op) * (-1.4 * shift_gain_rear) + y_op_1;
    } 
    else if (i == 2) {
      // vehicle FRONT 
      // change log : 0.75m move forward for avoiding rear vehicle (by cy)
      x_op_shift = cos(phi_op) * (car_length + 0.95) + x_op;
      y_op_shift = sin(phi_op) * (car_length + 0.95) + y_op;
      x_op_shift_1 = cos(phi_op) * (car_length + 0.95) + x_op_1;
      y_op_shift_1 = sin(phi_op) * (car_length + 0.95) + y_op_1;
    } 

    double dist_ego_op_shift =
        pow(x_op_shift - x_op, 2.0) + pow(y_op_shift - y_op, 2.0);
    if (dist_ego_op_shift < dist_ego_op_min) {
      op_colli_x = x_op_shift;
      op_colli_y = y_op_shift;
      op_colli_x_1 = x_op_shift_1;
      op_colli_y_1 = y_op_shift_1;
      dist_ego_op_min = dist_ego_op_shift;
    }
  }

  // update xy_op_shift, xy_op_shift_1
  x_op_shift = op_colli_x;
  y_op_shift = op_colli_y;
  x_op_shift_1 = op_colli_x_1;
  y_op_shift_1 = op_colli_y_1;

  double distance =
      pow(x_op_shift - ego_colli_x, 2.0) + pow(y_op_shift - ego_colli_y, 2.0);
  // double distance = pow(x_op_shift-x.X,2.0)+pow(y_op_shift-x.Y,2.0);
  double p = 1. / pow(collision_param, 4.); // p = 1/(2*sig^2)
  potential_field_value(0) = exp(-p * distance * distance);

  // std::cout << "k : " << k << " distance : " << pow(distance, 0.5) << "
  // exp(-p*distance*distance) : " << exp(-p*distance*distance) << std::endl;

  double df_dd = exp(-p * distance * distance) * (-2 * p * distance);
  // double dd_dx = 2*(x.X-x_op_shift);
  // double dd_dy = 2*(x.Y-y_op_shift);
  double dd_dx = 2 * (ego_colli_x - x_op_shift);
  double dd_dy = 2 * (ego_colli_y - y_op_shift);
  double dx_op_ds = (x_op_shift_1 - x_op_shift) / dt * 1. / std::max(x.vs, 1.);
  double dy_op_ds = (y_op_shift_1 - y_op_shift) / dt * 1. / std::max(x.vs, 1.);
  double dd_ds = 2 * (x_op_shift - ego_colli_x) * dx_op_ds +
                 2 * (y_op_shift - ego_colli_y) * dy_op_ds;
  // double dd_ds = 2*(x_op_shift-x.X)*dx_op_ds + 2*(y_op_shift-x.Y)*dy_op_ds;
  // compute all remaining partial derivatives and store the in dError
  d_potential_field_value(0, si_index.X) = df_dd * dd_dx;
  d_potential_field_value(0, si_index.Y) = df_dd * dd_dy;
  d_potential_field_value(0, si_index.s) = df_dd * dd_ds;

  return {potential_field_value, d_potential_field_value};
}

ErrorInfo
Cost::calcProbabilityDensity(const ArcLengthSpline &track,
                             const Eigen::VectorXd considering_path_phi,
                             const State &x, const int k) const {
  /*
      PDF of N(x; u,Sig) is
          f(x;u,Sig) = 1/sqrt(2*pi*sig_x*sig_y) * exp(-1/2 *
     [x-u]^T*(Rot(yaw_oppo)*Sig_0)^-1*[x-u])

      where
          x = [x_ego, y_ego]^T
          u = [x_op,  y_op]^T
          Sig_0 = [sig_x^2, 0;
                  0, sig_y^2]

      We want
          df/dx_ego, df/dy_ego, df/ds

      Let
          x_hat = x-u
          Sig_i = (Rot(yaw_oppo)*Sig_0)^-1 = [Sig_i_11, Sig_i_12;
                                              Sig_i_21, Sig_i_22]
          A     = 1/sqrt(2*pi*sig_x*sig_y)

      Then
          f(x;u,Sig) = A * exp(-1/2 * x_hat^T * Sig_i * x_hat)

          df/dx = A * exp(-1/2 * x_hat^T * Sig_i * x_hat) * (-1/2 * (Sig_i +
     Sig_i^T) * x_hat) which is df/dx_ego = A * exp(-1/2 * x_hat^T * Sig_i *
     x_hat) * (-1/2 * (Sig_i_11+Sig_i_11)*x_hat_1 +
     (Sig_i_12+Sig_i_21)*x_hat_2)) df/dy_ego = A * exp(-1/2 * x_hat^T * Sig_i *
     x_hat) * (-1/2 * (Sig_i_12+Sig_i_21)*x_hat_1 +
     (Sig_i_22+Sig_i_22)*x_hat_2))

          df/ds = df/dx*dx/ds = df/dx*(-du/ds) ds/dt=v=sqrt(vx^2+vy^2)

      df/dd =
      dd/dx =
      dd/ds =
      dx_op/ds = dx_op/dt*dt/ds = dot(x_op)/vs
      Thus df/dx = df/dd * dd/dx ...
  */
  Eigen::Matrix<double, 1, 2> potential_field_value =
      Eigen::Matrix<double, 1, 2>::Zero();
  Eigen::Matrix<double, 2, NX> d_potential_field_value =
      Eigen::Matrix<double, 2, NX>::Zero();

  int kk = k;

  if (k >= N)
    kk = N - 1;

  double dt = Ts_;
  double x_op = track.path_data_.X(kk);
  double y_op = track.path_data_.Y(kk);
  double x_op_1 = track.path_data_.X(kk + 1);
  double y_op_1 = track.path_data_.Y(kk + 1);
  // Assume circle
  double distance = pow(x_op - x.X, 2.0) + pow(y_op - x.Y, 2.0);
  double collision_distance = 0.9;
  double p = 1. / pow(collision_distance, 4.);
  potential_field_value(0) = exp(-p * distance * distance);

  double df_dd = exp(-p * distance * distance) * (-2 * p * distance);
  double dd_dx = 2 * (x.X - x_op);
  double dd_dy = 2 * (x.Y - y_op);
  double dx_op_ds = (x_op_1 - x_op) / dt * 1. / std::max(x.vs, 1.);
  double dy_op_ds = (y_op_1 - y_op) / dt * 1. / std::max(x.vs, 1.);
  double dd_ds = 2 * (x_op - x.X) * dx_op_ds + 2 * (y_op - x.Y) * dy_op_ds;
  // compute all remaining partial derivatives and store the in dError
  d_potential_field_value(0, si_index.X) = df_dd * dd_dx;
  d_potential_field_value(0, si_index.Y) = df_dd * dd_dy;
  d_potential_field_value(0, si_index.s) = df_dd * dd_ds;

  return {potential_field_value, d_potential_field_value};
}

CostMatrix Cost::getInputCost() const {
  // input cost and rate of chagen of real inputs
  Q_MPC Q_input_cost = Q_MPC::Zero();
  R_MPC R_input_cost = R_MPC::Zero();
  // cost of "real" inputs
  Q_input_cost(si_index.D, si_index.D) = cost_param_.r_D;
  Q_input_cost(si_index.delta, si_index.delta) = cost_param_.r_delta;
  Q_input_cost(si_index.vs, si_index.vs) = cost_param_.r_vs;
  // quadratic part
  R_input_cost(si_index.dD, si_index.dD) = cost_param_.r_dD;
  R_input_cost(si_index.dDelta, si_index.dDelta) = cost_param_.r_dDelta;
  R_input_cost(si_index.dVs, si_index.dVs) = cost_param_.r_dVs;
  // solver interface expects 0.5 u^T R u + r^T u
  Q_input_cost = 2.0 * Q_input_cost;
  R_input_cost = 2.0 * R_input_cost;

  return {Q_input_cost,  R_input_cost,  S_MPC::Zero(), q_MPC::Zero(),
          r_MPC::Zero(), Z_MPC::Zero(), z_MPC::Zero()};
}

CostMatrix Cost::getSoftConstraintCost() const {
  // input cost and rate of chagen of real inputs
  Z_MPC Z_cost = Z_MPC::Identity();
  z_MPC z_cost = z_MPC::Ones();
  // cost of "real" inputs

  Z_cost(si_index.con_track, si_index.con_track) = cost_param_.sc_quad_track;
  Z_cost(si_index.con_tire, si_index.con_tire) = cost_param_.sc_quad_tire;
  Z_cost(si_index.con_alpha, si_index.con_alpha) = cost_param_.sc_quad_alpha;

  z_cost(si_index.con_track) = cost_param_.sc_lin_track;
  z_cost(si_index.con_tire) = cost_param_.sc_lin_tire;
  z_cost(si_index.con_alpha) = cost_param_.sc_lin_alpha;

  return {Q_MPC::Zero(), R_MPC::Zero(), S_MPC::Zero(), q_MPC::Zero(),
          r_MPC::Zero(), Z_cost,        z_cost};
}

CostMatrix Cost::getCost(const ArcLengthSpline &track, const State &x,
                         const Input &u, const int k) const {
  // generate quadratic cost function
  const CostMatrix contouring_cost = getContouringCost(track, x, k);
  const CostMatrix heading_cost = getHeadingCost(track, x, k);
  const CostMatrix input_cost = getInputCost();
  CostMatrix beta_cost;
  if (cost_param_.beta_kin_cost == 1)
    beta_cost = getBetaKinCost(x);
  else
    beta_cost = getBetaCost(x);
  const CostMatrix soft_con_cost = getSoftConstraintCost();

  Q_MPC Q_not_sym =
      contouring_cost.Q + heading_cost.Q + input_cost.Q + beta_cost.Q;
  Q_MPC Q_reg = 1e-9 * Q_MPC::Identity();

  Q_MPC Q_full = 0.5 * (Q_not_sym.transpose() + Q_not_sym);
  R_MPC R_full =
      contouring_cost.R + heading_cost.R + input_cost.R + beta_cost.R;
  q_MPC q_full =
      contouring_cost.q + heading_cost.q + input_cost.q + beta_cost.q;
  r_MPC r_full =
      contouring_cost.r + heading_cost.r + input_cost.r + beta_cost.r;

  // TODO do this properly directly in the differnet functions computing the
  // cost
  // double discount = 0.97; // sw: cost discount factor
  const Q_MPC Q = Q_full; // * pow(discount, (double)k)
  const R_MPC R = R_full; // * pow(discount, (double)k)
  const q_MPC q = q_full + (stateToVector(x).adjoint() * Q_full)
                               .adjoint(); // * pow(discount, (double)k)
  const r_MPC r = r_full + (inputToVector(u).adjoint() * R_full)
                               .adjoint(); // * pow(discount, (double)k)
  const Z_MPC Z = 2.0 * soft_con_cost.Z;   // * pow(discount, (double)k)
  const z_MPC z = soft_con_cost.z;         // * pow(discount, (double)k)

  return {Q, R, S_MPC::Zero(), q, r, Z, z};
}

// Pass through predicted trajectories of opponents.
CostMatrix
Cost::getCost(const int num_of_opponents,
              const std::vector<ArcLengthSpline> &tracks,
              const std::vector<Eigen::VectorXd> considering_paths_phi,
              const ArcLengthSpline &track, const State &x, const Input &u,
              const int k) const {
  // generate quadratic cost function

  const CostMatrix contouring_cost = getContouringCost(track, x, k);
  const CostMatrix heading_cost = getHeadingCost(track, x, k);
  const CostMatrix input_cost = getInputCost();

  // const CostMatrix speed_cost = getSpeedCost(x);

  const CostMatrix collision_cost =
      getCollisionCost(tracks, considering_paths_phi, x, k);
  CostMatrix beta_cost;
  if (cost_param_.beta_kin_cost == 1)
    beta_cost = getBetaKinCost(x);
  else
    beta_cost = getBetaCost(x);
  const CostMatrix soft_con_cost = getSoftConstraintCost();

  Q_MPC Q_not_sym = contouring_cost.Q + heading_cost.Q + input_cost.Q +
                    beta_cost.Q + collision_cost.Q;
  Q_MPC Q_reg = 1e-9 * Q_MPC::Identity();

  Q_MPC Q_full = 0.5 * (Q_not_sym.transpose() + Q_not_sym);
  R_MPC R_full = contouring_cost.R + heading_cost.R + input_cost.R +
                 beta_cost.R + collision_cost.R;
  q_MPC q_full = contouring_cost.q + heading_cost.q + input_cost.q +
                 beta_cost.q + collision_cost.q;
  r_MPC r_full = contouring_cost.r + heading_cost.r + input_cost.r +
                 beta_cost.r + collision_cost.r;

  // if(abs(collision_cost.q(0)) > 0.0001)
  // {
  //     std::cout << "time step:        "<< k << std::endl;
  //     std::cout << "Q not sym:        "<< Q_not_sym << std::endl;
  //     std::cout << "collision_cost.Q: "<< collision_cost.Q << std::endl;

  //     std::cout << "q_full:           "<< q_full << std::endl;
  //     std::cout << "collision_cost.q: "<< collision_cost.q << std::endl;
  // }

  // TODO do this properly directly in the differnet functions computing the
  // cost
  // double discount = 0.97; // sw: cost discount factor
  const Q_MPC Q = Q_full; // * pow(discount, (double)k)
  const R_MPC R = R_full; // * pow(discount, (double)k)
  const q_MPC q = q_full + (stateToVector(x).adjoint() * Q_full)
                               .adjoint(); // * pow(discount, (double)k)
  const r_MPC r = r_full + (inputToVector(u).adjoint() * R_full)
                               .adjoint(); // * pow(discount, (double)k)
  const Z_MPC Z = 2.0 * soft_con_cost.Z;   // * pow(discount, (double)k)
  const z_MPC z = soft_con_cost.z;         // * pow(discount, (double)k)

  return {Q, R, S_MPC::Zero(), q, r, Z, z};
}
} // namespace mpcc