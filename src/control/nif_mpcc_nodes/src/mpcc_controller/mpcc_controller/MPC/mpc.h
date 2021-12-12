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

#ifndef MPCC_MPC_H
#define MPCC_MPC_H

#include "Constraints/bounds.h"
#include "Constraints/constraints.h"
#include "Cost/cost.h"
#include "Model/integrator.h"
#include "Model/model.h"
#include "Params/params.h"
#include "Spline/arc_length_spline.h"
#include "config.h"
#include "types.h"

#include "Interfaces/hpipm_interface.h"
#include "Interfaces/solver_interface.h"

#include "iac_aero_manager/c_aero_manager.h"
#include "iac_model_manager/c_model_manager.h"
#include "iac_terrain_manager/c_terrain_manager.h"

#include "nav_msgs/msg/path.hpp"

#include <array>
#include <chrono>
#include <ctime>
#include <memory>
#include <ratio>

namespace mpcc
{

  struct OptVariables
  {
    State xk;
    Input uk;
  };

  struct Stage
  {
    LinModelMatrix lin_model;
    CostMatrix cost_mat;
    ConstrainsMatrix constrains_mat;

    Bounds_x u_bounds_x;
    Bounds_x l_bounds_x;

    Bounds_u u_bounds_u;
    Bounds_u l_bounds_u;

    Bounds_s u_bounds_s;
    Bounds_s l_bounds_s;

    // nx    -> number of states
    // nu    -> number of inputs
    // nbx   -> number of bounds on x
    // nbu   -> number of bounds on u
    // ng    -> number of polytopic constratins
    // ns   -> number of soft constraints
    int nx, nu, nbx, nbu, ng, ns;
  };

  struct MPCReturn
  {
    const Input u0;
    const std::array<OptVariables, N + 1> mpc_horizon;
    const double time_total;
  };

  class MPC
  {
  public:
    MPCReturn runMPC(State &x0);

    void setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y);

    void setTrack(const nav_msgs::msg::Path &planned_path);

    void setTrack(const Eigen::VectorXd &X, const Eigen::VectorXd &Y,
                  const Eigen::VectorXd &X_inner, const Eigen::VectorXd &Y_inner,
                  const Eigen::VectorXd &X_outer, const Eigen::VectorXd &Y_outer);
    void setTrack(const Eigen::VectorXd &X_inner, const Eigen::VectorXd &Y_inner,
                  const Eigen::VectorXd &X_outer, const Eigen::VectorXd &Y_outer);
    void setInitialTrackLeftRight(const Eigen::VectorXd &X_left,
                                  const Eigen::VectorXd &Y_left,
                                  const Eigen::VectorXd &X_right,
                                  const Eigen::VectorXd &Y_right);

    void setm25m40(const Eigen::VectorXd &X_left, const Eigen::VectorXd &Y_left,
                   const Eigen::VectorXd &X_right,
                   const Eigen::VectorXd &Y_right);

    void changeTrack(int initial_raceline);
    void setElapsedTime(double time_elapsed);

    void setOvertakeCost(const PathToJson &path);
    void setSecondCost(const PathToJson &path);
    void setOppoMode(bool is_oppo_mpc);
    void setPredictions(const Eigen::VectorXd &X, const Eigen::VectorXd &Y,
                        const int k);
    void setMultiplePredictions(
        const std::vector<ArcLengthSpline> &considering_paths,
        const std::vector<Eigen::VectorXd> &considering_paths_phi);
    void setSinglePredictionFront(
        const State &x0_ego,
        const std::vector<ArcLengthSpline> &considering_paths,
        const std::vector<Eigen::VectorXd> &considering_paths_phi);

    void resetPredictions();
    void resetPredictions(const std::vector<ArcLengthSpline> &considering_paths);
    void resetPrediction(const ArcLengthSpline &considering_path);
    void checkIfOvertake(bool is_track_straight);
    bool returnOvertakeMode();
    int returnSolverStatus();
    int returnSolverIter();

    MPC();
    MPC(int n_sqp, int n_reset, double sqp_mixing, double Ts,
        const PathToJson &path);
    Constraints constraints_;
    std::vector<std::vector<double>> pos_outer_;
    std::vector<std::vector<double>> pos_inner_;

    // for calc centerline yaw error
    std::vector<std::vector<double>> pos_center_;
    std::vector<double> centerline_yaw_error_;
    void updateCenterlineYawError();

    // Terrain
    c_terrain_manager *terrain_manager_;

    // Model param
    c_model_manager *model_manager_;

    // Aero param
    c_aero_manager *aero_manager_;
    void getPosDiffAtOppoFrame(State &xk, State &xk_oppo, double &delta_x,
                               double &delta_y);
    void findClosestOppo(const State &xk, const int time_step, State &xk_oppo);

    void findAheadOppo(std::vector<ArcLengthSpline> considering_paths);

    std::vector<ArcLengthSpline> considering_paths_;
    std::vector<Eigen::VectorXd> considering_paths_phi_;
    std::vector<ArcLengthSpline> considering_paths_ahead_;
    std::vector<Eigen::VectorXd> considering_paths_phi_ahead_;

    double driven_dist_; // for checking driven distance

    // Opponents
    std::vector<ArcLengthSpline> tracks_;
    int num_of_opponents_;

    int overtake_mode_hold_max_ = 3;
    int overtake_mode_hold_counter_ = overtake_mode_hold_max_ + 1;

    // for finding front closest oppo
    int ahead_closest_tracks_idx_ = -1;

    // for namespace
    std::string ns_;
    ArcLengthSpline track_;

  private:
    bool valid_initial_guess_;

    std::array<Stage, N + 1> stages_;

    std::array<OptVariables, N + 1> initial_guess_;
    std::array<OptVariables, N + 1> optimal_solution_;

    void setMPCProblem();

    void setStage(State &xk, const Input &uk, State &xk1, int time_step);

    CostMatrix normalizeCost(const CostMatrix &cost_mat);
    LinModelMatrix normalizeDynamics(const LinModelMatrix &lin_model);
    ConstrainsMatrix normalizeCon(const ConstrainsMatrix &con_mat);
    std::array<OptVariables, N + 1>
    deNormalizeSolution(const std::array<OptVariables, N + 1> &solution);

    void updateInitialGuess(const State &x0);
    void updateInitialGuess2(const State &x0);
    void propagateOneStep(const State &xk, const Input &uk, State &xk1);

    void generateNewInitialGuess(const State &x0);
    void generateNewInitialGuess2(const State &x0);

    void unwrapInitialGuess();
    double unwrapProgressDifference(double s_ego, double s_oppo);

    std::array<OptVariables, N + 1>
    sqpSolutionUpdate(const std::array<OptVariables, N + 1> &last_solution,
                      const std::array<OptVariables, N + 1> &current_solution);

    int n_sqp_;
    double sqp_mixing_;
    int n_non_solves_;
    int n_no_solves_sqp_;
    int n_reset_;
    int solver_status_;
    int solver_iter_;

    double Ts_ = 0.01;

    double Cr2_default_;

    bool is_oppo_mpc_;

    bool is_overtake_mode_;
    bool is_overtake_mode_counted_ = false;
    double time_elapsed_;

    Model model_;
    Integrator integrator_;
    Cost cost_;
    Cost cost_original_;
    Cost cost_overtake_;
    Cost cost_follow_;
    ArcLengthSpline track_fast_line_;

    ArcLengthSpline track_i;
    ArcLengthSpline track_o;
    ArcLengthSpline track_initial_left_;
    ArcLengthSpline track_initial_right_;
    ArcLengthSpline track_m25_;
    ArcLengthSpline track_m40_;
    int initial_raceline_;

    Bounds bounds_;
    NormalizationParam normalization_param_;
    Param param_;

    std::unique_ptr<SolverInterface> solver_interface_;
  };

} // namespace mpcc

#endif // MPCC_MPC_H
