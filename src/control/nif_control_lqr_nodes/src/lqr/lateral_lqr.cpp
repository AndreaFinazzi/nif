#include "nif_control_lqr_nodes/lqr/lateral_lqr.h"
#include "nif_control_lqr_nodes/lqr/lateral_dynamics.h"
#include "nif_control_lqr_nodes/lqr/solver.h"
#include <cmath>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

namespace bvs_control {
namespace lqr {

LateralLQR::LateralLQR(std::string config_file) {
    loadConfig(config_file);
}

void
LateralLQR::loadConfig(std::string config_file) {
    YAML::Node config = YAML::LoadFile(config_file);

    //! A few checks for a valid yaml, let YAML throw the rest if any values are missing
    if(!config["model_params"]) {
        throw std::runtime_error("model_params not defined in config file.");
    }
    if(!config["velocity_brackets"]) {
        throw std::runtime_error("velocity_brackets not defined in config file.");
    }
    //! Load model parameters
    YAML::Node model_params = config["model_params"];
    LateralDynamics::ModelParams model;
    model.cornering_stiffness_front = model_params["cornering_stiffness_front"].as<double>();
    model.cornering_stiffness_rear = model_params["cornering_stiffness_rear"].as<double>();
    model.mass = model_params["mass"].as<double>();
    model.length_front = model_params["length_front"].as<double>();
    model.length_rear = model_params["length_rear"].as<double>();
    model.yaw_moment_inertia = model_params["yaw_moment_inertia"].as<double>();
    
    //! Use those params to generate the model and generate feedback matrices
    LateralDynamics bicycle_model(model);
    KMatrices new_matrices;
    YAML::Node velocity_brackets = config["velocity_brackets"];
    for(std::size_t i = 0; i < velocity_brackets.size(); ++i) {
        //! Get A and B matrices
        double velocity = velocity_brackets[i]["velocity"].as<double>();
        auto AB = bicycle_model.linearize(velocity);

        //! Generate Q and R matrices
        YAML::Node q_trace = velocity_brackets[i]["q_trace"];
        YAML::Node r_trace = velocity_brackets[i]["r_trace"];
        LateralDynamics::QMatrix Q = LateralDynamics::QMatrix::Zero();
        LateralDynamics::RMatrix R = LateralDynamics::RMatrix::Zero();
        for(std::size_t i = 0; i < q_trace.size(); ++i) {
            Q(i,i) = q_trace[i].as<double>();
        }
        for(std::size_t i = 0; i < r_trace.size(); ++i) {
            R(i,i) = r_trace[i].as<double>();
        }

        //! Calculate feedback matrix with lqr
        Eigen::MatrixXd K;
        lqr(AB.first, AB.second, Q, R, K);

        new_matrices.push_back({velocity, K});
    }
    //! Make sure the config is valid before setting matrices_
    matrices_ = new_matrices;
} /* loadConfig() */

LateralLQR::ErrorMatrix
LateralLQR::computeError(LateralLQR::StateMatrix state, LateralLQR::GoalMatrix goal) {
    //! Do some frame shifting
    double x_goal_shift = goal(0,0) - state(0,0);
    double y_goal_shift = goal(1,0) - state(1,0);
    double y_goal = x_goal_shift * std::sin(-state(4,0)) + y_goal_shift * std::cos(-state(4,0));
    double yaw_error = wrap_angle(state(4,0) - goal(2,0));

    //! Assign results and return
    LateralLQR::ErrorMatrix result;
    result(0,0) = -y_goal;
    result(1,0) = state(3,0) + state(2,0) * yaw_error;
    result(2,0) = yaw_error;
    //! This could be calculated better based on curvature.. 
    //! But we have no decent yaw rate estimate any :shrug:
    result(3,0) = state(5,0);
    return result;
}

double
LateralLQR::process(
    LateralLQR::ErrorMatrix error,
    double x_vel_body
) {
    LateralLQR::KMatrix matrix;
    // Find the tier where the upper limit is higher than the current velocity
    // If no tier contains this velocity it will use the last tier
    for(auto it = matrices_.begin(); it != matrices_.end(); ++it) {
        matrix = it->second;
        if(it->first > x_vel_body) {
            break;
        }
    }
    // Compute feedback control
    return -matrix * error;
}

double
LateralLQR::process(
    LateralLQR::StateMatrix state,
    LateralLQR::GoalMatrix goal
) {
    // Convert to error matrix, x vel body
    // and call the other process function
    return process(
        computeError(state, goal),
        state(2,0)
    );
}

LateralLQR::KMatrices&
LateralLQR::getKMatrices() {
    return matrices_;
}

double
LateralLQR::wrap_angle(double theta) {
    while(theta >= M_PI) {
        theta -= 2.0 * M_PI;
    }
    while(theta < -M_PI) {
        theta += 2.0 * M_PI;
    }
    return theta;
}

} /* namespace lqr */
} /* namespace bvs_control */
