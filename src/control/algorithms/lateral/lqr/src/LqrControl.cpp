//
// Created by usrg on 8/30/21.
//

#include "LqrControl.h"

nif::control::lateral::LqrControl::LqrControl(string config_file_path_) {
    this->config_load_success_flg = false;
    this->path_update_good_flg = false;

    this->setConfigFilePath(config_file_path_);
    this->config_load_success_flg = this->loadConfig(config_file_path_);
    if (!this->config_load_success_flg) {
        throw std::runtime_error("Config file is not successfully loaded.");
    }
    this->lateral_lqr = nif::control::lateral::lqr::LateralLQR::newPtr(config_file_path_);
}

bool nif::control::lateral::LqrControl::loadConfig(string config_file_path_) {
    bool success_flg = true;

    YAML::Node config = YAML::LoadFile(config_file_path_);

    if (!config["max_steering_angle_deg"]) {
        success_flg = false;
    }
    if (!config["steering_units_multiplier"]) {
        success_flg = false;
    }
    if (!config["pure_pursuit_min_dist_m"]) {
        success_flg = false;
    }
    if (!config["pure_pursuit_max_dist_m"]) {
        success_flg = false;
    }
    if (!config["pure_pursuit_k_vel_m_ms"]) {
        success_flg = false;
    }
    if (!config["steering_max_ddeg_dt"]) {
        success_flg = false;
    }

    if (success_flg) {
        this->max_steering_angle_deg = config["max_steering_angle_deg"].as<double>();
        this->steering_units_multiplier = config["steering_units_multiplier"].as<double>();
        this->pure_pursuit_min_dist_m = config["pure_pursuit_min_dist_m"].as<double>();
        this->pure_pursuit_max_dist_m = config["pure_pursuit_max_dist_m"].as<double>();
        this->pure_pursuit_k_vel_m_ms = config["pure_pursuit_k_vel_m_ms"].as<double>();
        this->steering_max_ddeg_dt = config["steering_max_ddeg_dt"].as<double>();
    } else {
        std::cout << "Missing params on Config file. Ignoring param updates." << std::endl;
    }

    return success_flg;
}

const string &nif::control::lateral::LqrControl::getConfigFilePath() const {
    return config_file_path;
}

void nif::control::lateral::LqrControl::setConfigFilePath(const string &configFilePath) {
    config_file_path = configFilePath;
    this->config_load_success_flg = this->loadConfig(config_file_path);

}

bool nif::control::lateral::LqrControl::isConfigLoad() const {
    return config_load_success_flg;
}

const LqrControlParam &nif::control::lateral::LqrControl::getParams() const {
    return params;
}

void nif::control::lateral::LqrControl::setParams(const LqrControlParam &params) {
    this->params = params;
}

void nif::control::lateral::LqrControl::solve() {

    // NOTE : Before calling the solve function, make sure that you update the reference path and odometry in global frame
    auto cur_state = nif::control::lateral::lqr::utils::LQRState(this->cur_odom_in_global);
    cur_state(2, 0) = this->current_speed_ms;

    double track_distance =
            pure_pursuit_min_dist_m + pure_pursuit_k_vel_m_ms * cur_state(2, 0);
    if (track_distance > pure_pursuit_max_dist_m)
        track_distance = pure_pursuit_max_dist_m;
    if (track_distance < pure_pursuit_min_dist_m)
        track_distance = pure_pursuit_min_dist_m;

    // Track on the trajectory
    double target_distance = 0.0;
    bool target_reached_end = false;

    nif::control::lateral::lqr::utils::track(this->ref_path_in_global.poses, this->cur_odom_in_global, track_distance,
                                             this->lqr_tracking_idx, target_distance, target_reached_end);

    auto goal_state = nif::control::lateral::lqr::utils::LQRGoal(
            this->ref_path_in_global.poses[this->lqr_tracking_idx]);
    auto state_error = this->lateral_lqr->computeError(cur_state, goal_state);
    this->steer_angle_cmd = this->lateral_lqr->process(cur_state, goal_state);

    // Make sure steering angle is within range
    if (steer_angle_cmd > max_steering_angle_deg)
        steer_angle_cmd = max_steering_angle_deg;
    if (steer_angle_cmd < -max_steering_angle_deg)
        steer_angle_cmd = -max_steering_angle_deg;

    this->steer_angle_cmd *= nif::common::vehicle_param::STEERING_RATIO;
    this->steer_angle_cmd = this->invert_steering ? -1 * this->steer_angle_cmd : this->steer_angle_cmd;
    this->last_steer_angle_cmd = this->steer_angle_cmd;
}

double nif::control::lateral::LqrControl::getSteerAngleCmd() const {
    return steer_angle_cmd;
}

void nif::control::lateral::LqrControl::setGlobalRefPath(const nav_msgs::msg::Path::SharedPtr path_ptr) {
    if (path_ptr->poses.size() > 0)
        ref_path_in_global = *path_ptr;
    else
        std::cout << "Reference path is empty. Ignore updating." << std::endl;
}

void nif::control::lateral::LqrControl::setCurOdom(nav_msgs::msg::Odometry::SharedPtr odom_ptr) {
    cur_odom_in_global = *odom_ptr;
}
