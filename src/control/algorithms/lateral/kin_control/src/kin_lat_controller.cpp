//
// Created by usrg on 8/30/21.
//

#include "../include/kin_lat_controller.h"

nif::control::lateral::kinControl::kinControl(const string &config_file_path_) {
    this->config_load_success_flg = false;
    this->path_update_good_flg = false;

    this->setConfigFilePath(config_file_path_);
    this->config_load_success_flg = this->loadConfig(config_file_path_);
    if (!config_load_success_flg) {
        throw std::runtime_error("Config file is not successfully loaded.");
    }
}

const string &nif::control::lateral::kinControl::getConfigFilePath() const {
    return config_file_path;
}

void nif::control::lateral::kinControl::setConfigFilePath(const string &configFilePath) {
    config_file_path = configFilePath;
    this->config_load_success_flg = this->loadConfig(config_file_path);
}

bool nif::control::lateral::kinControl::isConfigLoad() const {
    return config_load_success_flg;
}

const kinControlParam &nif::control::lateral::kinControl::getParams() const {
    return params;
}

void nif::control::lateral::kinControl::setParams(const kinControlParam &params_) {
    this->params = params_;
}

void nif::control::lateral::kinControl::setBodyRefPath(const nav_msgs::msg::Path::SharedPtr &path_ptr) {
    ref_path = *path_ptr;

    // Determines lookahead distance based on speed and bounds
    double lookahead_distance =
            std::max(this->params.min_lookahead,
                     std::min(this->params.max_lookahead, this->cur_vel * this->params.lookahead_speed_ratio));

    // Unpacks the message and finds the index correlated to the lookahead
    // distance
    std::vector<geometry_msgs::msg::PoseStamped> path = ref_path.poses;
    int idx = findLookaheadIndex(path, lookahead_distance);

    // Sets the lookahead and lateral error
    if (path.size() > 0) {
        this->look_ahead_error = path[idx].pose.position.y;
        this->lat_error = path[0].pose.position.y;
    } else {
        this->look_ahead_error = 0.0;
    }

//    this->path_update_time = this->now();
}

// Final output(steering angle cmd)
double nif::control::lateral::kinControl::getSteerAngleCmd() const {
    return steer_angle_cmd;
}

void nif::control::lateral::kinControl::solve() {
    // NOTE : Before calling the solve function, make sure that you update the reference path
    // calc output
    this->calculateFFW();
    this->calculateFB();
    this->calculateSteeringCmd();
}

bool nif::control::lateral::kinControl::loadConfig(const string &config_file_path_) {
    bool success_flg = true;

    YAML::Node config = YAML::LoadFile(config_file_path_);
    if (!config["min_lookahead"]) {
        success_flg = false;
    }
    if (!config["max_lookahead"]) {
        success_flg = false;
    }
    if (!config["lookahead_speed_ratio"]) {
        success_flg = false;
    }
    if (!config["proportional_gain"]) {
        success_flg = false;
    }
    if (!config["max_steer_angle"]) {
        success_flg = false;
    }
    if (!config["steering_override_threshold"]) {
        success_flg = false;
    }

    if (success_flg) {
        this->params.min_lookahead = config["min_lookahead"].as<double>();
        this->params.min_lookahead = config["max_lookahead"].as<double>();
        this->params.min_lookahead = config["lookahead_speed_ratio"].as<double>();
        this->params.min_lookahead = config["proportional_gain"].as<double>();
        this->params.min_lookahead = config["max_steer_angle"].as<double>();
        this->params.min_lookahead = config["steering_override_threshold"].as<double>();
    } else {
        std::cout << "Missing params on Config file. Ignoring param updates." << std::endl;
    }

    return success_flg;
}

void nif::control::lateral::kinControl::calculateFFW() {
    // Desired yaw rate from feedforward
    this->feedforward_error = 0.0; // can add feedforward with a curvature:
    // this->speed_ * this->curvature_;
}

void nif::control::lateral::kinControl::calculateFB() {
    double Kp = this->params.proportional_gain;
    this->feedback_error = -Kp * this->look_ahead_error;
}

void nif::control::lateral::kinControl::calculateSteeringCmd() {
    double L = nif::common::vehicle_param::VEH_WHEEL_BASE;
    this->steer_angle_cmd = (this->cur_vel > 1.0) ?
                            L * (this->feedback_error + this->feedforward_error) / this->cur_vel :
                            L * (this->feedback_error + this->feedforward_error);
    this->steer_angle_cmd = this->steer_angle_cmd * nif::common::vehicle_param::STEERING_RATIO;
    this->steer_angle_cmd = this->invert_steering ? -1 * this->steer_angle_cmd : this->steer_angle_cmd;
    this->last_steer_angle_cmd = this->steer_angle_cmd;
}

void nif::control::lateral::kinControl::setCmdsToZeros() {
    this->feedforward_error = 0.0;
    this->feedback_error = 0.0;
    this->look_ahead_error = 0.0;
    this->curvature = 0.0;

    this->steer_angle_cmd - 0.0;
}

int nif::control::lateral::kinControl::findLookaheadIndex(std::vector<geometry_msgs::msg::PoseStamped> refPath,
                                                          double desLookaheadValue) {
    // Finds first path pose that has x distance > lookahead distance
    for (int i = 0; i < refPath.size(); i++) {
        if (refPath[i].pose.position.x > desLookaheadValue) {
            return i;
        }
    }
    return refPath.size() - 1;
}
