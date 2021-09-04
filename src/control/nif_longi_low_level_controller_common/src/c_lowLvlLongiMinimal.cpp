//
// Created by usrg on 9/2/21.
//

#include "c_lowLvlLongiMinimal.h"

using namespace nif::control::low_level;

LowLvlLongiMinimal::LowLvlLongiMinimal(std::string &config_file_path_) {
    this->config_file_path = config_file_path_;
    YAML::Node config = YAML::LoadFile(config_file_path_);

    if(!config["throttle_params"] || !config["throttle_params"] || !config["brake_params"]){
        throw std::runtime_error("Longi low level controller : model_params not fully defined in config file.");
    }

    //! Load model parameters
    YAML::Node throttle_params = config["throttle_params"];
    YAML::Node brake_params = config["brake_params"];

    this->param_ts = config["time_step"].as<double>();

    this->param_throttle_p_gain = throttle_params["P_gain"].as<double>();
    this->param_throttle_i_gain = throttle_params["I_gain"].as<double>();
    this->param_throttle_d_gain = throttle_params["D_gain"].as<double>();
    this->param_throttle_max_integrator_error = throttle_params["max_integrator_error"].as<double>();
    this->param_throttle_cmd_max = throttle_params["cmd_max"].as<double>();
    this->param_throttle_cmd_min = throttle_params["cmd_min"].as<double>();
    //! Init throttle pid controller
    this->throttle_pid = utils::PID(param_throttle_p_gain,
                                    param_throttle_i_gain,
                                    param_throttle_d_gain,
                                    param_ts,
                                    param_throttle_max_integrator_error,
                                    param_throttle_cmd_max,
                                    param_throttle_cmd_min);

    this->param_brake_p_gain = brake_params["P_gain"].as<double>();
    this->param_brake_i_gain = brake_params["I_gain"].as<double>();
    this->param_brake_d_gain = brake_params["D_gain"].as<double>();
    this->param_brake_max_integrator_error = brake_params["max_integrator_error"].as<double>();
    this->param_vel_error_deadband = brake_params["vel_error_deadband"].as<double>();
    this->param_brake_cmd_max = throttle_params["cmd_max"].as<double>();
    this->param_brake_cmd_min = throttle_params["cmd_min"].as<double>();
    //! Init brake pid controller
    this->brake_pid = utils::PID(param_brake_p_gain,
                                 param_brake_i_gain,
                                 param_brake_d_gain,
                                 param_ts,
                                 param_brake_max_integrator_error,
                                 param_brake_cmd_max,
                                 param_brake_cmd_min);
}

double LowLvlLongiMinimal::getDesiredVelMps() const {
    return desired_vel_mps;
}

void LowLvlLongiMinimal::setDesiredVelMps(double desiredVelMps) {
    desired_vel_mps = desiredVelMps;
}

double LowLvlLongiMinimal::getParamThrottlePGain() const {
    return param_throttle_p_gain;
}

void LowLvlLongiMinimal::setParamThrottlePGain(double paramThrottlePGain) {
    param_throttle_p_gain = paramThrottlePGain;
}

double LowLvlLongiMinimal::getParamThrottleIGain() const {
    return param_throttle_i_gain;
}

void LowLvlLongiMinimal::setParamThrottleIGain(double paramThrottleIGain) {
    param_throttle_i_gain = paramThrottleIGain;
}

double LowLvlLongiMinimal::getParamThrottleDGain() const {
    return param_throttle_d_gain;
}

void LowLvlLongiMinimal::setParamThrottleDGain(double paramThrottleDGain) {
    param_throttle_d_gain = paramThrottleDGain;
}

double LowLvlLongiMinimal::getParamThrottleMaxIntegratorError() const {
    return param_throttle_max_integrator_error;
}

void LowLvlLongiMinimal::setParamThrottleMaxIntegratorError(double paramThrottleMaxIntegratorError) {
    param_throttle_max_integrator_error = paramThrottleMaxIntegratorError;
}

double LowLvlLongiMinimal::getParamBrakePGain() const {
    return param_brake_p_gain;
}

void LowLvlLongiMinimal::setParamBrakePGain(double paramBrakePGain) {
    param_brake_p_gain = paramBrakePGain;
}

double LowLvlLongiMinimal::getParamBrakeIGain() const {
    return param_brake_i_gain;
}

void LowLvlLongiMinimal::setParamBrakeIGain(double paramBrakeIGain) {
    param_brake_i_gain = paramBrakeIGain;
}

double LowLvlLongiMinimal::getParamBrakeDGain() const {
    return param_brake_d_gain;
}

void LowLvlLongiMinimal::setParamBrakeDGain(double paramBrakeDGain) {
    param_brake_d_gain = paramBrakeDGain;
}

double LowLvlLongiMinimal::getParamBrakeMaxIntegratorError() const {
    return param_brake_max_integrator_error;
}

void LowLvlLongiMinimal::setParamBrakeMaxIntegratorError(double paramBrakeMaxIntegratorError) {
    param_brake_max_integrator_error = paramBrakeMaxIntegratorError;
}
