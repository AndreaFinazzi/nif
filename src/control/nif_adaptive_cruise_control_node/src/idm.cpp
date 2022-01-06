// #include "../include/idm.hpp"
#include "nif_adaptive_cruise_control_node/idm.hpp"

IDM::IDM() {
  // default configuration
  m_idm_config_file_path = "";

  m_idm_param.s0 = 5.0;
  m_idm_param.s1 = 0.0;
  m_idm_param.v_desired = DEFAULT_DES_SPEED; // mps
  m_idm_param.time_headway = 2.0;
  m_idm_param.accel_max = 2.0;
  m_idm_param.decel_desired = 5.0;
  m_idm_param.delta = 4.0;
  m_idm_param.veh_l = 0.0;

  m_init_done_flg = true;
}

IDM::IDM(const std::string &config_file_path_) {
  m_idm_config_file_path = config_file_path_;

  loadConfig(m_idm_config_file_path);
  m_init_done_flg = true;
}

void IDM::loadConfig(const std::string &config_file_path_) {
  YAML::Node config = YAML::LoadFile(config_file_path_);

  if (!config["acc_config_param"]) {
    throw std::runtime_error("acc_config_param not defined in config file.");
  }

  //! Load model parameters
  YAML::Node model_params = config["acc_config_param"];

  if (!model_params["param_s0"]) {
    throw std::runtime_error("param_s0 is not properly settup.");
  } else {
    m_idm_param.s0 = model_params["param_s0"].as<double>();
  }

  if (!model_params["param_s1"]) {
    throw std::runtime_error("param_s1 is not properly settup.");
  } else {
    m_idm_param.s1 = model_params["param_s1"].as<double>();
  }

  if (!model_params["param_v_desired"]) {
    throw std::runtime_error("param_v_desired is not properly settup.");
  } else {
    m_idm_param.v_desired = model_params["param_v_desired"].as<double>();
  }

  if (!model_params["param_time_headway"]) {
    throw std::runtime_error("param_time_headway is not properly settup.");
  } else {
    m_idm_param.time_headway = model_params["param_time_headway"].as<double>();
  }

  if (!model_params["param_accel_max"]) {
    throw std::runtime_error("param_accel_max is not properly settup.");
  } else {
    m_idm_param.accel_max = model_params["param_accel_max"].as<double>();
  }

  if (!model_params["param_decel_desired"]) {
    throw std::runtime_error("param_decel_desired is not properly settup.");
  } else {
    m_idm_param.decel_desired =
        model_params["param_decel_desired"].as<double>();
  }

  if (!model_params["param_delta"]) {
    throw std::runtime_error("param_delta is not properly settup.");
  } else {
    m_idm_param.delta = model_params["param_delta"].as<double>();
  }

  if (!model_params["param_veh_l"]) {
    throw std::runtime_error("param_veh_l is not properly settup.");
  } else {
    m_idm_param.veh_l = model_params["param_veh_l"].as<double>();
  }
}

void IDM::setParams(const IDM_PARAM &param_) { m_idm_param = param_; }

IDM_PARAM IDM::getParams() { return m_idm_param; }

void IDM::setParamS0(const double s0_) { m_idm_param.s0 = s0_; }
void IDM::setParamS1(const double s1_) { m_idm_param.s1 = s1_; }
void IDM::setParamVDesired(const double v_desired_) {
  m_idm_param.v_desired = v_desired_;
}
void IDM::setParamTimeHeadway(const double time_headway_) {
  m_idm_param.time_headway = time_headway_;
}
void IDM::setParamAccelMax(const double accel_max_) {
  m_idm_param.accel_max = accel_max_;
}
void IDM::setParamDecelDesired(const double decel_desired_) {
  m_idm_param.decel_desired = decel_desired_;
}
void IDM::setParamAccelDelta(const double delta_) {
  m_idm_param.delta = delta_;
}
void IDM::setParamVehLen(const double veh_l_) { m_idm_param.veh_l = veh_l_; }
void IDM::setEgoVel(double ego_vel_) { m_ego_vel_abs = ego_vel_; }
void IDM::setOppoStatus(double gap_, double cipv_vel_rel_) {
  m_cur_gap = gap_;
  m_cipv_vel_rel = cipv_vel_rel_;
}

double IDM::getParamS0() { return m_idm_param.s0; }
double IDM::getParamS1() { return m_idm_param.s1; }
double IDM::getParamVDesired() { return m_idm_param.v_desired; }
double IDM::getParamTimeHeadway() { return m_idm_param.time_headway; }
double IDM::getParamAccelMax() { return m_idm_param.accel_max; }
double IDM::getParamDecelDesired() { return m_idm_param.decel_desired; }
double IDM::getParamAccelDelta() { return m_idm_param.delta; }
double IDM::getParamVehLen() { return m_idm_param.veh_l; }

void IDM::calcAccel(double ego_vel_, double gap_, double cipv_vel_rel_) {
  // Calculate ACC command using IDM
  auto cipv_vel_rel =
      ego_vel_ - cipv_vel_rel_; // in IDM, other_v_rel == ego_v - other_v

  if (m_estop_flg) {
    cipv_vel_rel_ = ego_vel_;
  }
  ego_vel_ = std::max(ego_vel_, 0.0);

  auto desired_gap =
      m_idm_param.s0 + m_idm_param.s1 * sqrt(ego_vel_ / m_idm_param.v_desired) +
      m_idm_param.time_headway * ego_vel_ +
      ego_vel_ * cipv_vel_rel /
          (2 * sqrt(m_idm_param.accel_max * m_idm_param.decel_desired));

  auto curr_gap = std::max(gap_ - m_idm_param.veh_l, EPS);

  // if (m_estop_flg) {
  //   curr_gap = 0.5 * m_idm_param.s0;
  // }

  m_desired_accel =
      m_idm_param.accel_max *
      (1 - pow((ego_vel_ / m_idm_param.v_desired), m_idm_param.delta) -
       pow((desired_gap / curr_gap), 2));
  m_desired_accel = std::clamp(m_desired_accel, -1 * m_idm_param.decel_desired,
                               m_idm_param.accel_max);
}

double IDM::getACCCmd() { return m_desired_accel; }