
/*
intelligent driver model in c++
Created on : Nov 29th, 2021
Author : Calvin Chanyoung Jung
*/

#pragma once

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <math.h>
#include <string>
#include <yaml-cpp/yaml.h>

#define DEFAULT_DES_SPEED 120 / 3.6 // KPH
#define EPS 0.0000001

struct IDM_PARAM {
  /* data */
  double s0;            // [m] desired gap dist
  double s1;            // [m] jam dist (zero is okay)
  double v_desired;     // [mps] desired speed
  double time_headway;  // [sec] time gap. 1.6
  double accel_max;     // [m/s2] maximum accel. 0.73
  double decel_desired; // [m/s2] desired deceleration. 1.67
  double delta;         // accel exponent
  double veh_l;         //[m] vehicle length for calc gap
};

class IDM {
  /*
  Intelligent Driver Model (IDM)
      @ Parameters [params]
      - s0              : jam distance front car
      - s1              : jam distance rear car
      - v_desired       : desired ego velocity
      - time_headway    : time headway (== time gap)
      - accel_max       : max acceleration
      - decel_desired   : desired deceleration
      - delta           : acceleration exponent
      - veh_l           : vehicle length
  */

public:
  IDM();
  IDM(const std::string& config_file_path_);
  void loadConfig(const std::string& config_file_path_);
  void setParams(const IDM_PARAM& param_);
  IDM_PARAM getParams();

  // setter
  void setParamS0(const double s0_);
  void setParamS1(const double s1_);
  void setParamVDesired(const double v_desired_);
  void setParamTimeHeadway(const double time_headway_);
  void setParamAccelMax(const double accel_max_);
  void setParamDecelDesired(const double decel_desired_);
  void setParamAccelDelta(const double delta_);
  void setParamVehLen(const double veh_l_);
  void setEgoVel(double ego_vel_);
  void setOppoStatus(double gap_, double cipv_vel_rel);

  // getter
  double getParamS0();
  double getParamS1();
  double getParamVDesired();
  double getParamTimeHeadway();
  double getParamAccelMax();
  double getParamDecelDesired();
  double getParamAccelDelta();
  double getParamVehLen();

  // main func
  void calcAccel(double ego_vel_, double gap_, double cipv_vel_rel_);
  double getACCCmd();

private:
  std::string m_idm_config_file_path;
  IDM_PARAM m_idm_param;

  bool m_init_done_flg;

  double m_ego_vel_abs;  // absolute velocity [mps]
  double m_cur_gap;      // meter
  double m_cipv_vel_rel; // relative velocity [mps]
  bool m_estop_flg;      // if true, complete stop

  // result
  double m_desired_accel;
};