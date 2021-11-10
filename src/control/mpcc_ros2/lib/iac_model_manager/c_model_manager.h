#ifndef C_MODEL_MANAGER_H
#define C_MODEL_MANAGER_H

#include <algorithm> // std::min_element
#include <fstream>
#include <iostream>
#include <math.h>
#include <random>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <tuple>
#include <vector>

using namespace std;

class c_model_manager {
private:

  // Driving Distance
  double drivingDistMax = 81000;

  // Vehicle Params (100km/h, 0.8, 0.7 friction)
  double m_Bf_100_08  = 7.831491987387147;
  double m_Cf_100_08  = 1.8151536397336405;
  double m_Df_100_08  = 2251.5264848254137;
  double m_Br_100_08  = 7.46289775363069;
  double m_Cr_100_08  = 2.0786686954613547;
  double m_Dr_100_08  = 2000.0407172109299;
  double m_Cm1_100_08 = 2853.6789;
  double m_Cm1_brake_100_08 = 356.1283;

  double m_Bf_100_066  = 7.464550107079001;
  double m_Cf_100_066  = 1.8638355567362692;
  double m_Df_100_066  = 2287.544711577761;
  double m_Br_100_066  = 6.659046287781959;
  double m_Cr_100_066  = 1.84920913851957;
  double m_Dr_100_066  = 2000.1954989659037;
  double m_Cm1_100_066 = 2853.6789;
  double m_Cm1_brake_100_066 = 356.1283;

  double m_Bf_100_072 = 0.4286 * m_Bf_100_08 + (1-0.4286) * m_Bf_100_066;
  double m_Cf_100_072 = 0.4286 * m_Cf_100_08 + (1-0.4286) * m_Cf_100_066;
  double m_Df_100_072 = 0.4286 * m_Df_100_08 + (1-0.4286) * m_Df_100_066;
  double m_Br_100_072 = 0.4286 * m_Br_100_08 + (1-0.4286) * m_Br_100_066;
  double m_Cr_100_072 = 0.4286 * m_Cr_100_08 + (1-0.4286) * m_Cr_100_066;
  double m_Dr_100_072 = 0.4286 * m_Dr_100_08 + (1-0.4286) * m_Dr_100_066;
  double m_Cm1_100_072 = 0.4286 * m_Cm1_100_08 + (1-0.4286) * m_Cm1_100_066;
  double m_Cm1_brake_100_072 = 0.4286 * m_Cm1_brake_100_08 + (1-0.4286) * m_Cm1_brake_100_066;

  // Vehicle Params (270km/h, 0.8, 0.7 friction);
  double m_Bf_270_08  = 8.176016901254698;
  double m_Cf_270_08  = 1.77049249898797;
  double m_Df_270_08  = 4100.902813473657;
  double m_Br_270_08  = 5.000442572395044;
  double m_Cr_270_08  = 3.0107210273986285;
  double m_Dr_270_08  = 5357.536665163966;
  double m_Cm1_270_08 = 2853.6789;
  double m_Cm1_brake_270_08 = 356.1283;

  double m_Bf_270_066  = 7.582177733390208;
  double m_Cf_270_066  = 1.9079721815357398;
  double m_Df_270_066  = 3828.188787044911;
  double m_Br_270_066  = 9.936416343659259;
  double m_Cr_270_066  = 1.60200302710662;
  double m_Dr_270_066  = 4941.434338235474;
  double m_Cm1_270_066 = 2853.6789;
  double m_Cm1_brake_270_066 = 356.1283;

  double m_Bf_270_072 = 0.4286 * m_Bf_270_08 + (1-0.4286) * m_Bf_270_066;
  double m_Cf_270_072 = 0.4286 * m_Cf_270_08 + (1-0.4286) * m_Cf_270_066;
  double m_Df_270_072 = 0.4286 * m_Df_270_08 + (1-0.4286) * m_Df_270_066;
  double m_Br_270_072 = 0.4286 * m_Br_270_08 + (1-0.4286) * m_Br_270_066;
  double m_Cr_270_072 = 0.4286 * m_Cr_270_08 + (1-0.4286) * m_Cr_270_066;
  double m_Dr_270_072 = 0.4286 * m_Dr_270_08 + (1-0.4286) * m_Dr_270_066;
  double m_Cm1_270_072 = 0.4286 * m_Cm1_270_08 + (1-0.4286) * m_Cm1_270_066;
  double m_Cm1_brake_270_072 = 0.4286 * m_Cm1_brake_270_08 + (1-0.4286) * m_Cm1_brake_270_066;

public:
  c_model_manager();
  ~c_model_manager();
  vector<double> getModelParam(double vx, double driving_dist);
  vector<double> modelParamInterpolation(double vx, double driving_dist);
//   double frictionDecayRatio(double driving_dist);

};



#endif //C_MODEL_MANAGER_H