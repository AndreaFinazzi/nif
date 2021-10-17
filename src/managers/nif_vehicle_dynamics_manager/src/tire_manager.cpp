/**
 * @file   tire_manager.cpp
 * @author Hyunki Seong
 * @date   2021-08-22
 * @brief  class implementation of load transfer based on vehicle dynamics
 * @arg    input      : lateral acceleration, longitudinal acceleration
 *         output     : wheel load: Fz_LF, Fz_RF, Fz_LR, Fz_RR
 *         parameters : ccccc
 * @param  steering_ratio double
 * @param  use_current_speed bool
 * @param  speed_max double
 * @param  lat_accel_max double
 * @param  lon_accel_max double
 * @param  lon_accel_min double
 * @param  longi_time_const double
 * @param  longi_preview double
 *
 * @note   ***IMPORTANT ASSUMPTION***
 *
 */

#include "nif_vehicle_dynamics_manager/tire_manager.hpp"

double TireManager::CalcTireSlipRatio(double v_front, double v_rear) {
  // Calculate tire slip ratio using front/rear wheel speed
  // - scaling rear wheel speed
  double v_rear_scaled = scale_wheelspeed * v_rear;
  // - calculate tire slip ratio
  double sigma = (v_front - v_rear_scaled) /
                 std::max(0.00001, std::max(v_front, v_rear_scaled));
  // - scaling tire slip ratio, convert sign and return it
  return scale_slip_ratio * -sigma;
}

bool TireManager::CalcDynamicsFeasibility(nav_msgs::msg::Path path, double vx,
                                          double ax, double yaw_rate,
                                          double current_steer,
                                          double bank_angle, double dt) {
  // Compute dynamics feasibility of input path w.r.t. tire model
  /* example
  bool dyn_feasiblity = m_tire_manager.CalcDynamicsFeasibility(current_path,
  current_velocity, a_lon, yaw_rate, current_steer, 0.01);
  */
  // - get curvature array of path
  int path_len = path.poses.size();
  std::vector<double> curv_array(path_len, 0.0);
  getCurvatureArray(path, curv_array);
  // - current velocity
  double current_velocity = vx;
  // - check dynamics feasibility
  bool dyn_feasible = true;
  for (int i = 0; i < path.poses.size(); i++) {
    // - calculate current lateral acceleration w.r.t. curvature
    double ay_i = pow(current_velocity, 2.0) * curv_array[i];
    double ay_imax = ComputeLateralAccelLimit(ax, ay_i, yaw_rate, current_steer,
                                              vx, bank_angle);
    // if calculated ay_i is larger than ay limit, infeasible
    if (ay_i > ay_imax) {
      dyn_feasible = false;
      break;
    }
    // propagate velocity using current ax
    current_velocity += ax * dt;
  }
  return dyn_feasible;
}

double TireManager::ComputeLateralAccelLimit(double a_lon, double a_lat,
                                             double yaw_rate,
                                             double current_steer,
                                             double current_velocity,
                                             double bank_angle) {
  // Get Lateral Acceleration Limit considering Tire load transfer.
  auto tire_data = Compute4WheelLateralForceLimit(a_lon, a_lat);
  std::vector<double> tire_loads = std::get<0>(tire_data);
  std::vector<double> tire_FyLimit;
  if (yaw_rate >= 0) {
    // CCW rotation (Fy >= 0)
    tire_FyLimit = std::get<1>(tire_data); // Fy_Max
  } else {
    // CW rotation (Fy < 0)
    tire_FyLimit = std::get<2>(tire_data); // Fy_Max
  }
  double FyF = (tire_FyLimit[0] + tire_FyLimit[1]); // LF, RF tire force
  double FyR = (tire_FyLimit[2] + tire_FyLimit[3]); // LR, RR tire force

  // Get Lateral Acceleration Limit using Vehicle model
  // TODO: Use Vehicle dynamics model class
  double a_lat_max =
      1 / m_total *
      (FyR + FyF * cos(current_steer) - m_total * current_velocity * yaw_rate +
       g * sin(bank_angle));

  return a_lat_max;
}

double TireManager::ComputeLongitudinalAccelLimit(double a_lon, double a_lat) {
  // Get Longitudinal Acceleration Limit considering Tire load transfer.
  auto tire_data = Compute4WheelLongitudinalForceLimit(a_lon, a_lat);
  // TODO check here
  std::vector<double> tire_loads = std::get<0>(tire_data);
  std::vector<double> tire_FxLimit = std::get<1>(tire_data); // positive Fx_ma
  double FxR = (tire_FxLimit[2] + tire_FxLimit[3]); // rear wheel-driven vehicle

  // Get Longitudinal Acceleration Limit using Vehicle model
  // 'desired_accel' from high-level controller is '1 / mass *
  // tire_traction_force', not the net_acceleration.
  double a_lon_max = 1 / m_total * FxR;

  return a_lon_max;
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
TireManager::Compute4WheelLateralForceLimit(double a_lon, double a_lat) {
  // Compute Tire load (Fy) of each (LF, RF, LR, RR)
  std::vector<double> tire_loads;
  tire_loads = ComputeLoadTransfer(a_lon, a_lat);

  // Compute Maximum Lateral Tire Force for each (LF, RF, LR, RR)
  std::vector<double> tire_FyMaxs;
  std::vector<double> tire_FyMins;
  std::tuple<double, double> FyMaxMin;
  FyMaxMin = ComputeForceLimit(tire_loads[0], m_tire_fz_fy_LF);
  tire_FyMaxs.push_back(std::get<0>(FyMaxMin));
  tire_FyMins.push_back(std::get<1>(FyMaxMin));
  FyMaxMin = ComputeForceLimit(tire_loads[1], m_tire_fz_fy_RF);
  tire_FyMaxs.push_back(std::get<0>(FyMaxMin));
  tire_FyMins.push_back(std::get<1>(FyMaxMin));
  FyMaxMin = ComputeForceLimit(tire_loads[2], m_tire_fz_fy_LR);
  tire_FyMaxs.push_back(std::get<0>(FyMaxMin));
  tire_FyMins.push_back(std::get<1>(FyMaxMin));
  FyMaxMin = ComputeForceLimit(tire_loads[3], m_tire_fz_fy_RR);
  tire_FyMaxs.push_back(std::get<0>(FyMaxMin));
  tire_FyMins.push_back(std::get<1>(FyMaxMin));

  // // for Debug
  // for (int i = 0; i < tire_FyMaxs.size(); i++) {
  //   std::cout << "tire_FyMaxs : " << tire_FyMaxs[i] << std::endl;
  // }
  // for (int i = 0; i < tire_FyMins.size(); i++) {
  //   std::cout << "tire_FyMins : " << tire_FyMins[i] << std::endl;
  // }

  return std::make_tuple(tire_loads, tire_FyMaxs, tire_FyMins);
}

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
TireManager::Compute4WheelLongitudinalForceLimit(double a_lon, double a_lat) {
  // Compute Tire load (Fy) of each (LF, RF, LR, RR)
  std::vector<double> tire_loads;
  tire_loads = ComputeLoadTransfer(a_lon, a_lat);
  // Compute Maximum Longitudinal Tire Force for each (LF, RF, LR, RR)
  std::vector<double> tire_FxMaxs;
  std::vector<double> tire_FxMins;
  std::tuple<double, double> FxMaxMin;
  FxMaxMin = ComputeForceLimit(tire_loads[0], m_tire_fz_fx_LF);
  tire_FxMaxs.push_back(std::get<0>(FxMaxMin));
  tire_FxMins.push_back(std::get<1>(FxMaxMin));
  FxMaxMin = ComputeForceLimit(tire_loads[1], m_tire_fz_fx_RF);
  tire_FxMaxs.push_back(std::get<0>(FxMaxMin));
  tire_FxMins.push_back(std::get<1>(FxMaxMin));
  FxMaxMin = ComputeForceLimit(tire_loads[2], m_tire_fz_fx_LR);
  tire_FxMaxs.push_back(std::get<0>(FxMaxMin));
  tire_FxMins.push_back(std::get<1>(FxMaxMin));
  FxMaxMin = ComputeForceLimit(tire_loads[3], m_tire_fz_fx_RR);
  tire_FxMaxs.push_back(std::get<0>(FxMaxMin));
  tire_FxMins.push_back(std::get<1>(FxMaxMin));

  // // for Debug
  // for (int i = 0; i < tire_FxMaxs.size(); i++) {
  //   std::cout << "tire_FxMaxs : " << tire_FxMaxs[i] << std::endl;
  // }
  // for (int i = 0; i < tire_FxMins.size(); i++) {
  //   std::cout << "tire_FxMins : " << tire_FxMins[i] << std::endl;
  // }

  return std::make_tuple(tire_loads, tire_FxMaxs, tire_FxMins);
}

std::tuple<double, double> TireManager::ComputeForceLimit(
    double Fz, std::vector<std::vector<double>> table_ForceMaxMin) {
  // Return Lateral Force limits (FyMax, FyMin)
  std::vector<double> Fz_table, ForceMax_table, ForceMin_table;
  for (int i = 0; i < table_ForceMaxMin.size(); i++) {
    Fz_table.push_back(table_ForceMaxMin[i][0]);
    ForceMax_table.push_back(table_ForceMaxMin[i][1]);
    ForceMin_table.push_back(table_ForceMaxMin[i][2]);
  }
  // Linear interpolation
  // w.r.t. Fz-FyMax(Min) lookup table
  // (with safety factor, model parameter belief)
  double ForceMax_out =
      gamma * LinearInterpolation2D(Fz, Fz_table, ForceMax_table);
  double ForceMin_out =
      gamma * LinearInterpolation2D(Fz, Fz_table, ForceMin_table);

  return std::make_tuple(ForceMax_out, ForceMin_out);
}

double TireManager::LinearInterpolation2D(double input_data,
                                          std::vector<double> input_table,
                                          std::vector<double> output_table) {
  // Interpolation using 2D table (N, 2)
  // size, resolution w.r.t. input_table
  int size_table = input_table.size();
  double res_table =
      (input_table[input_table.size() - 1] - input_table[0]) / (size_table - 1);
  int ind = (input_data - input_table[0]) / res_table;
  int ind_next = ind + 1;
  //
  if (ind_next >= size_table) {
    // Terminal output value in table
    return output_table[size_table - 1];

  } else if (ind <= 0) {
    // Initial output value in table
    return output_table[0];

  } else {
    // Interpolate i th and i+1 th output values
    return output_table[ind] + (output_table[ind_next] - output_table[ind]) /
                                   (input_table[ind_next] - input_table[ind]) *
                                   (input_data - input_table[ind]);
  }
}

std::vector<double> TireManager::ComputeLoadTransfer(double a_lon,
                                                     double a_lat) {
  // Compute and Return Current Tire Loads vector (LF, RF, LR, RR)

  // Compute Longitudinal Load Transfer
  ComputeLongiTransfer(a_lon);
  // Compute Lateral Load Transfer
  ComputeLateralTransfer(a_lat);

  // Total lateral load transfer (left is inner wheel)
  // - sign of lon_load_transfer
  //    (i.e., add m_lon_load_transfer at rear axle when a_lon > 0)
  // - sign of lat_load_transfer
  //    (i.e., add lat_load_transfer at right when a_lat < 0 (CCW))
  double wheel_load_LF =
      static_wheel_load_F - m_lon_load_transfer + m_lat_load_transfer_F;
  double wheel_load_RF =
      static_wheel_load_F - m_lon_load_transfer - m_lat_load_transfer_F;
  double wheel_load_LR =
      static_wheel_load_R + m_lon_load_transfer + m_lat_load_transfer_R;
  double wheel_load_RR =
      static_wheel_load_R + m_lon_load_transfer - m_lat_load_transfer_R;

  // std::cout << "static_wheel_load_F   : " << static_wheel_load_F;
  // std::cout << "\nstatic_wheel_load_R   : " << static_wheel_load_R;
  // std::cout << "\nm_lon_load_transfer   : " << m_lon_load_transfer;
  // std::cout << "\nm_lat_load_transfer_F : " << m_lat_load_transfer_F
  //           << std::endl;

  std::vector<double> wheel_loads_LF_RF_LR_RR;
  wheel_loads_LF_RF_LR_RR.push_back(wheel_load_LF);
  wheel_loads_LF_RF_LR_RR.push_back(wheel_load_RF);
  wheel_loads_LF_RF_LR_RR.push_back(wheel_load_LR);
  wheel_loads_LF_RF_LR_RR.push_back(wheel_load_RR);

  return wheel_loads_LF_RF_LR_RR;
}

void TireManager::ComputeLongiTransfer(double a_lon) {
  // Compute Longitudinal Load Transfer
  m_lon_load_transfer = m_s * a_lon * h_ms / L;
}

void TireManager::ComputeLateralTransfer(double a_lat_ctp) {
  // Compute Lateral Load Transfer (using centrifugal, not centripetal)
  // ! a_lat_ctp : centripetal acceleration (inside rotation point)
  // ! a_lat_ctf : centrifugal acceleration (outside rotation point)
  double a_lat_ctf = -a_lat_ctp;

  // Step 1 - Static wheel load (already done in TireManager::init())

  // Step 2 - Unsprung mass lateral force (unsprung load transfer)
  // (front F, rear R)
  double del_Fz_us_F = a_lat_ctf * m_uns_f * r_wheel / T_f;
  double del_Fz_us_R = a_lat_ctf * m_uns_r * r_wheel / T_r;

  // Step 3 - Sprung mass lateral force through the suspension links
  // (front F, rear R)
  double del_Fz_s_F = m_s * a_lat_ctf * lr / L * h_rcf / T_f;
  double del_Fz_s_R = m_s * a_lat_ctf * lf / L * h_rcr / T_r;

  // Step 4 - Sprung mass roll couple through the springs
  // height of roll axis at COG point.
  double h_rcc = h_rcf + lf * (h_rcr - h_rcf) / L;
  // height of roll couple
  double h_a = h_ms - h_rcc;
  // roll couple
  double C = m_s * a_lat_ctf * h_a;
  // roll rate front/rear axle (counter-clockwise roll) [Nm/deg]
  double K_phi_F_CCW = std::pow(T_f, 2) * K_R_RF / (114.6 * 1000);
  double K_phi_R_CCW = std::pow(T_r, 2) * K_R_RR / (114.6 * 1000);
  // roll rate front/rear axle (clockwise roll) [Nm/deg]
  double K_phi_F_CW = std::pow(T_f, 2) * K_R_LF / (114.6 * 1000);
  double K_phi_R_CW = std::pow(T_r, 2) * K_R_LR / (114.6 * 1000);
  // resulting load transfer (sprung_couple)
  double Fz_F_sc_CCW = K_phi_F_CCW / (K_phi_F_CCW + K_phi_R_CCW) * C / T_f;
  double Fz_R_sc_CCW = K_phi_R_CCW / (K_phi_F_CCW + K_phi_R_CCW) * C / T_r;
  double Fz_F_sc_CW = K_phi_F_CW / (K_phi_F_CW + K_phi_R_CW) * C / T_f;
  double Fz_R_sc_CW = K_phi_R_CW / (K_phi_F_CW + K_phi_R_CW) * C / T_r;

  // Total lateral load transfer (left is inner wheel)
  if (a_lat_ctf < 0) {
    // centrifugal lateral acceleration < 0 (counter-clockwise rotating)
    m_lat_load_transfer_F = del_Fz_us_F + del_Fz_s_F + Fz_F_sc_CCW;
    m_lat_load_transfer_R = del_Fz_us_R + del_Fz_s_R + Fz_R_sc_CCW;
  }

  else {
    // centrifugal lateral acceleration > 0 (clockwise rotating)
    m_lat_load_transfer_F = del_Fz_us_F + del_Fz_s_F + Fz_F_sc_CW;
    m_lat_load_transfer_R = del_Fz_us_R + del_Fz_s_R + Fz_R_sc_CW;
  }
}

void TireManager::getCurvatureArray(nav_msgs::msg::Path path,
                                    std::vector<double> &curvature_array) {
  // Iterate computeCurvature to get curvature for each point in path.
  std::vector<double> vec(path.poses.size(), 0.0);
  for (int i = 1; i < path.poses.size() - 1; i++) {
    vec[i] =
        computeCurvature(path.poses[i - 1], path.poses[i], path.poses[i + 1]);
  }
  // first & last index
  vec[0] = vec[1];
  vec[path.poses.size() - 1] = vec[path.poses.size() - 2];
  curvature_array = vec;
}

double
TireManager::computeCurvature(geometry_msgs::msg::PoseStamped pose_i_prev,
                              geometry_msgs::msg::PoseStamped pose_i,
                              geometry_msgs::msg::PoseStamped pose_i_next) {

  double curvature = CURVATURE_MINIMUM;
  // Calcalate curvature(+/) of a line with discrete points(3)
  // References:
  // https://ed-matrix.com/mod/page/view.php?id=2771
  // https://www.skedsoft.com/books/maths-for-engineers-1/radius-of-curvature-in-parametric-form
  double x_1 = pose_i_prev.pose.position.x;
  double x_2 = pose_i.pose.position.x;
  double x_3 = pose_i_next.pose.position.x;

  double y_1 = pose_i_prev.pose.position.y;
  double y_2 = pose_i.pose.position.y;
  double y_3 = pose_i_next.pose.position.y;

  double s_12 = sqrt((x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2));
  double s_23 = sqrt((x_2 - x_3) * (x_2 - x_3) + (y_2 - y_3) * (y_2 - y_3));
  double s_13 = sqrt((x_1 - x_3) * (x_1 - x_3) + (y_1 - y_3) * (y_1 - y_3));

  if (s_12 * s_23 * s_13 == 0) {
    curvature = CURVATURE_MINIMUM;
  } else {
    double x_d = (x_3 - x_1) / s_13;
    double x_d12 = (x_2 - x_1) / s_12;
    double x_d23 = (x_3 - x_2) / s_23;
    double x_dd = (x_d23 - x_d12) / s_13;

    double y_d = (y_3 - y_1) / s_13;
    double y_d12 = (y_2 - y_1) / s_12;
    double y_d23 = (y_3 - y_2) / s_23;
    double y_dd = (y_d23 - y_d12) / s_13;

    if (x_d * x_d + y_d * y_d == 0) {
      curvature = CURVATURE_MINIMUM;
    } else {
      // curvature = abs(x_d * y_dd - y_d * x_dd) /
      //             pow(sqrt(x_d * x_d + y_d * y_d), 3.0);
      curvature =
          (x_d * y_dd - y_d * x_dd) / pow(sqrt(x_d * x_d + y_d * y_d), 3.0);
    }
  }

  return curvature;
}

// // usage example
// int main() {
//   double a_lat, a_lon;

//   TireManager tire_load_manager;

//   std::vector<double> tire_load;
//   tire_load = tire_load_manager.ComputeLoadTransfer(a_lon, a_lat);

//   return 0;
// }
