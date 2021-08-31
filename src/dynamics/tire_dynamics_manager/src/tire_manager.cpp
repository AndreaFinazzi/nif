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

#include "tire_dynamics_manager/tire_manager.hpp"

std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
TireManager::Compute4WheelLateralForceLimit(double a_lon, double a_lat) {
    // Compute Tire load (Fy) of each (LF, RF, LR, RR)
    std::vector<double> tire_loads;
    tire_loads = ComputeLoadTransfer(a_lon, a_lat);

    // Compute Maximum Lateral Tire Force for each (LF, RF, LR, RR)
    std::vector<double> tire_FyMaxs;
    std::vector<double> tire_FyMins;
    std::tuple<double, double> FyMaxMin;
    FyMaxMin = ComputeLateralForceLimit(tire_loads[0], m_tire_fz_fy_LF);
    tire_FyMaxs.push_back(std::get<0>(FyMaxMin));
    tire_FyMins.push_back(std::get<1>(FyMaxMin));
    FyMaxMin = ComputeLateralForceLimit(tire_loads[1], m_tire_fz_fy_RF);
    tire_FyMaxs.push_back(std::get<0>(FyMaxMin));
    tire_FyMins.push_back(std::get<1>(FyMaxMin));
    FyMaxMin = ComputeLateralForceLimit(tire_loads[2], m_tire_fz_fy_LR);
    tire_FyMaxs.push_back(std::get<0>(FyMaxMin));
    tire_FyMins.push_back(std::get<1>(FyMaxMin));
    FyMaxMin = ComputeLateralForceLimit(tire_loads[3], m_tire_fz_fy_RR);
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

std::tuple<double, double> TireManager::ComputeLateralForceLimit(
        double Fz, std::vector<std::vector<double>> table_FyMaxMin) {
    // Return Lateral Force limits (FyMax, FyMin)
    std::vector<double> Fz_table_LF, FyMax_table_LF, FyMin_table_LF;
    for (int i = 0; i < table_FyMaxMin.size(); i++) {
        Fz_table_LF.push_back(table_FyMaxMin[i][0]);
        FyMax_table_LF.push_back(table_FyMaxMin[i][1]);
        FyMin_table_LF.push_back(table_FyMaxMin[i][2]);
    }
    // Linear interpolation
    // w.r.t. Fz-FyMax(Min) lookup table
    // (with safety factor, model parameter belief)
    double FyMax_out =
            gamma * LinearInterpolation2D(Fz, Fz_table_LF, FyMax_table_LF);
    double FyMin_out =
            gamma * LinearInterpolation2D(Fz, Fz_table_LF, FyMin_table_LF);

    return std::make_tuple(FyMax_out, FyMin_out);
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

void TireManager::ComputeLateralTransfer(double a_lat) {
    // Compute Lateral Load Transfer

    // Step 1 - Static wheel load (already done in TireManager::init())

    // Step 2 - Unsprung mass lateral force (unsprung load transfer)
    // (front F, rear R)
    double del_Fz_us_F = a_lat * m_uns_f * r_wheel / T_f;
    double del_Fz_us_R = a_lat * m_uns_r * r_wheel / T_r;

    // Step 3 - Sprung mass lateral force through the suspension links
    // (front F, rear R)
    double del_Fz_s_F = m_s * a_lat * lr / L * h_rcf / T_f;
    double del_Fz_s_R = m_s * a_lat * lf / L * h_rcr / T_r;

    // Step 4 - Sprung mass roll couple through the springs
    // height of roll axis at COG point.
    double h_rcc = h_rcf + lf * (h_rcr - h_rcf) / L;
    // height of roll couple
    double h_a = h_ms - h_rcc;
    // roll couple
    double C = m_s * a_lat * h_a;
    // roll rate front/rear axle (counter-clockwise roll) [Nm/deg]
    double K_phi_F_CCW = std::pow(T_f, 2) * K_R_LF / (114.6 * 1000);
    double K_phi_R_CCW = std::pow(T_f, 2) * K_R_LR / (114.6 * 1000);
    // roll rate front/rear axle (clockwise roll) [Nm/deg]
    double K_phi_F_CW = std::pow(T_f, 2) * K_R_RF / (114.6 * 1000);
    double K_phi_R_CW = std::pow(T_f, 2) * K_R_RR / (114.6 * 1000);
    // resulting load transfer (sprung_couple)
    double Fz_F_sc_CCW = K_phi_F_CCW / (K_phi_F_CCW + K_phi_R_CCW) * C / T_f;
    double Fz_R_sc_CCW = K_phi_R_CCW / (K_phi_F_CCW + K_phi_R_CCW) * C / T_r;
    double Fz_F_sc_CW = K_phi_F_CW / (K_phi_F_CW + K_phi_R_CW) * C / T_f;
    double Fz_R_sc_CW = K_phi_R_CW / (K_phi_F_CW + K_phi_R_CW) * C / T_r;

    // Total lateral load transfer (left is inner wheel)
    if (a_lat < 0) {
        // If lateral accel < 0 (counter-clockwise rotating)
        m_lat_load_transfer_F = del_Fz_us_F + del_Fz_s_F + Fz_F_sc_CCW;
        m_lat_load_transfer_R = del_Fz_us_R + del_Fz_s_R + Fz_R_sc_CCW;
    } else {
        // If lateral accel > 0 (clockwise rotating)
        m_lat_load_transfer_F = del_Fz_us_F + del_Fz_s_F + Fz_F_sc_CW;
        m_lat_load_transfer_R = del_Fz_us_R + del_Fz_s_R + Fz_R_sc_CW;
    }
}

// // usage example
// int main() {
//   double a_lat, a_lon;

//   TireManager tire_load_manager;

//   std::vector<double> tire_load;
//   tire_load = tire_load_manager.ComputeLoadTransfer(a_lon, a_lat);

//   return 0;
// }
