#ifndef TIRE_MANAGER_H
#define TIRE_MANAGER_H

#include <cmath>
// for reading csv
#include <fstream>
#include <iostream>
#include <sstream>
// for tuple output functions
#include <tuple>
#include <vector>

// for "getPath" of this packages
#include "ament_index_cpp/get_package_prefix.hpp"

// ROS2
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"

class TireManager {
private:
  // Helper functions - load tire data
  std::vector<std::vector<double>>
  getTireFzFyDataFromCSV(std::string file_path) {
    // Debug file path
    std::cout << "Loaded tire path : " << file_path << std::endl;

    // Load Tire data (fz, FyMax, FyMin). (N,3) array
    std::vector<std::vector<double>> fzFy;
    // - read from file
    std::ifstream file(file_path);
    std::string line, cell;  // for reading lines, cells
    std::vector<double> row; // for (fz, FyMax, FyMin)
    // - loop for reading each row (line) of csv
    while (file) {
      std::getline(file, line);
      std::istringstream lineStream(line);
      row.clear();
      // - read a row (Fz, FyMax, FyMin) in csv
      while (std::getline(lineStream, cell, ',')) {
        row.push_back(stod(cell));
        // std::cout << "cell : " << cell << std::endl;
      }
      // - add in fzFy
      if (!row.empty())
        fzFy.push_back(row);
    }

    return fzFy;
  }

public:
  // Functions
  bool CalcDynamicsFeasibility(nav_msgs::msg::Path path, double vx, double ax,
                               double yaw_rate, double current_steer,
                               double dt);
  double ComputeLateralAccelLimit(double a_lon, double a_lat, double yaw_rate,
                                  double current_steer,
                                  double current_velocity);
  std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>
  Compute4WheelLateralForceLimit(double a_lon, double a_lat);
  std::tuple<double, double>
  ComputeLateralForceLimit(double Fz,
                           std::vector<std::vector<double>> table_FyMaxMin);
  double LinearInterpolation2D(double input_data,
                               std::vector<double> input_table,
                               std::vector<double> output_table);
  std::vector<double> ComputeLoadTransfer(double a_lon, double a_lat);
  void ComputeLongiTransfer(double a_lon);
  void ComputeLateralTransfer(double a_lat);
  // Helper functions
  void getCurvatureArray(nav_msgs::msg::Path path,
                         std::vector<double> &curvature_array);
  double computeCurvature(geometry_msgs::msg::PoseStamped pose_i_prev,
                          geometry_msgs::msg::PoseStamped pose_i,
                          geometry_msgs::msg::PoseStamped pose_i_next);

  // Load Tire data
  std::string pkg_install_dir =
      ament_index_cpp::get_package_prefix("nif_vehicle_dynamics_manager");
  std::string m_tire_fz_fy_LF_path =
      pkg_install_dir + "/data/tire_fz_fy/Fz_FyMax_FyMin_LF.csv";
  std::string m_tire_fz_fy_RF_path =
      pkg_install_dir + "/data/tire_fz_fy/Fz_FyMax_FyMin_RF.csv";
  std::string m_tire_fz_fy_LR_path =
      pkg_install_dir + "/data/tire_fz_fy/Fz_FyMax_FyMin_LR.csv";
  std::string m_tire_fz_fy_RR_path =
      pkg_install_dir + "/data/tire_fz_fy/Fz_FyMax_FyMin_RR.csv";

  std::vector<std::vector<double>> m_tire_fz_fy_LF =
      getTireFzFyDataFromCSV(m_tire_fz_fy_LF_path); // [Fz, FyMax, FyMin]
  std::vector<std::vector<double>> m_tire_fz_fy_RF =
      getTireFzFyDataFromCSV(m_tire_fz_fy_RF_path);
  std::vector<std::vector<double>> m_tire_fz_fy_LR =
      getTireFzFyDataFromCSV(m_tire_fz_fy_LR_path);
  std::vector<std::vector<double>> m_tire_fz_fy_RR =
      getTireFzFyDataFromCSV(m_tire_fz_fy_RR_path);

  // Parameters
  double g = 9.80665;
  // Safety factor (aggressiveness; belief w.r.t. model parameters)
  double gamma = 0.4;
  // Vehicle Parameters (geometric)
  double m_uns_f = 29.5;                    // unsprung mass front [kg]
  double m_uns_r = 36;                      // unsprung mass rear [kg]
  double m_s = 630;                         // sprung mass (base mass) [kg]
  double m_total = m_s + m_uns_f + m_uns_r; // total mass [kg]
  double r_wheel = 0.1905;                  // wheel radius [m]
  double T_f = 1.638762;                    // track length of front axle [m]
  double T_r = 1.523969;                    // track length of rear axle [m]
  double L = 2.9718;                        // wheelbase[m]
  double lf = L * 0.58;                     // length from front axle to COG [m]
  double lr = L * 0.42;                     // length from COG to rear axle [m]
  double h_ms = 0.275;   // height of sprung mass (COG point) [m] 275 mm.
  double h_rcf = 0.0476; // height of roll axis at front axie.
                         // assume (r_wheel / 4). [m] 47.6 mm.
  double h_rcr = 0.0576; // height of roll axis at rear axie.
                         // assume (h_rcf + 10 mm). [m] 57.6 mm.
  double steer_ratio =
      9.0; // steering wheel ratio (steer wheel angle : tire angle)

  double CURVATURE_MINIMUM = 0.000001;

  // Vehicle Parameters (spring)
  // - Wheel Center rate (Hard car. (>200 N/mm) (P93, "Race car design")
  double K_W_LF = 297.716; // spring stiffness front left. [N/mm]. 1800 lbs/in.
  double K_W_RF = 315.228; // spring stiffness front right. [N/mm]. 1800 lbs/in.
  double K_W_LR = 280.203; // spring stiffness rear left. [N/mm]. 1800 lbs/in.
  double K_W_RR = 280.203; // spring stiffness rear right. [N/mm]. 1800 lbs/in.
  // - Tire Vertical Stiffness (P95, "Race car design")
  double K_T_LF = 309.5; // from "EY124H.tir". [N/mm]. 309500 N/m.
  double K_T_RF = 419.3; // from "EY116H.tir". [N/mm]. 419300 N/m.
  double K_T_LR = 358.8; // from "EY125H.tir". [N/mm]. 358800 N/m.
  double K_T_RR = 502.4; // from "EY117H.tir". [N/mm]. 502400 N/m.
  // - Combined Stiffness (Ride Rate, K_R)
  double K_R_LF = (K_W_LF * K_T_LF) / (K_W_LF + K_T_LF);
  double K_R_RF = (K_W_RF * K_T_RF) / (K_W_RF + K_T_RF);
  double K_R_LR = (K_W_LR * K_T_LR) / (K_W_LR + K_T_LR);
  double K_R_RR = (K_W_RR * K_T_RR) / (K_W_RR + K_T_RR);

  // Static wheel load
  double static_wheel_load_R = 0.5 * g * (m_uns_r + m_s * lf / L);
  double static_wheel_load_F =
      0.5 * g * (m_uns_f + m_uns_r + m_s) - static_wheel_load_R;

  // Variables
  double m_lon_load_transfer;
  double m_lat_load_transfer_F;
  double m_lat_load_transfer_R;
};
#endif