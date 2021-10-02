#ifndef ENGINE_MANAGER_H
#define ENGINE_MANAGER_H

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

// for vector calculation
#include <algorithm>

class EngineManager {
private:
  // Helper functions
  // - load engine map data
  std::vector<std::vector<double>>
  getEngineModelDataFromCSV(std::string file_path) {
    // Debug file path
    std::cout << "Loaded engine model path : " << file_path << std::endl;

    // Load Engine model data
    // - (engine_speed, throttle): (123,101)
    // - line: engine_speed axis / row: throttle axis
    // - engine_speed_min = 900
    // - engine_speed_res = 50
    // - engine_speed_max = 7000
    // - throttle_min = 0
    // - throttle_res = 1
    // - throttle_max = 100

    std::vector<std::vector<double>> engineTorque;
    // - read from file
    std::ifstream file(file_path);
    std::string line, cell;  // for reading lines, cells
    std::vector<double> row; // for torque values w.r.t. throttle(0:1:100)
    // - loop for reading each row (line) of csv
    while (file) {
      std::getline(file, line);
      std::istringstream lineStream(line);
      row.clear();
      // - read a row (torque w.r.t. throttle 0:1:100) in csv
      while (std::getline(lineStream, cell, ',')) {
        row.push_back(stod(cell));
        // std::cout << "cell : " << cell << std::endl;
      }
      // - add in engineTorque
      if (!row.empty())
        engineTorque.push_back(row);
    }

    return engineTorque;
  }

  int valueToIndex(double input_data, double data_min, double data_res,
                   double data_max) {
    // Compute index of the table w.r.t. input data
    int idx_data = (int)((input_data - data_min) / data_res);
    int data_length = (int)((data_max - data_min) / data_res);
    // Saturate index value
    if (input_data < data_min)
      idx_data = 0;
    else if (input_data > data_max)
      idx_data = data_length - 1;

    return idx_data;
  }
  double indexToValue(int idx_data, double data_min, double data_res) {
    return data_min + data_res * idx_data;
  }

public:
  // Functions
  double forwardEngineModel(double engine_speed, double throttle_position);
  double inverseEngineModel(double engine_speed, double desired_engine_torque);
  double computeEngineTorque(double tire_longitudinal_force, int gear_num);

  // Load data
  std::string pkg_install_dir =
      ament_index_cpp::get_package_prefix("vehicle_dynamics_manager");

  // - engine model data
  std::string m_engine_model_path =
      pkg_install_dir +
      "/data/engine_model/engine_map_extrapolated_with_data_210930_132133.csv";
  std::vector<std::vector<double>> m_engine_model =
      getEngineModelDataFromCSV(m_engine_model_path); // [Fz, FxMax, FxMin]
  // - gear model
  std::vector<double> m_gear_ratios = {
      1 / 2.917, 1 / 1.875, 1 / 1.381, 1 / 1.115,
      1 / 0.96,  1 / 0.889, 1 / 3.}; // 1st, 2nd, 3rd, 4th, 5th, 6th, Drive
  std::vector<double> m_gear_efficiencies = {
      0.91, 0.91, 0.91, 0.96,
      0.96, 0.96, 0.98}; // 1st, 2nd, 3rd, 4th, 5th, 6th, Drive
  // 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; // 1st, 2nd, 3rd, 4th, 5th, 6th, Drive

  // Parameters
  double g = 9.80665;
  // Safety factor (aggressiveness; belief w.r.t. model parameters)
  double m_gamma = 0.4;
  int m_rpm_safe_thres = 3000;
  // Vehicle Parameters (geometric)
  double MASS_UNSPRUNG_FRONT = 29.5; // unsprung mass front [kg]
  double MASS_UNSPRUNG_REAR = 36;    // unsprung mass rear [kg]
  double MASS_SPRUNG = 630;          // sprung mass (base mass) [kg]
  double MASS_TOTAL = MASS_SPRUNG + 2 * (MASS_UNSPRUNG_FRONT +
                                         MASS_UNSPRUNG_REAR); // total mass [kg]
  double WHEEL_RADIUS = 0.31; // wheel radius [m]

  double scale_wheelspeed = 1.0177; // scale for rear wheel speed
  double scale_slip_ratio = 0.2323; // scale for slip ratio using model

  // Lookup table Params
  int M_ENGINE_SPEED_MIN = 900;
  int M_ENGINE_SPEED_RES = 50;
  int M_ENGINE_SPEED_MAX = 7000;

  int M_THROTTLE_POSITION_MIN = 0;
  int M_THROTTLE_POSITION_RES = 1;
  int M_THROTTLE_POSITION_MAX = 100;

  // Variables
  double m_lon_load_transfer;
  double m_lat_load_transfer_F;
  double m_lat_load_transfer_R;
};
#endif