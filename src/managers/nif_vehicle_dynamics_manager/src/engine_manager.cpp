/**
 * @file   engine_manager.cpp
 * @author Hyunki Seong
 * @date   2021-09-30
 * @brief  class implementation of vehicle engine model
 * @arg    input      :
 *         output     :
 *         parameters :
 * @param  param1 double
 *
 * @note   ***IMPORTANT ASSUMPTION***
 *
 */

#include "nif_vehicle_dynamics_manager/engine_manager.hpp"

double EngineManager::forwardEngineModel(double engine_speed,
                                         double throttle_position) {
  // Forward Engine Model
  // - input: engine_speed, throttle_position
  // - output: engine_torque
  // Compute index
  int idx_engine_speed = valueToIndex(engine_speed, M_ENGINE_SPEED_MIN,
                                      M_ENGINE_SPEED_RES, M_ENGINE_SPEED_MAX);
  int idx_throttle_position =
      valueToIndex(throttle_position, M_THROTTLE_POSITION_MIN,
                   M_THROTTLE_POSITION_RES, M_THROTTLE_POSITION_MAX);
  // Return engine torque
  return m_engine_model[idx_engine_speed][idx_throttle_position];
}

double EngineManager::inverseEngineModel(double engine_speed,
                                         double desired_engine_torque) {
  // Inverse Engine Model
  // - input: engine_speed, desired_engine_torque
  // - output: desired_throttle_position (int)
  double desired_throttle_position = 0.0;

  // Compute index
  int idx_engine_speed = valueToIndex(engine_speed, M_ENGINE_SPEED_MIN,
                                      M_ENGINE_SPEED_RES, M_ENGINE_SPEED_MAX);
  // Get feasible throttle range (from 0 to 100 throttle position)
  std::vector<double> engine_torque_range = m_engine_model[idx_engine_speed];

  // Check engine torque limit
  auto max_engine_torque = std::max_element(
      engine_torque_range.begin(),
      engine_torque_range.end()); // max_element returns iterator
  // // for debug
  // std::cout << "desired_engine_torque : " << desired_engine_torque <<
  // std::endl; std::cout << "Max torque limit      : " << *max_engine_torque <<
  // std::endl; std::cout << "engine_speed          : " << engine_speed <<
  // std::endl;

  // Return max engine torque when desired_engine_torque > max_engine_torque
  if (desired_engine_torque >= *max_engine_torque) {
    int idx_max_torque_throttle =
        std::distance(engine_torque_range.begin(), max_engine_torque);
    std::max_element(engine_torque_range.begin(), engine_torque_range.end()) -
        engine_torque_range.begin();
    desired_throttle_position =
        indexToValue(idx_max_torque_throttle, M_THROTTLE_POSITION_MIN,
                     M_THROTTLE_POSITION_RES);
    // std::cout << "!!!!!!Output Max engine torque throttle, index : "
    //           << desired_throttle_position << " " << idx_max_torque_throttle
    //           << std::endl;
  }

  // Return zero throttle when desired_engine_torque == 0
  else if (desired_engine_torque <= 0) {
    desired_throttle_position = 0.0;
  }

  // Search desired_throttle whose engine torque is close to the
  // desired_engine_torque (search from zero-throttle)
  else {
    for (int i = 0; i < engine_torque_range.size(); i++) {
      bool is_cross_desired_torque =
          (desired_engine_torque - engine_torque_range[i + 1]) *
              (desired_engine_torque - engine_torque_range[i]) <=
          0;
      if (is_cross_desired_torque) {
        desired_throttle_position =
            indexToValue(i, M_THROTTLE_POSITION_MIN, M_THROTTLE_POSITION_RES);
        break; // break for getting lower throttle solution
      }
    }
  }

  return desired_throttle_position;
}

double EngineManager::computeEngineTorque(double tire_longitudinal_force,
                                          int gear_num) {
  // Compute engine torque from tire longitudinal force
  // - tire force to tire torque
  double tire_torque = WHEEL_RADIUS * tire_longitudinal_force;
  // - get final drive & gear ratio
  double final_drive_ratio = m_gear_ratios.back();
  double final_drive_efficiency = m_gear_efficiencies.back(); // last idx
  double gear_ratio = m_gear_ratios[gear_num - 1];
  double gear_efficiency = m_gear_efficiencies[gear_num - 1];
  // - tire torque to engine torque
  double gearbox_output_torque =
      final_drive_ratio * tire_torque / final_drive_efficiency;
  double engine_torque = gearbox_output_torque * gear_ratio / gear_efficiency;

  return engine_torque;
}
