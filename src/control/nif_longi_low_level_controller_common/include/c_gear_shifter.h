//
// Created by usrg on 9/2/21.
//

#ifndef NIF_LONGI_LOW_LEVEL_CONTROLLER_COMMON_C_GEAR_SHIFTER_H
#define NIF_LONGI_LOW_LEVEL_CONTROLLER_COMMON_C_GEAR_SHIFTER_H

#include "iostream"
#include "vector"
#include <yaml-cpp/yaml.h>
#include "nif_common/vehicle_model.h"

namespace nif {
namespace control {
namespace low_level {

// Gear State
// Contains: gear_number, downshift speed, upshift speed, pointer to previous
// gear, pointer to next gear
struct GearState {
  int gear;
  double gearRatio;
  double downshiftSpeed;
  double upshiftSpeed;

  GearState() {}
  GearState(int gear, double gearRatio, double downshiftSpeed,
            double upshiftSpeed) {
    this->gear = gear;
    this->gearRatio = gearRatio;
    this->downshiftSpeed = downshiftSpeed;
    this->upshiftSpeed = upshiftSpeed;
  }
};

class GearShifter {
private:
  int current_gear;
  double current_vel;
  double desired_vel;

  int last_gear_cmd;
  int maximum_gear_usage;
  //! param file path
  std::string config_file_path;
  std::vector<GearState> gear_table;

public:
  GearShifter(std::string &config_file_path_);
  void setMaximumGearUsage(int maximumGearUsage);
  void loadConfig(std::string &config_file_path_);
  int getGearCmd(int desired_vel, double current_vel, int current_gear);
};
} // namespace low_level
} // namespace control
} // namespace nif

#endif // NIF_LONGI_LOW_LEVEL_CONTROLLER_COMMON_C_GEAR_SHIFTER_H
