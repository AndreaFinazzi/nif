//
// Created by usrg on 9/2/21.
//

#include "c_gear_shifter.h"

using namespace nif::control::low_level;

GearShifter::GearShifter(std::string &config_file_path_)
    : config_file_path(config_file_path_),
      maximum_gear_usage(nif::common::vehicle_param::gear_box::NUM_OF_GEAR) {
  this->gear_table.resize(nif::common::vehicle_param::gear_box::NUM_OF_GEAR);
  this->loadConfig(config_file_path_);
}

void GearShifter::loadConfig(std::string &config_file_path_) {
  YAML::Node config = YAML::LoadFile(config_file_path_);

  if (!config["Maximum_gear_usage"]) {
    std::cout
        << "Maximum gear usage param is not setup properly. Set to defualt, 6."
        << std::endl;
    this->maximum_gear_usage =
        nif::common::vehicle_param::gear_box::NUM_OF_GEAR;
  } else {
    this->maximum_gear_usage = config["Maximum_gear_usage"].as<double>();
  }
  if (!config["First_gear_params"] || !config["Second_gear_params"] ||
      !config["Third_gear_params"] || !config["Fourth_gear_params"] ||
      !config["Fifth_gear_params"] || !config["Sixth_gear_params"]) {
    throw std::runtime_error(
        "Longi low level controller : Gear shifting params not "
        "fully defined in config file.");
  }

  if (config["First_gear_params"]) {
    YAML::Node first_gear_params = config["First_gear_params"];
    if (!first_gear_params["downshifting_vel_mps"] ||
        !first_gear_params["upshifting_vel_mps"]) {
      throw std::runtime_error(
          "Longi low level controller : First gear shifting params not "
          "fully defined in config file.");
    } else if (first_gear_params["downshifting_vel_mps"].as<double>() >
               first_gear_params["upshifting_vel_mps"].as<double>()) {
      throw std::runtime_error(
          "Longi low level controller : First gear downshifting velocity is "
          "lower than upshifting velocity.");
    }
    int gear_pos = 1;
    nif::control::low_level::GearState first_gear_state(
        gear_pos,
        nif::common::vehicle_param::gear_box::ratio::GEAR_RATIO_VECTOR
            [gear_pos],
        first_gear_params["downshifting_vel_mps"].as<double>(),
        first_gear_params["upshifting_vel_mps"].as<double>());
    this->gear_table[gear_pos - 1] = first_gear_state;
  }

  if (config["Second_gear_params"]) {
    YAML::Node second_gear_params = config["Second_gear_params"];
    if (!second_gear_params["downshifting_vel_mps"] ||
        !second_gear_params["upshifting_vel_mps"]) {
      throw std::runtime_error(
          "Longi low level controller : Second gear shifting params not "
          "fully defined in config file.");
    } else if (second_gear_params["downshifting_vel_mps"].as<double>() >
               second_gear_params["upshifting_vel_mps"].as<double>()) {
      throw std::runtime_error(
          "Longi low level controller : Second gear downshifting velocity is "
          "lower than upshifting velocity.");
    }
    int gear_pos = 2;
    nif::control::low_level::GearState second_gear_state(
        gear_pos,
        nif::common::vehicle_param::gear_box::ratio::GEAR_RATIO_VECTOR
            [gear_pos],
        second_gear_params["downshifting_vel_mps"].as<double>(),
        second_gear_params["upshifting_vel_mps"].as<double>());
    this->gear_table[gear_pos - 1] = second_gear_state;
  }

  if (config["Third_gear_params"]) {
    YAML::Node third_gear_params = config["Third_gear_params"];
    if (!third_gear_params["downshifting_vel_mps"] ||
        !third_gear_params["upshifting_vel_mps"]) {
      throw std::runtime_error(
          "Longi low level controller : Third gear shifting params not "
          "fully defined in config file.");
    } else if (third_gear_params["downshifting_vel_mps"].as<double>() >
               third_gear_params["upshifting_vel_mps"].as<double>()) {
      throw std::runtime_error(
          "Longi low level controller : Third gear downshifting velocity is "
          "lower than upshifting velocity.");
    }
    int gear_pos = 3;
    nif::control::low_level::GearState third_gear_state(
        gear_pos,
        nif::common::vehicle_param::gear_box::ratio::GEAR_RATIO_VECTOR
            [gear_pos],
        third_gear_params["downshifting_vel_mps"].as<double>(),
        third_gear_params["upshifting_vel_mps"].as<double>());
    this->gear_table[gear_pos - 1] = third_gear_state;
  }

  if (config["Fourth_gear_params"]) {
    YAML::Node fourth_gear_params = config["Fourth_gear_params"];
    if (!fourth_gear_params["downshifting_vel_mps"] ||
        !fourth_gear_params["upshifting_vel_mps"]) {
      throw std::runtime_error(
          "Longi low level controller : Fourth gear shifting params not "
          "fully defined in config file.");
    } else if (fourth_gear_params["downshifting_vel_mps"].as<double>() >
               fourth_gear_params["upshifting_vel_mps"].as<double>()) {
      throw std::runtime_error(
          "Longi low level controller : Fourth gear downshifting velocity is "
          "lower than upshifting velocity.");
    }
    int gear_pos = 4;
    nif::control::low_level::GearState fourth_gear_state(
        gear_pos,
        nif::common::vehicle_param::gear_box::ratio::GEAR_RATIO_VECTOR
            [gear_pos],
        fourth_gear_params["downshifting_vel_mps"].as<double>(),
        fourth_gear_params["upshifting_vel_mps"].as<double>());
    this->gear_table[gear_pos - 1] = fourth_gear_state;
  }

  if (config["Fifth_gear_params"]) {
    YAML::Node fifth_gear_params = config["Fifth_gear_params"];
    if (!fifth_gear_params["downshifting_vel_mps"] ||
        !fifth_gear_params["upshifting_vel_mps"]) {
      throw std::runtime_error(
          "Longi low level controller : Fifth gear shifting params not "
          "fully defined in config file.");
    } else if (fifth_gear_params["downshifting_vel_mps"].as<double>() >
               fifth_gear_params["upshifting_vel_mps"].as<double>()) {
      throw std::runtime_error(
          "Longi low level controller : Fifth gear downshifting velocity is "
          "lower than upshifting velocity.");
    }
    int gear_pos = 5;
    nif::control::low_level::GearState fifth_gear_state(
        gear_pos,
        nif::common::vehicle_param::gear_box::ratio::GEAR_RATIO_VECTOR
            [gear_pos],
        fifth_gear_params["downshifting_vel_mps"].as<double>(),
        fifth_gear_params["upshifting_vel_mps"].as<double>());
    this->gear_table[gear_pos - 1] = fifth_gear_state;
  }

  if (config["Sixth_gear_params"]) {
    YAML::Node sixth_gear_params = config["Sixth_gear_params"];
    if (!sixth_gear_params["downshifting_vel_mps"] ||
        !sixth_gear_params["upshifting_vel_mps"]) {
      throw std::runtime_error(
          "Longi low level controller : Sixth gear shifting params not "
          "fully defined in config file.");
    } else if (sixth_gear_params["downshifting_vel_mps"].as<double>() >
               sixth_gear_params["upshifting_vel_mps"].as<double>()) {
      throw std::runtime_error(
          "Longi low level controller : Sixth gear downshifting velocity is "
          "lower than upshifting velocity.");
    }
    int gear_pos = 6;
    nif::control::low_level::GearState sixth_gear_state(
        gear_pos,
        nif::common::vehicle_param::gear_box::ratio::GEAR_RATIO_VECTOR
            [gear_pos],
        sixth_gear_params["downshifting_vel_mps"].as<double>(),
        sixth_gear_params["upshifting_vel_mps"].as<double>());
    this->gear_table[gear_pos - 1] = sixth_gear_state;
  }
}

void GearShifter::setMaximumGearUsage(int maximumGearUsage) {
  if (maximumGearUsage > 0 &&
      maximumGearUsage < nif::common::vehicle_param::gear_box::NUM_OF_GEAR) {
    maximum_gear_usage = maximumGearUsage;
  } else if (maximumGearUsage >
             nif::common::vehicle_param::gear_box::NUM_OF_GEAR) {
    maximum_gear_usage = nif::common::vehicle_param::gear_box::NUM_OF_GEAR;
  } else {
    // maximumGearUsage < 0
    maximum_gear_usage = 1;
  }
}

int GearShifter::getGearCmd(int desired_vel, double current_vel,
                            int current_gear) {
  this->current_gear = current_gear;
  this->current_vel = current_vel;
  this->desired_vel = desired_vel;

  if (this->current_gear > nif::common::vehicle_param::gear_box::NUM_OF_GEAR) {
    std::cout << "Current gear can not higher than the maximum gear position. "
                 "Set to maximum gear, 6"
              << std::endl;
    this->current_gear = nif::common::vehicle_param::gear_box::NUM_OF_GEAR;
  } else if (this->current_gear < 1) {
    std::cout << "Current gear can not lower than the first gear. "
                 "Set to first gear."
              << std::endl;
    this->current_gear = 1;
  }

  if (this->current_vel > this->gear_table[this->current_gear].upshiftSpeed &&
      this->desired_vel > this->current_vel &&
      current_gear < this->maximum_gear_usage) {
    this->last_gear_cmd = current_gear + 1;
  } else if (this->current_vel <
             this->gear_table[this->current_gear].downshiftSpeed) {
    this->last_gear_cmd = current_gear - 1;
  } else {
    this->last_gear_cmd = this->current_gear;
  }
  return this->last_gear_cmd;
}
