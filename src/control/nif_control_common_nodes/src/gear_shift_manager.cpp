//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Calvin Chanyoung Jung

#include "nif_control_common_nodes/gear_shift_manager.h"

using namespace nif::common::vehicle_param;
using namespace std::chrono;

GearShiftManager::GearShiftManager(/* args */) {
  this->init();
}
void GearShiftManager::init() {
  this->max_gear = gear_box::MAX_GEAR_POS;
  this->min_gear = gear_box::MIN_GEAR_FOR_DOWNSHIFT_POS;
}
GearShiftManager::GearShiftManager(int maxGear, int minGear) {
  this->init(maxGear, minGear);
}
void GearShiftManager::init(int maxGear, int minGear) {
  this->max_gear = maxGear;
  this->min_gear = minGear;
}

GearShiftManager::~GearShiftManager() {}

bool GearShiftManager::isUpshiftRequire() const {
  return is_upshift_require;
}
bool GearShiftManager::isDownshiftRequire() const {
  return is_downshift_require;
}
bool GearShiftManager::isStayGearRequire() const {
  return is_stay_gear_require;
}
void GearShiftManager::setCurrentVel(double currentVel) {
  current_vel = currentVel;
}
void GearShiftManager::setDesiredVel(double desiredVel) {
  desired_vel = desiredVel;
}
void GearShiftManager::setCurrentGear(int currentGear) {
  current_gear = currentGear;
}
void GearShiftManager::setCurrentRpm(int currentRpm) {
  current_rpm = currentRpm;
}
int GearShiftManager::getDesiredGear(double desiredVel,
                                     double currentVel,
                                     int currentGear,
                                     int currentRpm) {
  this->setDesiredVel(desiredVel);
  this->setCurrentVel(currentVel);
  this->setCurrentGear(currentGear);
  this->setCurrentRpm(currentRpm);

  auto current_time = std::chrono::system_clock::now();
  auto duration_in_seconds =
      std::chrono::duration<double>(current_time.time_since_epoch());
  double current_timestamp_sec = duration_in_seconds.count();

  if (desiredVel > currentVel) {
    //    considering up shift (acceleration required)
    if(abs(current_timestamp_sec-last_gearshift_timestamp_sec) < 0.5 &&
        hold_gearshift_flg == true) {
      if(last_desired_gear > current_gear){
        is_upshift_require = true;
        is_downshift_require = false;
        is_stay_gear_require = false;
      } else {
        is_upshift_require = false;
        is_downshift_require = false;
        is_stay_gear_require = true;
      }
      return last_desired_gear;
    } else {
      hold_gearshift_flg = false;
      if (current_rpm >
              gear_box::safe_upshift::max_rpm_vector[current_gear] *
                  rpm_gain_for_upshift &&
          current_gear != this->max_gear) {
        //      need up shifting
        is_upshift_require = true;
        is_downshift_require = false;
        is_stay_gear_require = false;
        desired_gear = current_gear + 1;
        //      store last gearshift info
        hold_gearshift_flg = true;
        last_desired_gear = desired_gear;
        auto current_time = std::chrono::system_clock::now();
        auto duration_in_seconds =
            std::chrono::duration<double>(current_time.time_since_epoch());
        last_gearshift_timestamp_sec = duration_in_seconds.count();
      } else {
        //      keep the current gear
        is_upshift_require = false;
        is_downshift_require = false;
        is_stay_gear_require = true;
        desired_gear = current_gear;
        hold_gearshift_flg = false;
      }
    }
  } else {
    //    considering downshift (deceleration required)
    if(abs(current_timestamp_sec-last_gearshift_timestamp_sec) < 0.5 &&
       hold_gearshift_flg == true) {
      if(last_desired_gear > current_gear){
        is_upshift_require = true;
        is_downshift_require = false;
        is_stay_gear_require = false;
      } else {
        is_upshift_require = false;
        is_downshift_require = false;
        is_stay_gear_require = true;
      }
      return last_desired_gear;
    } else {
      hold_gearshift_flg = false;
      if (current_rpm <
              gear_box::safe_downshift::reference_rpm_vector[current_gear] *
                  rpm_gain_for_downshift &&
          current_gear != this->min_gear) {
        //      good to go down shifting
        is_upshift_require = false;
        is_downshift_require = true;
        is_stay_gear_require = false;
        desired_gear = current_gear - 1;
        hold_gearshift_flg = true;
        last_desired_gear = desired_gear;
        //      store last gearshift info
        auto current_time = std::chrono::system_clock::now();
        auto duration_in_seconds =
            std::chrono::duration<double>(current_time.time_since_epoch());
        last_gearshift_timestamp_sec = duration_in_seconds.count();
      } else {
        //      keep the current gear
        is_upshift_require = false;
        is_downshift_require = false;
        is_stay_gear_require = true;
        desired_gear = current_gear;
        hold_gearshift_flg = false;
      }
    }
  }

  desired_gear = std::clamp(desired_gear,
                            nif::common::vehicle_param::gear_box::MIN_GEAR_POS,
                            nif::common::vehicle_param::gear_box::MAX_GEAR_POS);

  return desired_gear;
}
/**
 * It must be lower or equal to 1.0 (percentage)
 * @param rpmGainForUpshift
 */
void GearShiftManager::setRpmGainForUpshift(double rpmGainForUpshift) {
  rpm_gain_for_upshift = rpmGainForUpshift;
}

/**
 * It must be greater or equal to 1.0 (percentage)
 * @param rpmGainForDownshift
 */
void GearShiftManager::setRpmGainForDownshift(double rpmGainForDownshift) {
  rpm_gain_for_downshift = rpmGainForDownshift;
}
int GearShiftManager::getMaxGear() const {
  return max_gear;
}
void GearShiftManager::setMaxGear(int maxGear) {
  max_gear = maxGear;
}
int GearShiftManager::getMinGear() const {
  return min_gear;
}
void GearShiftManager::setMinGear(int minGear) {
  min_gear = minGear;
}
