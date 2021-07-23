//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author: Calvin Chanyoung Jung

#include "nif_control_common_nodes/gear_shift_manager.h"

using namespace nif::common::vehicle_param;
GearShiftManager::GearShiftManager(/* args */) {}

GearShiftManager::~GearShiftManager() {}

void GearShiftManager::setCurrentGear(int currentGear) {
  current_gear = currentGear;
}
bool GearShiftManager::isUpshiftRequire() const { return is_upshift_require; }
bool GearShiftManager::isDownshiftRequire() const {
  return is_downshift_require;
}
bool GearShiftManager::isHoldGearRequire() const {
  return is_hold_gear_require;
}
void GearShiftManager::setCurrentVel(double currentVel) {
  current_vel = currentVel;
}
void GearShiftManager::setDesiredVel(double desiredVel) {
  desired_vel = desiredVel;
}
int GearShiftManager::getDesiredGear(double desiredVel, double currentVel,
                                     int currentGear) {
  desired_vel = desiredVel;
  current_vel = currentVel;
  current_gear = currentGear;

  // TODO: calculate desired gear position
  if (desiredVel > currentVel) {
    //    considering up shift (acceleration required)
    if (current_rpm > gear_box::safe_upshift::max_rpm_vector[current_gear] *
                          rpm_gain_for_upshift &&
        current_gear != gear_box::MAX_GEAR_POS) {
      //      need up shifting
      is_upshift_require = true;
      is_downshift_require = false;
      is_hold_gear_require = false;
      desired_gear = current_gear + 1;
    } else {
      //      keep the current gear
      is_upshift_require = false;
      is_downshift_require = false;
      is_hold_gear_require = true;
      desired_gear = current_gear;
    }
  } else {
    //    considering downshift (deceleration required)
    if (current_rpm <
            gear_box::safe_downshift::reference_rpm_vector[current_gear] *
                rpm_gain_for_downshift &&
        current_gear != gear_box::MIN_GEAR_FOR_DOWNSHIFT_POS) {
      //      good to go down shifting
      is_upshift_require = false;
      is_downshift_require = true;
      is_hold_gear_require = false;
      desired_gear = current_gear - 1;
    } else {
      //      keep the current gear
      is_upshift_require = false;
      is_downshift_require = false;
      is_hold_gear_require = true;
      desired_gear = current_gear;
    }
  }

  desired_gear = std::clamp(desired_gear,
                            nif::common::vehicle_param::gear_box::MIN_GEAR_POS,
                            nif::common::vehicle_param::gear_box::MAX_GEAR_POS);
  return desired_gear;
}
void GearShiftManager::setCurrentRpm(int currentRpm) {
  current_rpm = currentRpm;
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
