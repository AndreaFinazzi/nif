//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/26/21.
//

#ifndef ROS2MASTER_VEHICLE_MODEL_H
#define ROS2MASTER_VEHICLE_MODEL_H

namespace nif {
namespace common {
class VehicleModel {};
namespace vehicle_dim {
// TODO: it should be updated ASAP
/**
 * Vehicle wheel base length in meter
 */
const double VEH_WHEEL_BASE = 5.0;

/**
 * Vehicle width in meter
 */
const double VEH_WIDTH = 5.0;

/**
 * Vehicle hight length in meter
 */
const double VEH_HEIGHT = 5.0;

/**
 * Vehicle mass in kg
 */
const double VEH_MASS_KG = 5.0;

/**
 * Steering ratio between steering wheel angle and steer angle.
 */
const double STEERING_RATIO = 5.0;
} // namespace vehicle_dim
} // namespace common

} // namespace nif

#endif // ROS2MASTER_VEHICLE_MODEL_H
