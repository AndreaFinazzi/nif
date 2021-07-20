//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/26/21.
//

#ifndef NIF_COMMON_CONSTANTS_H
#define NIF_COMMON_CONSTANTS_H

#include <chrono>
#include <limits>
#include <rclcpp/qos.hpp>

namespace nif {
namespace common {
namespace constants {

/**
 * Maximum number of opponent on the track.
 */
const int NUMBER_OF_OPPO_MAX = 10;

/**
 * DEGREE to RADIAN.
 */
const double DEG2RAD = 0.0174533;

/**
 * RADIAN to DEGREE.
 */
const double RAD2DEG = 57.2958;

/**
 * Default period for synchronized node.
 */
const std::chrono::microseconds SYNC_PERIOD_DEFAULT(10000);

/**
 * Min period for synchronized node.
 */
const std::chrono::microseconds SYNC_PERIOD_MIN(10000);

/**
 * Max period for synchronized node.
 */
const std::chrono::microseconds SYNC_PERIOD_MAX(10000);

/**
 * Default QoS parameter.
 * TODO look into details and take proper decision
 */
const rclcpp::QoS QOS_DEFAULT(5);

/**
 * Name for the main logger.
 */
const char* const LOG_MAIN_LOGGER_NAME = "MAIN_LOGGER";

namespace numeric {
/**
 * pi is pi
 */
const float PI = 3.141592653589793;

/**
 * Infinite value (float)
 */
const float INF = std::numeric_limits<float>::max();

/**
 * Epsilon value (float)
 */
const float EPSILON = std::numeric_limits<float>::epsilon();

} // namespace numeric

namespace planner {
/**
 * pi is pi
 */
const float WPT_MINIMUM_LEN = 1.0;

} // namespace planner

} // namespace constants
} // namespace common
} // namespace nif

#endif // NIF_COMMON_CONSTANTS_H
