//  Copyright (c) 2021 Unmanned System Research Group @ KAIST
//  Author:

//
// Created by usrg on 6/26/21.
//

#ifndef ROS2MASTER_VEHICLE_MODEL_H
#define ROS2MASTER_VEHICLE_MODEL_H

#include <vector>

namespace nif {
namespace common {
class VehicleModel {};
namespace vehicle_param {
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

/**
 * Radius of the rear tyre in [mm].
 */
const double REAR_TYRE_RADIUS = 310.0;

namespace gear_box {
/**
 * Number of gears.
 */
const int NUM_OF_GEAR = 6;

/**
 * Max gear position
 */
const int MAX_GEAR_POS = 6;

/**
 * Min gear position
 */
const int MIN_GEAR_POS = 1;

/**
 * Min gear position for down shift
 */
const int MIN_GEAR_FOR_DOWNSHIFT_POS = 2;

namespace ratio {

/**
 * Vector of fear-ratio (use current gear position as an index of this vector).
 * GEAR_RATIO_VECTOR[0] is set to zero.
 */
const std::vector<double> GEAR_RATIO_VECTOR{
    0, 2.917, 1.875, 1.381, 1.115, 0.96, 0.889};

} // namespace ratio

namespace rpm_speed_sample {
/**
 * Vector of speed at certain rpm in [kph](use current gear position as an index
 * of this SPEED_PER_GEAR_VECTOR_AT_RPM1500[0] is set to zero
 */
const std::vector<double> SPEED_PER_GEAR_VECTOR_AT_RPM_1500{
    0,
    20.0321,
    31.1646,
    42.31254,
    52.40684,
    60.86836,
    65.72961,
};

/**
 * Vector of speed at certain rpm in [kph] (use current gear position as an
 * index of this SPEED_PER_GEAR_VECTOR_AT_RPM1900[0] is set to zero
 */
const std::vector<double> SPEED_PER_GEAR_VECTOR_AT_RPM_1900{
    0,
    25.37399,
    39.47516,
    53.59389,
    66.38199,
    77.09992,
    83.25751,
};

/**
 * Vector of speed at certain rpm in [kph] (use current gear position as an
 * index of this SPEED_PER_GEAR_VECTOR_AT_RPM4000[0] is set to zero
 */
const std::vector<double> SPEED_PER_GEAR_VECTOR_AT_RPM_4000{
    0,
    53.41892205,
    83.10559766,
    112.8334508,
    139.7515656,
    162.3156204,
    175.2789602,
};

/**
 * Vector of speed at certain rpm in [kph] (use current gear position as an
 * index of this SPEED_PER_GEAR_VECTOR_AT_RPM7000[0] is set to zero
 */
const std::vector<double> SPEED_PER_GEAR_VECTOR_AT_RPM_7000{
    0,
    93.48311,
    145.4348,
    197.4585,
    244.5652,
    284.0523,
    306.7382,
};

/**
 * Vehicle speed at the first gear, 1500rpm [kph].
 */
const double SPEED_AT_FISRT_GEAR_RPM_1500 = 20.0321;
/**
 * Vehicle speed at the first gear, 7000rpm [kph].
 */
const double SPEED_AT_FISRT_GEAR_RPM_7000 = 93.48311;

/**
 * Vehicle speed at the second gear, 1500rpm [kph].
 */
const double SPEED_AT_SECOND_GEAR_RPM_1500 = 31.1646;
/**
 * Vehicle speed at the second gear, 7000rpm [kph].
 */
const double SPEED_AT_SECOND_GEAR_RPM_7000 = 145.4348;

/**
 * Vehicle speed at the third gear, 1500rpm [kph].
 */
const double SPEED_AT_third_GEAR_RPM_1500 = 42.31254;
/**
 * Vehicle speed at the third gear, 7000rpm [kph].
 */
const double SPEED_AT_third_GEAR_RPM_7000 = 197.4585;

/**
 * Vehicle speed at the fourth gear, 1500rpm [kph].
 */
const double SPEED_AT_fourth_GEAR_RPM_1500 = 52.40684;
/**
 * Vehicle speed at the fourth gear, 7000rpm [kph].
 */
const double SPEED_AT_fourth_GEAR_RPM_7000 = 244.5652;

/**
 * Vehicle speed at the fifth gear, 1500rpm [kph].
 */
const double SPEED_AT_fifth_GEAR_RPM_1500 = 60.86836;
/**
 * Vehicle speed at the fifth gear, 7000rpm [kph].
 */
const double SPEED_AT_fifth_GEAR_RPM_7000 = 284.0523;

/**
 * Vehicle speed at the sixth gear, 1500rpm [kph].
 */
const double SPEED_AT_sixth_GEAR_RPM_1500 = 65.72961;
/**
 * Vehicle speed at the sixth gear, 7000rpm [kph].
 */
const double SPEED_AT_sixth_GEAR_RPM_7000 = 306.7382;
} // namespace rpm_speed_sample

namespace safe_downshift {
/**
 * Reference RPM vector to stay below 4000 rpm (use current gear position as an
 * index of this vector)
 * Index 0 and 1 is set to zero.
 * (ex. If current gear position is 2 and you want to downshift. Get the
 * reference rpm like this : reference_rpm_vector[2])
 * You should downshift below this rpm.
 */
const std::vector<int> reference_rpm_vector{
    0,
    0,
    2571,
    2946,
    3230,
    3444,
    3704,
};

const std::vector<double> speed_at_reference_rpm_vector{
    0,
    0,
    2571/8.55671738,
    2946/8.55671738,
    3230/8.55671738,
    3444/8.55671738,
    3704/8.55671738,
};
} // namespace safe_downshift

namespace aggressive_downshift_DONT_USE_THIS_NOW {
/**
 * Reference RPM vector to stay below 7000 rpm (use current gear position as an
 * index of this vector)
 * Index 0 and 1 is set to zero.
 * (ex. If current gear position is 2 and you want to downshift, get the
 * reference rpm like this : reference_rpm_vector[2])
 * You should downshift below this rpm.
 */
const std::vector<int> reference_rpm_vector{
    0,
    0,
    4499,
    5155,
    5651,
    6026,
    6482,
};

const std::vector<double> speed_at_reference_rpm_vector{
    0,
    0,
    4499/8.55671738,
    5155/8.55671738,
    5651/8.55671738,
    6026/8.55671738,
    6482/8.55671738,
};
} // namespace aggressive_downshift_DONT_USE_THIS_NOW

namespace safe_upshift {
/**
 * Maximum RPM vector for each gear position (use current gear position as an
 * index of this vector)
 * (ex. Let's say that current gear position is 2 and you want to upshift. Get
 * the rpm limit like this : max_rpm_vector[2])
 * You can not exeed this rpm.
 */
const std::vector<int> max_rpm_vector{
    0,
    4000,
    4000,
    4000,
    4000,
    4000,
    10000000,
};

const std::vector<double> speed_at_max_rpm_vector{
    0,
    4000/8.55671738,
    4000/8.55671738,
    4000/8.55671738,
    4000/8.55671738,
    4000/8.55671738,
    10000000/8.55671738,
};
} // namespace safe_upshift

} // namespace gear_box
} // namespace vehicle_param
} // namespace common

} // namespace nif

#endif // ROS2MASTER_VEHICLE_MODEL_H
