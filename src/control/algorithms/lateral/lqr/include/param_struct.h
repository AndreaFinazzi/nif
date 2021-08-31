//
// Created by usrg on 8/30/21.
//

#ifndef ROS2MASTER_PARAM_STRUCT_H
#define ROS2MASTER_PARAM_STRUCT_H

struct LqrControlParam {
    // Max Steering Angle in Degrees
    double max_steering_angle_deg;

    // convert from degress to steering units (should be 1 - 1 ?)
    double steering_units_multiplier;

    // Minimum pure pursuit tracking distance
    double pure_pursuit_min_dist_m;

    // Maximimum pure pursuit tracking distance
    double pure_pursuit_max_dist_m;

    // Factor to increase the pure pursuit tracking distance as a function of
    // speed (m/s)
    double pure_pursuit_k_vel_m_ms;

    // Limit the max change in the steering signal over time
    double steering_max_ddeg_dt;
};

#endif //ROS2MASTER_PARAM_STRUCT_H
