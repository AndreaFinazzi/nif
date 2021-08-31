//
// Created by usrg on 8/30/21.
//

#ifndef ROS2MASTER_PARAM_STRUCT_H
#define ROS2MASTER_PARAM_STRUCT_H

struct kinControlParam{
    double min_lookahead;
    double max_lookahead;
    double lookahead_speed_ratio;
    double proportional_gain;
    double max_steer_angle;
    double steering_override_threshold;
};

#endif //ROS2MASTER_PARAM_STRUCT_H
