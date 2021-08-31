//
// Created by usrg on 8/31/21.
//

#ifndef ROS2MASTER_PARAM_STRUCT_H
#define ROS2MASTER_PARAM_STRUCT_H

struct tireDynVelProfileParam{
    double max_vel;
    double lpf_curve_f;
    double lpf_curve_dt;
    double lpf_curve_x0;
};

#endif //ROS2MASTER_PARAM_STRUCT_H
