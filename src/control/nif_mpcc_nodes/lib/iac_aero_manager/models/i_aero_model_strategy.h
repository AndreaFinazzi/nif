#pragma once

typedef struct
{
    double pose_x;
    double pose_y;
    double vel_x;
    double vel_y;
} t_veh_state;

typedef struct
{
    double pose_x;
    double pose_y;
    double vel_x;
    double vel_y;
    double vel;
} t_self_veh_state;


/* Represents a three, 3-dimensional, vectors structure applied at
    - drag_f: CoM
    - lift_front_f: center of front axle
    - lift_rear_f: center of rear axle
*/
typedef struct
{
    double drag_f[3];
    double lift_front_f[3];
    double lift_rear_f[3];
} t_aero_state;

class i_aero_model_strategy
{
private:
    /* data */
public:

    // virtual static void initialize() = 0;
    virtual t_aero_state getAeroState(const t_self_veh_state& ego,  const std::vector<t_veh_state>& others) = 0;

    virtual double getCr2Coefficient(        
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &c_coeff
        ) = 0;

    virtual double getDeltaCr2Coefficient(        
        const double &cr2_default,
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &c_coeff
        ) = 0;
};
