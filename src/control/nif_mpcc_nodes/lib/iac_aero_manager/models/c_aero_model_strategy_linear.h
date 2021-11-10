#pragma once

#include <vector>
#include <cmath>
#include "i_aero_model_strategy.h"
#include "../c_csv_tools.h"

#define CSV_PATH_DRAG "/home/usrg/ros2_ws/src/mpcc_ros2/lib/iac_aero_manager/assets/a_aero_model_linear_drag.csv"
#define CSV_PATH_LIFT_FRONT "/home/usrg/ros2_ws/src/mpcc_ros2/lib/iac_aero_manager/assets/a_aero_model_linear_lift_front.csv"
#define CSV_PATH_LIFT_REAR "/home/usrg/ros2_ws/src/mpcc_ros2/lib/iac_aero_manager/assets/a_aero_model_linear_lift_rear.csv"

#define CSV_DIM_ROW 6
#define CSV_DIM_COL 24

// Step [m] between two adjacent cells of the matrix
#define CSV_GRANULARITY_X 3.0           // Cell's width
#define CSV_GRANULARITY_Y 0.5           // Cell's height
#define CSV_MIN_X 4
#define CSV_MIN_Y 0

// Aerodynamics coefficients
#define AERO_COEF_DRAG 0.725            // Cd
#define AERO_COEF_LIFT_FRONT 0.725      // Cl1
#define AERO_COEF_LIFT_REAR 0.725       // Cl2

#define AERO_COEF_AIR_DENSITY 1.2       // r [kg/m^3]
#define AERO_COEF_CONTACT_SURFACE 1.247 // A [m^2]
#define AERO_COEF_V_MAX 83.3333         // Vmax [m/s]


class c_aero_model_strategy_linear : public i_aero_model_strategy
{

private:
    /*  Beta coefficent matrix:  
                        X
               4    7   10  13  16  19  22  25  ... 73         
        _________________________________________________
        | 0  |
        | 0.5|
    Y   | 1  |
        | 1.5|
        | 2  |
        | 2.5|
     */
    double **drag_beta_matrix;
    double **lift_front_beta_matrix;
    double **lift_rear_beta_matrix;


    const double activation_max_x = CSV_MIN_X + CSV_DIM_COL * CSV_GRANULARITY_X,
                 activation_max_y = CSV_MIN_Y + CSV_DIM_ROW * CSV_GRANULARITY_Y;

    // Read from CSV and load in *_beta_matrix
    void initBetaMatrices();

    double getForce(const double beta, const double ego_v, double Clift);
    double getCr2(const double beta, const double ego_v, double c_coeff);


public:

    c_aero_model_strategy_linear();
    ~c_aero_model_strategy_linear();

    t_aero_state getAeroState(
        const t_self_veh_state &ego,
        const std::vector<t_veh_state> &others) override;
  
    double getCr2Coefficient(        
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &c_coeff) override;
        
    double getDeltaCr2Coefficient(
        const double &cr2_default,
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &c_coeff
    ) override;
};