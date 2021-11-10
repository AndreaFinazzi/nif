#ifndef C_AERO_MANAGER_H
#define C_AERO_MANAGER_H

#include <stdlib.h>
#include <iostream>
#include <memory>
#include <exception>
#include <vector>
#include <stdio.h>

#include "models/c_aero_model_strategy_linear.h"

// Aero manager designed with Strategy pattern for keeping flexibility against possible changes on the simulator side. 
// Optional i_aero_model_strategy parameter can be passed to change aerodynamic model. 
// Default is set to 'c_aero_model_strategy_linear'.
// 
class c_aero_manager
{
private:
    /* data */
    std::unique_ptr<i_aero_model_strategy> _model;

public:
    // Default strategy to c_aero_model_strategy_linear
    c_aero_manager();

    // Custom constructor enable different strategy selection
    c_aero_manager(std::unique_ptr<i_aero_model_strategy> model_strategy);

    ~c_aero_manager();

    // Returns the aerodynamic draft advantages at a specific location, based on the opponent's pose
    // t_aero_state getAeroDraftAdvantage(
    //     const double &ego_x,
    //     const double &ego_y,
    //     const double &ego_v,
    //     const t_veh_state &other);

    t_aero_state getAeroDraftAdvantage(
        const double &ego_x,
        const double &ego_y,
        const double &ego_v,
        const std::vector<t_veh_state> &others);

    double getCr2Coefficient(        
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &&c_coeff
        );

    double getDeltaCr2Coefficient(        
        const double &cr2_default,
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &&c_coeff
        );
};

#endif
