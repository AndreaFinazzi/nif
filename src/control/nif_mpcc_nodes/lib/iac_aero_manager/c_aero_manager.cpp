#include "c_aero_manager.h"

c_aero_manager::c_aero_manager() : _model (std::make_unique<c_aero_model_strategy_linear>()) {}

c_aero_manager::c_aero_manager(std::unique_ptr<i_aero_model_strategy> model_strategy) : _model(std::move(model_strategy)) {}

c_aero_manager::~c_aero_manager() {}

t_aero_state c_aero_manager::getAeroDraftAdvantage(
    const double &ego_x,
    const double &ego_y,
    const double &ego_v,
    const std::vector<t_veh_state> &others)
{

    t_self_veh_state ego_state = {
        ego_x,
        ego_y,
        0.0,
        0.0,
        ego_v};

    t_aero_state aero_state = _model->getAeroState(ego_state, others);

    return aero_state;
}


    double c_aero_manager::getCr2Coefficient(        
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &&c_coeff
        ) {
            return _model->getCr2Coefficient(delta_x, delta_y, ego_v, c_coeff );
        }

    double c_aero_manager::getDeltaCr2Coefficient(        
        const double &cr2_default,
        const double &delta_x,
        const double &delta_y,
        const double &ego_v,
        const double &&c_coeff
        ){
            return _model->getDeltaCr2Coefficient(cr2_default, delta_x, delta_y, ego_v, c_coeff );
        }
