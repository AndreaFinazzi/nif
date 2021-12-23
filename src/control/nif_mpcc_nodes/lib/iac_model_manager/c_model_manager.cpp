#include "c_model_manager.h"

c_model_manager::c_model_manager()
{
    //
}
c_model_manager::~c_model_manager()
{
    //
}

std::vector<double> c_model_manager::getModelParam(double vx, double driving_dist)
{
    // Get Model param
    // [Bf, Cf, Df, Br, Cr, Dr, Cm1, Cm1_brake]

    // model param interpolation
    std::vector<double> model_params_output = modelParamInterpolation(vx, driving_dist);

    // // friction decay ratio
    // double roh_decay = frictionDecayRatio(driving_dist);
    // model_params_output[2] *= roh_decay; // Dr
    // model_params_output[5] *= roh_decay; // Df
    // model_params_output[6] *= roh_decay; // Cm1
    // model_params_output[7] *= roh_decay; // Cm1_brake

    return model_params_output;
}

std::vector<double> c_model_manager::modelParamInterpolation(double vx, double driving_dist)
{
    // Interpolate tire model params w.r.t. current velocity and driven distance
    // [Bf, Cf, Df, Br, Cr, Dr, Cm1, Cm1_brake]

    std::vector<double> model_params(8);
    double rambda = (drivingDistMax - driving_dist) / drivingDistMax;

    if (vx < 100/3.6){
        // vx < 100 km/h
        // Interpolation w.r.t. driven_distance
        model_params[0] = rambda * m_Bf_100_08 + (1-rambda) * m_Bf_100_072;
        model_params[1] = rambda * m_Cf_100_08 + (1-rambda) * m_Cf_100_072;
        model_params[2] = rambda * m_Df_100_08 + (1-rambda) * m_Df_100_072;
        model_params[3] = rambda * m_Br_100_08 + (1-rambda) * m_Br_100_072;
        model_params[4] = rambda * m_Cr_100_08 + (1-rambda) * m_Cr_100_072;
        model_params[5] = rambda * m_Dr_100_08 + (1-rambda) * m_Dr_100_072;
        model_params[6] = rambda * m_Cm1_100_08 + (1-rambda) * m_Cm1_100_072;
        model_params[7] = rambda * m_Cm1_brake_100_08 + (1-rambda) * m_Cm1_brake_100_072;
    }
    else if (vx >= 100/3.6 && vx < 270/3.6){
        // 100 km/h <= vx < 270 km/h
        double roh = (vx - 100/3.6)/(270/3.6-100/3.6);
        
        // Interpolation w.r.t. velocity
        double param_08_0 = (1-roh) * m_Bf_100_08 + roh * m_Bf_270_08;
        double param_08_1 = (1-roh) * m_Cf_100_08 + roh * m_Cf_270_08;
        double param_08_2 = (1-roh) * m_Df_100_08 + roh * m_Df_270_08;
        double param_08_3 = (1-roh) * m_Br_100_08 + roh * m_Br_270_08;
        double param_08_4 = (1-roh) * m_Cr_100_08 + roh * m_Cr_270_08;
        double param_08_5 = (1-roh) * m_Dr_100_08 + roh * m_Dr_270_08;
        double param_08_6 = (1-roh) * m_Cm1_100_08 + roh * m_Cm1_270_08;
        double param_08_7 = (1-roh) * m_Cm1_brake_100_08 + roh * m_Cm1_brake_270_08;

        double param_072_0 = (1-roh) * m_Bf_100_072 + roh * m_Bf_270_072;
        double param_072_1 = (1-roh) * m_Cf_100_072 + roh * m_Cf_270_072;
        double param_072_2 = (1-roh) * m_Df_100_072 + roh * m_Df_270_072;
        double param_072_3 = (1-roh) * m_Br_100_072 + roh * m_Br_270_072;
        double param_072_4 = (1-roh) * m_Cr_100_072 + roh * m_Cr_270_072;
        double param_072_5 = (1-roh) * m_Dr_100_072 + roh * m_Dr_270_072;
        double param_072_6 = (1-roh) * m_Cm1_100_072 + roh * m_Cm1_270_072;
        double param_072_7 = (1-roh) * m_Cm1_brake_100_072 + roh * m_Cm1_brake_270_072;

        // Interpolation w.r.t. driven_distance
        model_params[0] = rambda * param_08_0 + (1-rambda) * param_072_0;
        model_params[1] = rambda * param_08_1 + (1-rambda) * param_072_1;
        model_params[2] = rambda * param_08_2 + (1-rambda) * param_072_2;
        model_params[3] = rambda * param_08_3 + (1-rambda) * param_072_3;
        model_params[4] = rambda * param_08_4 + (1-rambda) * param_072_4;
        model_params[5] = rambda * param_08_5 + (1-rambda) * param_072_5;
        model_params[6] = rambda * param_08_6 + (1-rambda) * param_072_6;
        model_params[7] = rambda * param_08_7 + (1-rambda) * param_072_7;
    }
    else{
        // 270 km/h <= vx
        // Interpolation w.r.t. driven_distance
        model_params[0] = rambda * m_Bf_270_08 + (1-rambda) * m_Bf_270_072;
        model_params[1] = rambda * m_Cf_270_08 + (1-rambda) * m_Cf_270_072;
        model_params[2] = rambda * m_Df_270_08 + (1-rambda) * m_Df_270_072;
        model_params[3] = rambda * m_Br_270_08 + (1-rambda) * m_Br_270_072;
        model_params[4] = rambda * m_Cr_270_08 + (1-rambda) * m_Cr_270_072;
        model_params[5] = rambda * m_Dr_270_08 + (1-rambda) * m_Dr_270_072;
        model_params[6] = rambda * m_Cm1_270_08 + (1-rambda) * m_Cm1_270_072;
        model_params[7] = rambda * m_Cm1_brake_270_08 + (1-rambda) * m_Cm1_brake_270_072;
    }

    return model_params;
}

// double c_model_manager::frictionDecayRatio(double driving_dist);
// {
//     // Calculate friction decay ratio w.r.t. driving distance
//     double decay_ratio = 1/0.8 * (-(0.8-0.7)/drivingDistMax * driving_dist + 0.8);
//     decay_ratio = max(0.7, min(0.8, decay_ratio));
//     return decay_ratio;
// }