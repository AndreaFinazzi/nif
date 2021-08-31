#include "lqr/lateral_dynamics.h"
#include <cmath>

namespace nif {
    namespace control {
        namespace lateral {
            namespace lqr {

                LateralDynamics::LateralDynamics(LateralDynamics::ModelParams params) :
                        params_(params) {}

                std::pair<LateralDynamics::AMatrix, LateralDynamics::BMatrix>
                LateralDynamics::linearize(double longitudinal_velocity) {
                    std::pair<AMatrix, BMatrix> result;

                    // A Matrix (df/dx)
                    result.first << // First Row (df_1/dx)
                                 0., 1., 0., 0.,
                            // Second Row (df_2/dx)
                            0.,
                            (-2. * params_.cornering_stiffness_front - 2. * params_.cornering_stiffness_rear)
                            / (params_.mass * longitudinal_velocity),
                            (2. * params_.cornering_stiffness_front + 2. * params_.cornering_stiffness_rear)
                            / params_.mass,
                            (-2. * params_.cornering_stiffness_front * params_.length_front
                             + 2. * params_.cornering_stiffness_rear * params_.length_rear)
                            / (params_.mass * longitudinal_velocity),
                            // Third Row (df_3/dx)
                            0., 0., 0., 1.,
                            // Fourth Row (df_4/dx)
                            0.,
                            (-2. * params_.cornering_stiffness_front * params_.length_front
                             + 2. * params_.cornering_stiffness_rear * params_.length_rear)
                            / (params_.yaw_moment_inertia * longitudinal_velocity),
                            (2. * params_.cornering_stiffness_front * params_.length_front
                             - 2. * params_.cornering_stiffness_rear * params_.length_rear)
                            / (params_.yaw_moment_inertia),
                            (-2. * params_.cornering_stiffness_front * std::pow(params_.length_front, 2)
                             - 2. * params_.cornering_stiffness_rear * std::pow(params_.length_rear, 2))
                            / (params_.yaw_moment_inertia * longitudinal_velocity);

                    // B Matrix (df/du)
                    result.second << 0.,
                            2. * params_.cornering_stiffness_front / params_.mass,
                            0.,
                            2. * params_.cornering_stiffness_front * params_.length_front / params_.yaw_moment_inertia;

                    return result;
                } /* linearize() */
            } /* namespace lqr */
        } /* namespace bvs_control */
    }
}