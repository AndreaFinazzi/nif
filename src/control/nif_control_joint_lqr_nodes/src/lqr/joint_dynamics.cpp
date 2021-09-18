#include "nif_control_joint_lqr_nodes/lqr/joint_dynamics.h"
#include <cmath>

namespace joint_lqr {
namespace lqr {

JointDynamics::JointDynamics(JointDynamics::ModelParams params)
    : params_(params) {}

std::pair<JointDynamics::AMatrix, JointDynamics::BMatrix>
JointDynamics::linearize(double longitudinal_velocity) {
  std::pair<AMatrix, BMatrix> result;

  // A Matrix (df/dx)
  result.first <<
      // First Row (df_1/dx)
      0.,
      1., 0., 0., 0.,
      // Second Row (df_2/dx)
      0.,
      (-2. * params_.cornering_stiffness_front -
       2. * params_.cornering_stiffness_rear) /
          (params_.mass * longitudinal_velocity),
      (2. * params_.cornering_stiffness_front +
       2. * params_.cornering_stiffness_rear) /
          params_.mass,
      (-2. * params_.cornering_stiffness_front * params_.length_front +
       2. * params_.cornering_stiffness_rear * params_.length_rear) /
          (params_.mass * longitudinal_velocity),
      0.,
      // Third Row (df_3/dx)
      0., 0., 0., 1., 0.,
      // Fourth Row (df_4/dx)
      0.,
      (-2. * params_.cornering_stiffness_front * params_.length_front +
       2. * params_.cornering_stiffness_rear * params_.length_rear) /
          (params_.yaw_moment_inertia * longitudinal_velocity),
      (2. * params_.cornering_stiffness_front * params_.length_front -
       2. * params_.cornering_stiffness_rear * params_.length_rear) /
          (params_.yaw_moment_inertia),
      (-2. * params_.cornering_stiffness_front *
           std::pow(params_.length_front, 2) -
       2. * params_.cornering_stiffness_rear *
           std::pow(params_.length_rear, 2)) /
          (params_.yaw_moment_inertia * longitudinal_velocity),
      0.,
      // Fifth Row (df_5/dx)
      0., 0., 0., 0.,
      -2. / params_.mass * params_.drag_coeff * longitudinal_velocity;

  // B Matrix (df/du)
  result.second << 0., 0.,
      2. * params_.cornering_stiffness_front / params_.mass, 0., 0., 0.,
      2. * params_.cornering_stiffness_front * params_.length_front /
          params_.yaw_moment_inertia,
      0., 0., 1.;

  return result;
} /* linearize() */

} /* namespace lqr */
} /* namespace joint_lqr */