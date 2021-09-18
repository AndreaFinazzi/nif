/**
 * @author Hyunki Seong <hynkis@kaist.ac.kr>
 * @brief linearizes the lateral & longitudinal dynamic bicycle model
 *  for linear feedback control.
 *  based on the Joshua Spisak <jspisak@andrew.cmu.edu>'s code
 **/
#ifndef JOINT_DYNAMICS_H_
#define JOINT_DYNAMICS_H_

#include <eigen3/Eigen/Core>

namespace joint_lqr {
namespace lqr {

class JointDynamics {
public:
  /* Model Parameters */
  struct ModelParams {
    //! N/rad
    double cornering_stiffness_front;
    //! N/rad
    double cornering_stiffness_rear;
    //! kg
    double mass;
    //! m
    double length_front;
    //! m
    double length_rear;
    //! kg-m^2
    double yaw_moment_inertia;
    //! N/(m/s)^2
    double drag_coeff;
  };

  //! jacobian of the lateral longi. dynamics wrt state
  typedef Eigen::Matrix<double, 5, 5> AMatrix;
  //! jacobian of the lateral longi. dynamics wrt control input
  typedef Eigen::Matrix<double, 5, 2> BMatrix;

  //! Quadratic Cost matrices
  //! (typdef'd here because their dimensions are a function
  //!     of the state / control dimentions)
  typedef Eigen::Matrix<double, 5, 5> QMatrix;
  typedef Eigen::Matrix<double, 2, 2> RMatrix;

  /**
   * @brief generate lateral longi. dynamics with certain model parameters
   **/
  JointDynamics(ModelParams params);

  /**
   * @brief linearized the dynamics at a given velocity
   * @param velocity the velocity to linearize at
   * @return A and B matrices (jacobians wrt state / control)
   * @note The linearization is typically a function of the full
   *  state and control; however, for the lateral longi. dynamic bicycle
   *  model it is simply a function of longitudinal velocity (it's
   *  otherwise linear already)
   **/
  std::pair<AMatrix, BMatrix> linearize(double velocity);

private:
  //! Parameters to use in the linearization
  ModelParams params_;

}; /* class JointDynamics */

} /* namespace lqr */
} /* namespace joint_lqr */

#endif /* JOINT_DYNAMICS_H_ */
