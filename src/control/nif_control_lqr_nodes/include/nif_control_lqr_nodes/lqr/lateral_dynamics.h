/**
 * @author Joshua Spisak <jspisak@andrew.cmu.edu>
 * @brief linearizes the lateral dynamic bicycle model
 *  for linear feedback control
 **/
#ifndef BVS_CONTROL_LQR_LATERAL_DYNAMICS_H_
#define BVS_CONTROL_LQR_LATERAL_DYNAMICS_H_

#include <eigen3/Eigen/Core>

namespace bvs_control {
namespace lqr {

class LateralDynamics {
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
    };

    //! jacobian of the lateral dynamics wrt state
    typedef Eigen::Matrix<double, 4, 4> AMatrix;
    //! jacobian of the lateral dynamics wrt control input
    typedef Eigen::Matrix<double, 4, 1> BMatrix;

    //! Quadratic Cost matrices
    //! (typdef'd here because their dimensions are a function
    //!     of the state / control dimentions)
    typedef Eigen::Matrix<double, 4, 4> QMatrix;
    typedef Eigen::Matrix<double, 1, 1> RMatrix;

    /**
     * @brief generate lateral dynamics with certain model parameters
     **/
    LateralDynamics(ModelParams params);

    /**
     * @brief linearized the dynamics at a given velocity
     * @param velocity the velocity to linearize at
     * @return A and B matrices (jacobians wrt state / control)
     * @note The linearization is typically a function of the full
     *  state and control; however, for the lateral dynamic bicycle
     *  model it is simply a function of longitudinal velocity (it's
     *  otherwise linear already)
     **/
    std::pair<AMatrix, BMatrix> linearize(double velocity);

private:
    //! Parameters to use in the linearization
    ModelParams params_;

}; /* class LateralDynamics */

} /* namespace lqr */
} /* namespace bvs_control */

#endif /* BVS_CONTROL_LQR_LATERAL_DYNAMICS_H_ */
