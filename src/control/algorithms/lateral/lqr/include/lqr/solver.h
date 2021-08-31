/**
 * @author Joshua Spisak <jspisak@andrew.cmu.edu>
 * @brief functions for solving optimal control with lqr
 *
 * @note thanks to Horibe Takamasa
 **/
#ifndef LQR_CONTROL_LQR_SOLVER_H_
#define LQR_CONTROL_LQR_SOLVER_H_

#include <eigen3/Eigen/Dense>
#include <iostream>
#include <time.h>
#include <vector>

namespace nif {
    namespace control {
        namespace lateral {
            namespace lqr {
/**
 * @brief arimoto potter method to solve continiuous ricatti equations
 * @param[in] A model jacobian wrt the state
 * @param[in] B model jacobian wrt the control
 * @param[in] Q quadratic cost matrix for state error
 * @param[in] R quadratic cost matrix for control input
 * @param[out] P ricatti equation solution
 **/
                void care_ap(
                        const Eigen::MatrixXd &A,
                        const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &Q,
                        const Eigen::MatrixXd &R,
                        Eigen::MatrixXd &P
                );

/**
 * @brief lqr problem solver
 * @param[in] A model jacobian wrt the state
 * @param[in] B model jacobian wrt the control
 * @param[in] Q quadratic cost matrix for state error
 * @param[in] R quadratic cost matrix for control input
 * @param[out] K optimal feedback matrix
 **/
                void lqr(
                        const Eigen::MatrixXd &A,
                        const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &Q,
                        const Eigen::MatrixXd &R,
                        Eigen::MatrixXd &K
                );

            } /* namespace lqr */
        } /* namespace bvs_control */
    }
}

#endif /* LQR_CONTROL_LQR_SOLVER_H_ */