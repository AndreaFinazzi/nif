#include "nif_control_lqr_nodes/lqr/solver.h"

namespace bvs_control {
namespace lqr {

/**
 * @author Horibe Takamasa
 * thanks https://github.com/TakaHoribe/Riccati_Solver
 **/
void care_ap(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    Eigen::MatrixXd &P
) {
    const uint dim_x = A.rows();

    // set Hamilton matrix
    Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

    // calc eigenvalues and eigenvectors
    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

    // extract stable eigenvectors into 'eigvec'
    Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
    unsigned int j = 0;
    for (unsigned int i = 0; i < 2 * dim_x; ++i) {
        if (Eigs.eigenvalues()[i].real() < 0.) {
            eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
            ++j;
        }
    }

    // calc P with stable eigen vector matrix
    Eigen::MatrixXcd Vs_1, Vs_2;
    Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
    Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
    P = (Vs_2 * Vs_1.inverse()).real();
} /* care_ap() */

void lqr(
    const Eigen::MatrixXd &A,
    const Eigen::MatrixXd &B,
    const Eigen::MatrixXd &Q,
    const Eigen::MatrixXd &R,
    Eigen::MatrixXd &K
) {
    // Solve the riccati equation
    Eigen::MatrixXd P;
    care_ap(A, B, Q, R, P);

    // Generate the feedback matrix
    K = R.inverse() * (B.transpose() * P);
}

} /* namespace lqr */
} /* namespace bvs_control */