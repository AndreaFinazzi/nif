#include "nif_opponent_prediction_nodes/quartic_polynomial.h"

#include <assert.h>

QuarticPolynomial::QuarticPolynomial(double current_position,
                                     double current_velocity,
                                     double current_acceleration,
                                     double expected_velocity,
                                     double expected_acceleration,
                                     double time)
{
  coefficient_a0_ = current_position;
  coefficient_a1_ = current_velocity;
  coefficient_a2_ = current_acceleration / 2.0;

  cv::Mat matrix_A(2, 2, CV_64F, 0.0);
  cv::Mat matrix_b(2, 1, CV_64F, 0.0);

  matrix_A.at<double>(0, 0) = 3 * pow(time, 2);
  matrix_A.at<double>(0, 1) = 4 * pow(time, 3);
  matrix_A.at<double>(1, 0) = 6 * time;
  matrix_A.at<double>(1, 1) = 12 * pow(time, 2);

  matrix_b.at<double>(0, 0) =
      expected_velocity - coefficient_a1_ - 2 * coefficient_a2_ * time;
  matrix_b.at<double>(1, 0) = expected_acceleration - 2 * coefficient_a2_;

  cv::Mat matrix_x = matrix_A.inv() * matrix_b;

  coefficient_a3_ = matrix_x.at<double>(0, 0);
  coefficient_a4_ = matrix_x.at<double>(1, 0);
}

double QuarticPolynomial::calculate_zeroth_derivative(double time)
{
  return coefficient_a0_ + coefficient_a1_ * time +
         coefficient_a2_ * pow(time, 2) + coefficient_a3_ * pow(time, 3) +
         coefficient_a4_ * pow(time, 4);
}

double QuarticPolynomial::calculate_first_derivative(double time)
{
  return coefficient_a1_ + 2 * coefficient_a2_ * time +
         3 * coefficient_a3_ * pow(time, 2) +
         4 * coefficient_a4_ * pow(time, 3);
}

double QuarticPolynomial::calculate_second_derivative(double time)
{
  return 2 * coefficient_a2_ + 6 * coefficient_a3_ * time +
         12 * coefficient_a4_ * pow(time, 2);
}

double QuarticPolynomial::calculate_third_derivative(double time)
{
  return 6 * coefficient_a3_ + 24 * coefficient_a4_ * time;
}
