#ifndef QUARTIC_POLYNOMIAL_H_
#define QUARTIC_POLYNOMIAL_H_

#include <iomanip>
#include <memory>
#include <vector>

#include "opencv2/opencv.hpp"

class QuarticPolynomial {
public:
  QuarticPolynomial(double current_position, double current_velocity,
                    double current_acceleration, double expected_velocity,
                    double expected_acceleration, double time);
  QuarticPolynomial() {}

  double calculate_zeroth_derivative(double time);
  double calculate_first_derivative(double time);
  double calculate_second_derivative(double time);
  double calculate_third_derivative(double time);

private:
  // f(x) = a4x^4 + a3x^3 + a2x^2 + a1x + a0
  // coefficient_a0_: a0
  // coefficient_a1_: a1
  // coefficient_a2_: a2
  // coefficient_a3_: a3
  // coefficient_a4_: a4
  double coefficient_a0_;
  double coefficient_a1_;
  double coefficient_a2_;
  double coefficient_a3_;
  double coefficient_a4_;
};

#endif // QUARTIC_POLYNOMIAL_H_