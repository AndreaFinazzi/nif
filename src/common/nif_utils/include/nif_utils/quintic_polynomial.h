#ifndef QUINTIC_POLYNOMIAL_H_
#define QUINTIC_POLYNOMIAL_H_

#include <assert.h>
#include <memory>

#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"

using namespace cv;
using namespace std;

class QuinticPolynomial {
public:
  QuinticPolynomial(double current_position,
                    double current_velocity,
                    double current_acceleration,
                    double expected_position,
                    double expected_velocity,
                    double expected_acceleration,
                    double time);
  ~QuinticPolynomial() {}

  double calculate_zeroth_derivative(double time);
  double calculate_first_derivative(double time);
  double calculate_second_derivative(double time);
  double calculate_third_derivative(double time);

private:
  // f(x) = a5x^5 + a4x^4 + a3x^3 + a2x^2 + a1x + a0
  // coefficient_a0_: a0
  // coefficient_a1_: a1
  // coefficient_a2_: a2
  // coefficient_a3_: a3
  // coefficient_a4_: a4
  // coefficient_a5_: a5
  double coefficient_a0_;
  double coefficient_a1_;
  double coefficient_a2_;
  double coefficient_a3_;
  double coefficient_a4_;
  double coefficient_a5_;
};

#endif // QUINTIC_POLYNOMIAL_H_