#ifndef CUBIC_SPLINER_H_
#define CUBIC_SPLINER_H_

#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <memory>
#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <utility>
#include <vector>

#include "opencv4/opencv2/core.hpp"
#include "opencv4/opencv2/highgui.hpp"

class CubicSpliner {
public:
  // points is a collection of points of certain coordinate, where point is a
  // single value of certain coordinate

  CubicSpliner(std::vector<double>& points_x, std::vector<double>& points_y);
  ~CubicSpliner() {}

  int number_of_points() {
    return points_x_.size();
  }
  double calculate_zeroth_derivative(double time);
  double calculate_first_derivative(double time);
  double calculate_second_derivative(double time);

protected:
  int find_index(double point_x);

  std::shared_ptr<cv::Mat> calculate_matrix_A(std::vector<double>& delta_x);
  std::shared_ptr<cv::Mat> calculate_matrix_B(std::vector<double>& delta_x);

private:
  std::vector<double> points_x_;
  std::vector<double> points_y_;

  // Cubic function f(x) = ax^3 + bx^2 + cx + d
  // coefficient_a_ : a
  // coefficient_b_ : b
  // coefficient_c_ : c
  // coefficient_d_ : d
  std::vector<double> coefficient_a_;
  std::vector<double> coefficient_b_;
  std::vector<double> coefficient_c_;
  std::vector<double> coefficient_d_;
};

#endif // CUBIC_SPLINER_H_