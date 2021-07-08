#ifndef CUBIC_SPLINER_2D_H_
#define CUBIC_SPLINER_2D_H_

#include <algorithm>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <numeric>
#include <stdio.h>
#include <stdlib.h>
#include <utility>
#include <vector>

#include "cubic_spliner.h"

class CubicSpliner2D {
public:
  // points is a collection of points of certain coordinate, where point is a
  // single value of certain coordinate
  CubicSpliner2D(std::vector<double>& points_x, std::vector<double>& points_y);
  ~CubicSpliner2D() {}

  const std::vector<double>& points_s() const {
    return points_s_;
  }
  std::vector<double>& points_ss() {
    return points_s_;
  }

  std::tuple<double, double> calculate_position(double point_s);
  double calculate_curvature(double point_s);
  double calculate_yaw(double point_s);

protected:
  void calculate_points_s(std::vector<double>& points_x,
                          std::vector<double>& points_y);

private:
  std::vector<double> points_s_;
  std::vector<double> points_x_;
  std::vector<double> points_y_;

  std::shared_ptr<CubicSpliner> cubic_spliner_sx_;
  std::shared_ptr<CubicSpliner> cubic_spliner_sy_;
};

#endif // CUBIC_SPLINER_2D_H_