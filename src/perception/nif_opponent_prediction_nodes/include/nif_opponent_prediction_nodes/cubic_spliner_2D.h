#ifndef CUBIC_SPLINER_2D_H_
#define CUBIC_SPLINER_2D_H_

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <utility>
#include <vector>

#include "cubic_spliner.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

class CubicSpliner2D
{
public:
  // points is a collection of points of certain coordinate, where point is a
  // single value of certain coordinate
  CubicSpliner2D(std::vector<double> &points_x, std::vector<double> &points_y);
  ~CubicSpliner2D() {}

  std::vector<double> &points_s() { return points_s_; }

  std::tuple<double, double> calculate_position(double point_s);
  double calculate_curvature(double point_s);
  double calculate_yaw(double point_s);

protected:
  void calculate_points_s(std::vector<double> &points_x,
                          std::vector<double> &points_y);

private:
  std::vector<double> points_s_;
  std::vector<double> points_x_;
  std::vector<double> points_y_;

  std::shared_ptr<CubicSpliner> cubic_spliner_sx_;
  std::shared_ptr<CubicSpliner> cubic_spliner_sy_;
};

#endif // CUBIC_SPLINER_2D_H_