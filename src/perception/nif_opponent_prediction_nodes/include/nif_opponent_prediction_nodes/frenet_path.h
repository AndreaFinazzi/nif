#ifndef FRENET_PATH_H_
#define FRENET_PATH_H_

// #include <geometry_msgs/msg/point.hpp>
// #include <std_msgs/msg/ColorRGBA.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <tuple>
#include <vector>

#include "cubic_spliner_2D.h"
#include "opencv2/opencv.hpp"
#include "quartic_polynomial.h"
#include "quintic_polynomial.h"

class FrenetPath
{
public:
  FrenetPath() {}
  FrenetPath(FrenetPath &frenet_path);

  // Constructs an empty Frenet path
  FrenetPath(double point_d, double point_s)
  {
    points_d_.push_back(point_d);
    points_s_.push_back(point_s);
  }

  ~FrenetPath() {}

  void push_back_derivatives_d(double zeroth_derivative,
                               double first_derivative, double second_drivative,
                               double third_derivative);
  void push_back_derivatives_s(double zeroth_derivative,
                               double first_derivative, double second_drivative,
                               double third_derivative);

  void push_back_time(double time) { time_.push_back(time); }

  void push_back_point_x(double point_x) { points_x_.push_back(point_x); }
  void push_back_point_y(double point_y) { points_y_.push_back(point_y); }
  void push_back_yaw(double yaw) { yaw_.push_back(yaw); }

  void push_back_delta_velocity(double delta_velocity)
  {
    delta_velocity_.push_back(delta_velocity);
  }
  void push_back_curvature(double curvature)
  {
    curvature_.push_back(curvature);
  }

  const std::vector<double> &points_d() const { return points_d_; }
  const std::vector<double> &first_derivative_d() const
  {
    return first_derivative_d_;
  }
  const std::vector<double> &second_derivative_d() const
  {
    return second_derivative_d_;
  }
  const std::vector<double> &third_derivative_d() const
  {
    return third_derivative_d_;
  }

  const std::vector<double> &points_s() const { return points_s_; }
  const std::vector<double> &first_derivative_s() const
  {
    return first_derivative_s_;
  }
  const std::vector<double> &second_derivative_s() const
  {
    return second_derivative_s_;
  }
  const std::vector<double> &third_derivative_s() const
  {
    return third_derivative_s_;
  }

  const std::vector<double> &time() const { return time_; }

  const std::vector<double> &points_x() const { return points_x_; }
  const std::vector<double> &points_y() const { return points_y_; }
  const std::vector<double> &yaw() const { return yaw_; }
  const std::vector<double> &curvature() const { return curvature_; }

  const std::vector<double> &delta_velocity() const { return delta_velocity_; }

  void set_cost_d(double cost_d) { cost_d_ = cost_d; }
  void set_cost_s(double cost_s) { cost_s_ = cost_s; }
  void set_cost_total(double cost_total) { cost_total_ = cost_total; }
  void set_blocking_flg(bool blk_flg) { blocking_flg_ = blk_flg; }
  void set_blocking_idx(int blk_idx) { blocking_idx_ = blk_idx; }

  double cost_d() const { return cost_d_; }
  double cost_s() const { return cost_s_; }
  double cost_total() const { return cost_total_; }
  bool blocking_flg() const { return blocking_flg_; }
  int blocking_idx() const { return blocking_idx_; }

private:
  std::vector<double> points_d_;
  std::vector<double> first_derivative_d_;
  std::vector<double> second_derivative_d_;
  std::vector<double> third_derivative_d_;

  std::vector<double> points_s_;
  std::vector<double> first_derivative_s_;
  std::vector<double> second_derivative_s_;
  std::vector<double> third_derivative_s_;

  std::vector<double> time_;

  std::vector<double> points_x_;
  std::vector<double> points_y_;
  std::vector<double> yaw_;
  std::vector<double> curvature_;

  std::vector<double> delta_velocity_;

  double cost_d_;
  double cost_s_;
  double cost_total_;

  // for test
  bool blocking_flg_ = false;
  int blocking_idx_ = -1;
};

#endif // FRENET_PATH_H_