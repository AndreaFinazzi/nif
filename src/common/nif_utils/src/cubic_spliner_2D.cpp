#include "nif_utils/cubic_spliner_2D.h"

CubicSpliner2D::CubicSpliner2D(std::vector<double>& points_x,
                               std::vector<double>& points_y)
  : points_x_(points_x), points_y_(points_y) {
  assert(points_x_.size() == points_y_.size());

  calculate_points_s(points_x_, points_y_);

  cubic_spliner_sx_ =
      std::shared_ptr<CubicSpliner>(new CubicSpliner(points_s_, points_x_));
  cubic_spliner_sy_ =
      std::shared_ptr<CubicSpliner>(new CubicSpliner(points_s_, points_y_));
}

void CubicSpliner2D::calculate_points_s(std::vector<double>& points_x,
                                        std::vector<double>& points_y) {
  assert(points_x.size() == points_y.size());

  std::vector<double> delta_x(points_x.size());
  std::vector<double> delta_y(points_y.size());

  std::adjacent_difference(points_x.begin(), points_x.end(), delta_x.begin());
  std::adjacent_difference(points_y.begin(), points_y.end(), delta_y.begin());
  delta_x.erase(delta_x.begin());
  delta_y.erase(delta_y.begin());

  std::vector<double> delta_s;

  points_s_.push_back(0);

  for (auto itr = delta_x.begin(); itr != delta_x.end(); itr++) {
    int i = itr - delta_x.begin();

    double delta_s_i = hypot(delta_x[i], delta_y[i]);
    double point_s = points_s_[i] + delta_s_i;

    delta_s.push_back(delta_s_i);
    points_s_.push_back(point_s);
  }
}

std::tuple<double, double> CubicSpliner2D::calculate_position(double point_s) {
  double point_x = cubic_spliner_sx_->calculate_zeroth_derivative(point_s);
  double point_y = cubic_spliner_sy_->calculate_zeroth_derivative(point_s);

  // std::cout<<"point_x"<<std::endl;
  // std::cout<<point_x<<std::endl;

  return std::make_tuple(point_x, point_y);
}

double CubicSpliner2D::calculate_curvature(double point_s) {
  double first_derivative_x =
      cubic_spliner_sx_->calculate_first_derivative(point_s);
  double second_derivative_x =
      cubic_spliner_sx_->calculate_second_derivative(point_s);

  double first_derivative_y =
      cubic_spliner_sy_->calculate_first_derivative(point_s);
  double second_derivative_y =
      cubic_spliner_sy_->calculate_second_derivative(point_s);

  double curvature = (second_derivative_y * first_derivative_x -
                      second_derivative_x * first_derivative_y) /
      pow((first_derivative_x * first_derivative_x +
           first_derivative_y * first_derivative_y),
          1.5);

  return curvature;
}

double CubicSpliner2D::calculate_yaw(double point_s) {
  double first_derivative_x =
      cubic_spliner_sx_->calculate_first_derivative(point_s);
  double first_derivative_y =
      cubic_spliner_sy_->calculate_first_derivative(point_s);

  double yaw = atan2(first_derivative_y, first_derivative_x);

  return yaw;
}
