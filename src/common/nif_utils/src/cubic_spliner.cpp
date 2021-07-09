#include "nif_utils/cubic_spliner.h"

CubicSpliner::CubicSpliner(std::vector<double>& points_x,
                           std::vector<double>& points_y)
  : points_x_(points_x), points_y_(points_y) {
//  ROS_DEBUG("Object of Spline Class was created");

  assert(points_x_.size() == points_y_.size());

  std::vector<double> delta_x(points_x_.size());
  std::adjacent_difference(points_x_.begin(), points_x_.end(), delta_x.begin());
  delta_x.erase(delta_x.begin());

  coefficient_a_ = points_y;

  std::shared_ptr<cv::Mat> matrix_A = calculate_matrix_A(delta_x);
  std::shared_ptr<cv::Mat> matrix_B = calculate_matrix_B(delta_x);
  std::shared_ptr<cv::Mat> matrix_C =
      std::make_shared<cv::Mat>(matrix_A->inv() * (*matrix_B));

  if (matrix_C->isContinuous()) {
    matrix_C->col(0).copyTo(coefficient_c_);
  }

  for (auto itr = points_x_.begin(); itr != points_x_.end(); itr++) {
    int i = itr - points_x_.begin();
    double b = (coefficient_a_[i + 1] - coefficient_a_[i]) / delta_x[i] -
        delta_x[i] * (coefficient_c_[i + 1] + 2.0 * coefficient_c_[i]) / 3;
    double d = (coefficient_c_[i + 1] - coefficient_c_[i]) / (3.0 * delta_x[i]);

    coefficient_b_.push_back(b);
    coefficient_d_.push_back(d);
  }

  coefficient_b_.pop_back();
  coefficient_d_.pop_back();
}

int CubicSpliner::find_index(double point_x) {
  int index = std::lower_bound(points_x_.begin(), points_x_.end(), point_x) -
      points_x_.begin();

  return index == 0 ? 0 : index - 1;
}

std::shared_ptr<cv::Mat>
CubicSpliner::calculate_matrix_A(std::vector<double>& delta_x) {
  int dimension = number_of_points();
  std::shared_ptr<cv::Mat> matrix_A(
      new cv::Mat(dimension, dimension, CV_64F, 0.0));

  matrix_A->at<double>(0, 0) = 1.0;

  for (auto itr = delta_x.begin(); itr != delta_x.end(); itr++) {
    int i = itr - delta_x.begin();

    if (i != dimension) {
      matrix_A->at<double>(i + 1, i + 1) = 2.0 * (*itr + *(itr + 1));
    }
    matrix_A->at<double>(i + 1, i) = *itr;
    matrix_A->at<double>(i, i + 1) = *itr;
  }

  matrix_A->at<double>(0, 1) = 0.0;
  matrix_A->at<double>(dimension - 1, dimension - 2) = 0.0;
  matrix_A->at<double>(dimension - 1, dimension - 1) = 1.0;

  return matrix_A;
}

std::shared_ptr<cv::Mat>
CubicSpliner::calculate_matrix_B(std::vector<double>& delta_x) {
  int dimension = number_of_points();
  std::shared_ptr<cv::Mat> matrix_B(new cv::Mat(dimension, 1, CV_64F, 0.0));

  for (int i = 0; i < dimension - 2; i++) {
    matrix_B->at<double>(i, 1) =
        3.0 * (coefficient_a_[i + 2] - coefficient_a_[i + 1]) / delta_x[i + 1] -
        3.0 * (coefficient_a_[i + 1] - coefficient_a_[i]) / delta_x[i];
  }

  return matrix_B;
}

double CubicSpliner::calculate_zeroth_derivative(double time) {
  if (time < points_x_[0]) {
    assert(0 && "Out of range");
  } else if (time > points_x_.back()) {
    return -1;
  }

  int index = find_index(time);
  double dx = time - points_x_[index];

  double zeroth_derivative = coefficient_a_[index] +
      coefficient_b_[index] * dx + coefficient_c_[index] * pow(dx, 2) +
      coefficient_d_[index] * pow(dx, 3);

  // std::cout << "coefficient_a_[index] " << coefficient_a_[index] <<
  // std::endl; std::cout << "coefficient_b_[index] " << coefficient_b_[index]
  // << std::endl; std::cout << "coefficient_c_[index] " <<
  // coefficient_c_[index] << std::endl; std::cout << "coefficient_d_[index] "
  // << coefficient_d_[index] << std::endl;

  return zeroth_derivative;
}

double CubicSpliner::calculate_first_derivative(double time) {
  if (time < points_x_[0]) {
    assert(0 && "Out of range");
  } else if (time > points_x_.back()) {
    return 0;
  }

  int index = find_index(time);
  double dx = time - points_x_[index];
  double first_derivative = coefficient_b_[index] +
      2 * coefficient_c_[index] * dx + 3 * coefficient_d_[index] * pow(dx, 2);

  return first_derivative;
}

double CubicSpliner::calculate_second_derivative(double time) {
  if (time < points_x_[0]) {
    assert(0 && "Out of range");
  } else if (time > points_x_.back()) {
    return 0;
  }

  int index = find_index(time);
  double dx = time - points_x_[index];
  double second_derivative =
      2 * coefficient_c_[index] + 6 * coefficient_d_[index] * dx;

  return second_derivative;
}