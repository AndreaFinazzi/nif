#include "nif_utils/frenet_path_generator.h"
#include "nif_common/constants.h"

// cost weights
//#define cost_param_k_j 0.1
// #define cost_param_k_t 0.1
// #define cost_param_k_d 0
// #define cost_param_k_lat 10.0
// #define cost_param_k_lon 1.0

using namespace std;
using namespace Frenet;

void FrenetPathGenerator::loadPathConstraints(
    string path_contraints_yaml_file_path_) {
  m_path_contraints_yaml_file_path = path_contraints_yaml_file_path_;
  // TODO: load yaml file,read param and assign the value bellow:
  // constraint_param_max_speed
  // constraint_param_max_accel
  // constraint_param_max_curvature
  // constraint_param_robot_radius
  constraint_param_max_speed = 10000000000.0; // currently set up large number
  constraint_param_max_accel = 10000000000.0; // currently set up large number
}

void FrenetPathGenerator::loadCostParam(string cost_param_yaml_file_path_) {
  m_cost_param_yaml_file_path = cost_param_yaml_file_path_;
  // TODO: load yaml file,read param and assign the value bellow:
  // cost_param_k_j
  // cost_param_k_t
  // cost_param_k_d
  // cost_param_k_lat
  // cost_param_k_lon

  // currently, set to zero.
  cost_param_k_j = 0.0;
  cost_param_k_t = 0.0;
  cost_param_k_d = 0.0;
  cost_param_k_lat = 0.0;
  cost_param_k_lon = 0.0;
}

FrenetPathGenerator::FrenetPathGenerator(string path_contraints_yaml_file_path,
                                         string cost_param_yaml_file_path) {
  loadPathConstraints(path_contraints_yaml_file_path);
  loadCostParam(cost_param_yaml_file_path);
}

std::shared_ptr<FrenetPath> FrenetPathGenerator::genSingleFrenetPath(
    double ref_splined_end_s, double current_position_d,
    double current_position_s, double current_velocity_d,
    double current_velocity_s, double current_acceleration_d,
    double current_acceleration_s, double target_position_d,
    double target_velocity_d, double target_velocity_s,
    double target_acceleration_d, double target_acceleration_s,
    double planning_t, double dt) {
  double time = planning_t;
  std::shared_ptr<FrenetPath> frenet_path(new FrenetPath());
  std::shared_ptr<QuinticPolynomial> quintic_polynomial_lateral(
      new QuinticPolynomial(current_position_d, current_velocity_d,
                            current_acceleration_d, target_position_d,
                            target_velocity_d, target_acceleration_d, time));

  double frenet_path_time_i = 0.0;
  while (frenet_path_time_i < time) {
    frenet_path->push_back_time(frenet_path_time_i);
    frenet_path_time_i += dt;
  }

  const std::vector<double> &frenet_path_time_lat = frenet_path->time();
  for (auto itr = frenet_path_time_lat.begin();
       itr != frenet_path_time_lat.end(); itr++) {
    double frenet_path_time_i = *itr;

    double zeroth_derivative_d =
        quintic_polynomial_lateral->calculate_zeroth_derivative(
            frenet_path_time_i);
    double first_derivative_d =
        quintic_polynomial_lateral->calculate_first_derivative(
            frenet_path_time_i);
    double second_derivative_d =
        quintic_polynomial_lateral->calculate_second_derivative(
            frenet_path_time_i);
    double third_derivative_d =
        quintic_polynomial_lateral->calculate_third_derivative(
            frenet_path_time_i);

    frenet_path->pushBackDerivatives_d(zeroth_derivative_d, first_derivative_d,
                                       second_derivative_d, third_derivative_d);
  }

  std::shared_ptr<FrenetPath> frenet_path_target_velocity_s(
      new FrenetPath(*frenet_path));

  std::shared_ptr<QuarticPolynomial> quartic_polynomial_longitudinal(
      new QuarticPolynomial(current_position_s, current_velocity_s,
                            current_acceleration_s, target_velocity_s,
                            target_acceleration_s, time));

  const std::vector<double> &frenet_path_time_long = frenet_path->time();
  for (auto itr = frenet_path_time_long.begin();
       itr != frenet_path_time_long.end(); itr++) {
    double frenet_path_time_i = *itr;

    double zeroth_derivative_s =
        quartic_polynomial_longitudinal->calculate_zeroth_derivative(
            frenet_path_time_i);
    if (ref_splined_end_s < zeroth_derivative_s) {
      break;
    }
    double first_derivative_s =
        quartic_polynomial_longitudinal->calculate_first_derivative(
            frenet_path_time_i);
    double second_derivative_s =
        quartic_polynomial_longitudinal->calculate_second_derivative(
            frenet_path_time_i);
    double third_derivative_s =
        quartic_polynomial_longitudinal->calculate_third_derivative(
            frenet_path_time_i);

    frenet_path_target_velocity_s->pushBackDerivatives_s(
        zeroth_derivative_s, first_derivative_s, second_derivative_s,
        third_derivative_s);
  }

  double jerk_d = 0;
  const std::vector<double> &third_derivative_d =
      frenet_path_target_velocity_s->third_derivative_d();
  for (auto itr = third_derivative_d.begin(); itr != third_derivative_d.end();
       itr++) {
    jerk_d += pow(*itr, 2);
  }

  double jerk_s = 0;
  const std::vector<double> &third_derivative_s =
      frenet_path_target_velocity_s->third_derivative_s();
  for (auto itr = third_derivative_s.begin(); itr != third_derivative_s.end();
       itr++) {
    jerk_s += pow(*itr, 2);
  }

  double delta_velocity =
      pow(target_velocity_s -
              frenet_path_target_velocity_s->first_derivative_s().back(),
          2);

  double cost_d = cost_param_k_j * jerk_d + cost_param_k_t * time +
                  cost_param_k_d * pow(frenet_path->points_d().back(), 2);
  double cost_s = cost_param_k_j * jerk_s + cost_param_k_t * time +
                  cost_param_k_d * delta_velocity;
  double cost_total = cost_param_k_lat * cost_d + cost_param_k_lon * cost_s;

  frenet_path_target_velocity_s->set_cost_d(cost_d);
  frenet_path_target_velocity_s->set_cost_s(cost_s);
  frenet_path_target_velocity_s->set_cost_total(cost_total);

  frenet_path = frenet_path_target_velocity_s;
  return frenet_path;
}

std::vector<std::shared_ptr<FrenetPath>>
FrenetPathGenerator::genMultipleLongiFrenetPaths(
    double ref_splined_end_s, double current_position_d,
    double current_position_s, double current_velocity_d,
    double current_velocity_s, double current_acceleration_d,
    double current_acceleration_s, double target_position_d,
    double target_velocity_d, double target_velocity_s,
    double target_acceleration_d, double target_acceleration_s,
    double planning_min_t, double planning_max_t, double planning_sampling_t,
    double dt) {
  assert(planning_min_t < planning_max_t);
  assert(planning_sampling_t >= 0);

  std::vector<std::shared_ptr<FrenetPath>> frenet_paths;

  double expected_position_d = target_position_d;
  double time = planning_min_t;
  while (time <= planning_max_t) {
    std::shared_ptr<FrenetPath> frenet_path(new FrenetPath());
    std::shared_ptr<QuinticPolynomial> quintic_polynomial_lateral(
        new QuinticPolynomial(current_position_d, current_velocity_d,
                              current_acceleration_d, target_position_d,
                              target_velocity_d, target_acceleration_d, time));

    double frenet_path_time_i = 0.0;
    while (frenet_path_time_i < time) {
      frenet_path->push_back_time(frenet_path_time_i);
      frenet_path_time_i += dt;
    }

    const std::vector<double> &frenet_path_time_lat = frenet_path->time();
    for (auto itr = frenet_path_time_lat.begin();
         itr != frenet_path_time_lat.end(); itr++) {
      double frenet_path_time_i = *itr;

      double zeroth_derivative_d =
          quintic_polynomial_lateral->calculate_zeroth_derivative(
              frenet_path_time_i);
      double first_derivative_d =
          quintic_polynomial_lateral->calculate_first_derivative(
              frenet_path_time_i);
      double second_derivative_d =
          quintic_polynomial_lateral->calculate_second_derivative(
              frenet_path_time_i);
      double third_derivative_d =
          quintic_polynomial_lateral->calculate_third_derivative(
              frenet_path_time_i);

      frenet_path->pushBackDerivatives_d(
          zeroth_derivative_d, first_derivative_d, second_derivative_d,
          third_derivative_d);
    }

    std::shared_ptr<FrenetPath> frenet_path_target_velocity_s(
        new FrenetPath(*frenet_path));

    std::shared_ptr<QuarticPolynomial> quartic_polynomial_longitudinal(
        new QuarticPolynomial(current_position_s, current_velocity_s,
                              current_acceleration_s, target_velocity_s,
                              target_acceleration_s, time));

    const std::vector<double> &frenet_path_time_long = frenet_path->time();
    for (auto itr = frenet_path_time_long.begin();
         itr != frenet_path_time_long.end(); itr++) {
      double frenet_path_time_i = *itr;

      double zeroth_derivative_s =
          quartic_polynomial_longitudinal->calculate_zeroth_derivative(
              frenet_path_time_i);
      if (ref_splined_end_s < zeroth_derivative_s) {
        break;
      }
      double first_derivative_s =
          quartic_polynomial_longitudinal->calculate_first_derivative(
              frenet_path_time_i);
      double second_derivative_s =
          quartic_polynomial_longitudinal->calculate_second_derivative(
              frenet_path_time_i);
      double third_derivative_s =
          quartic_polynomial_longitudinal->calculate_third_derivative(
              frenet_path_time_i);

      frenet_path_target_velocity_s->pushBackDerivatives_s(
          zeroth_derivative_s, first_derivative_s, second_derivative_s,
          third_derivative_s);
    }

    double jerk_d = 0;
    const std::vector<double> &third_derivative_d =
        frenet_path_target_velocity_s->third_derivative_d();
    for (auto itr = third_derivative_d.begin(); itr != third_derivative_d.end();
         itr++) {
      jerk_d += pow(*itr, 2);
    }

    double jerk_s = 0;
    const std::vector<double> &third_derivative_s =
        frenet_path_target_velocity_s->third_derivative_s();
    for (auto itr = third_derivative_s.begin(); itr != third_derivative_s.end();
         itr++) {
      jerk_s += pow(*itr, 2);
    }

    double delta_velocity =
        pow(target_velocity_s -
                frenet_path_target_velocity_s->first_derivative_s().back(),
            2);

    double cost_d = cost_param_k_j * jerk_d + cost_param_k_t * time +
                    cost_param_k_d * pow(frenet_path->points_d().back(), 2);
    double cost_s = cost_param_k_j * jerk_s + cost_param_k_t * time +
                    cost_param_k_d * delta_velocity;
    double cost_total = cost_param_k_lat * cost_d + cost_param_k_lon * cost_s;

    frenet_path_target_velocity_s->set_cost_d(cost_d);
    frenet_path_target_velocity_s->set_cost_s(cost_s);
    frenet_path_target_velocity_s->set_cost_total(cost_total);

    frenet_paths.push_back(frenet_path_target_velocity_s);

    time += planning_sampling_t;
  }

  return frenet_paths;
}

std::vector<std::shared_ptr<FrenetPath>>
FrenetPathGenerator::genMultipleLateralFrenetPaths(
    double ref_splined_end_s, double current_position_d,
    double current_position_s, double current_velocity_d,
    double current_velocity_s, double current_acceleration_d,
    double current_acceleration_s,
    double target_position_min_d, // most left side
    double target_position_max_d, // most right side
    double target_velocity_d, double target_velocity_s,
    double target_acceleration_d, double target_acceleration_s,
    double planning_sampling_d, double planning_t, double dt) {
  assert(target_position_min_d < target_position_max_d);
  assert(planning_sampling_d >= 0);

  std::vector<std::shared_ptr<FrenetPath>> frenet_paths;

  double expected_position_d = target_position_min_d;
  while (expected_position_d <= target_position_max_d) {
    double time = planning_t;
    std::shared_ptr<FrenetPath> frenet_path(new FrenetPath());
    std::shared_ptr<QuinticPolynomial> quintic_polynomial_lateral(
        new QuinticPolynomial(current_position_d, current_velocity_d,
                              current_acceleration_d, expected_position_d,
                              target_velocity_d, target_acceleration_d, time));

    double frenet_path_time_i = 0.0;
    while (frenet_path_time_i < time) {
      frenet_path->push_back_time(frenet_path_time_i);
      frenet_path_time_i += dt;
    }

    const std::vector<double> &frenet_path_time_lat = frenet_path->time();
    for (auto itr = frenet_path_time_lat.begin();
         itr != frenet_path_time_lat.end(); itr++) {
      double frenet_path_time_i = *itr;

      double zeroth_derivative_d =
          quintic_polynomial_lateral->calculate_zeroth_derivative(
              frenet_path_time_i);
      double first_derivative_d =
          quintic_polynomial_lateral->calculate_first_derivative(
              frenet_path_time_i);
      double second_derivative_d =
          quintic_polynomial_lateral->calculate_second_derivative(
              frenet_path_time_i);
      double third_derivative_d =
          quintic_polynomial_lateral->calculate_third_derivative(
              frenet_path_time_i);

      frenet_path->pushBackDerivatives_d(
          zeroth_derivative_d, first_derivative_d, second_derivative_d,
          third_derivative_d);
    }

    std::shared_ptr<FrenetPath> frenet_path_target_velocity_s(
        new FrenetPath(*frenet_path));

    std::shared_ptr<QuarticPolynomial> quartic_polynomial_longitudinal(
        new QuarticPolynomial(current_position_s, current_velocity_s,
                              current_acceleration_s, target_velocity_s,
                              target_acceleration_s, time));

    const std::vector<double> &frenet_path_time_long = frenet_path->time();
    for (auto itr = frenet_path_time_long.begin();
         itr != frenet_path_time_long.end(); itr++) {
      double frenet_path_time_i = *itr;

      double zeroth_derivative_s =
          quartic_polynomial_longitudinal->calculate_zeroth_derivative(
              frenet_path_time_i);
      if (ref_splined_end_s < zeroth_derivative_s) {
        break;
      }
      double first_derivative_s =
          quartic_polynomial_longitudinal->calculate_first_derivative(
              frenet_path_time_i);
      double second_derivative_s =
          quartic_polynomial_longitudinal->calculate_second_derivative(
              frenet_path_time_i);
      double third_derivative_s =
          quartic_polynomial_longitudinal->calculate_third_derivative(
              frenet_path_time_i);

      frenet_path_target_velocity_s->pushBackDerivatives_s(
          zeroth_derivative_s, first_derivative_s, second_derivative_s,
          third_derivative_s);
    }

    double jerk_d = 0;
    const std::vector<double> &third_derivative_d =
        frenet_path_target_velocity_s->third_derivative_d();
    for (auto itr = third_derivative_d.begin(); itr != third_derivative_d.end();
         itr++) {
      jerk_d += pow(*itr, 2);
    }

    double jerk_s = 0;
    const std::vector<double> &third_derivative_s =
        frenet_path_target_velocity_s->third_derivative_s();
    for (auto itr = third_derivative_s.begin(); itr != third_derivative_s.end();
         itr++) {
      jerk_s += pow(*itr, 2);
    }

    double delta_velocity =
        pow(target_velocity_s -
                frenet_path_target_velocity_s->first_derivative_s().back(),
            2);

    double cost_d = cost_param_k_j * jerk_d + cost_param_k_t * time +
                    cost_param_k_d * pow(frenet_path->points_d().back(), 2);
    double cost_s = cost_param_k_j * jerk_s + cost_param_k_t * time +
                    cost_param_k_d * delta_velocity;
    double cost_total = cost_param_k_lat * cost_d + cost_param_k_lon * cost_s;

    frenet_path_target_velocity_s->set_cost_d(cost_d);
    frenet_path_target_velocity_s->set_cost_s(cost_s);
    frenet_path_target_velocity_s->set_cost_total(cost_total);

    frenet_paths.push_back(frenet_path_target_velocity_s);

    expected_position_d += planning_sampling_d;
  }

  return frenet_paths;
}

std::vector<std::shared_ptr<FrenetPath>>
FrenetPathGenerator::genMultipleLateralLongiFrenetPaths(
    double ref_splined_end_s, double current_position_d,
    double current_position_s, double current_velocity_d,
    double current_velocity_s, double current_acceleration_d,
    double current_acceleration_s,
    double target_position_min_d, // most left side
    double target_position_max_d, // most right side
    double target_velocity_d, double target_velocity_s,
    double target_acceleration_d, double target_acceleration_s,
    double planning_sampling_d, double planning_min_t, double planning_max_t,
    double planning_sampling_t, double dt) {
  assert(target_position_min_d > target_position_max_d);
  assert(planning_sampling_d >= 0);
  assert(planning_min_t < planning_max_t);
  assert(planning_sampling_t >= 0);

  std::vector<std::shared_ptr<FrenetPath>> frenet_paths;

  double expected_position_d = target_position_min_d;
  while (expected_position_d <= target_position_max_d) {
    double time = planning_min_t;
    while (time < planning_max_t) {
      std::shared_ptr<FrenetPath> frenet_path(new FrenetPath());

      std::shared_ptr<QuinticPolynomial> quintic_polynomial_lateral(
          new QuinticPolynomial(current_position_d, current_velocity_d,
                                current_acceleration_d, expected_position_d,
                                target_velocity_d, target_acceleration_d,
                                time));

      double frenet_path_time_i = 0.0;
      while (frenet_path_time_i < time) {
        frenet_path->push_back_time(frenet_path_time_i);
        frenet_path_time_i += dt;
      }

      const std::vector<double> &frenet_path_time_lat = frenet_path->time();
      for (auto itr = frenet_path_time_lat.begin();
           itr != frenet_path_time_lat.end(); itr++) {
        double frenet_path_time_i = *itr;

        double zeroth_derivative_d =
            quintic_polynomial_lateral->calculate_zeroth_derivative(
                frenet_path_time_i);
        double first_derivative_d =
            quintic_polynomial_lateral->calculate_first_derivative(
                frenet_path_time_i);
        double second_derivative_d =
            quintic_polynomial_lateral->calculate_second_derivative(
                frenet_path_time_i);
        double third_derivative_d =
            quintic_polynomial_lateral->calculate_third_derivative(
                frenet_path_time_i);

        frenet_path->pushBackDerivatives_d(
            zeroth_derivative_d, first_derivative_d, second_derivative_d,
            third_derivative_d);
      }

      std::shared_ptr<FrenetPath> frenet_path_target_velocity_s(
          new FrenetPath(*frenet_path));

      std::shared_ptr<QuarticPolynomial> quartic_polynomial_longitudinal(
          new QuarticPolynomial(current_position_s, current_velocity_s,
                                current_acceleration_s, target_velocity_s,
                                target_acceleration_s, time));

      const std::vector<double> &frenet_path_time_long = frenet_path->time();
      for (auto itr = frenet_path_time_long.begin();
           itr != frenet_path_time_long.end(); itr++) {
        double frenet_path_time_i = *itr;

        double zeroth_derivative_s =
            quartic_polynomial_longitudinal->calculate_zeroth_derivative(
                frenet_path_time_i);
        if (ref_splined_end_s < zeroth_derivative_s) {
          break;
        }
        double first_derivative_s =
            quartic_polynomial_longitudinal->calculate_first_derivative(
                frenet_path_time_i);
        double second_derivative_s =
            quartic_polynomial_longitudinal->calculate_second_derivative(
                frenet_path_time_i);
        double third_derivative_s =
            quartic_polynomial_longitudinal->calculate_third_derivative(
                frenet_path_time_i);

        frenet_path_target_velocity_s->pushBackDerivatives_s(
            zeroth_derivative_s, first_derivative_s, second_derivative_s,
            third_derivative_s);
      }

      double jerk_d = 0;
      const std::vector<double> &third_derivative_d =
          frenet_path_target_velocity_s->third_derivative_d();
      for (auto itr = third_derivative_d.begin();
           itr != third_derivative_d.end(); itr++) {
        jerk_d += pow(*itr, 2);
      }

      double jerk_s = 0;
      const std::vector<double> &third_derivative_s =
          frenet_path_target_velocity_s->third_derivative_s();
      for (auto itr = third_derivative_s.begin();
           itr != third_derivative_s.end(); itr++) {
        jerk_s += pow(*itr, 2);
      }

      double delta_velocity =
          pow(target_velocity_s -
                  frenet_path_target_velocity_s->first_derivative_s().back(),
              2);

      double cost_d = cost_param_k_j * jerk_d + cost_param_k_t * time +
                      cost_param_k_d * pow(frenet_path->points_d().back(), 2);
      double cost_s = cost_param_k_j * jerk_s + cost_param_k_t * time +
                      cost_param_k_d * delta_velocity;
      double cost_total = cost_param_k_lat * cost_d + cost_param_k_lon * cost_s;

      frenet_path_target_velocity_s->set_cost_d(cost_d);
      frenet_path_target_velocity_s->set_cost_s(cost_s);
      frenet_path_target_velocity_s->set_cost_total(cost_total);

      frenet_paths.push_back(frenet_path_target_velocity_s);

      time += planning_sampling_t;
    }
    expected_position_d += planning_sampling_d;
  }

  return frenet_paths;
}

void FrenetPathGenerator::convertFrenetPathsInGlobal(
    std::vector<std::shared_ptr<FrenetPath>> &frenet_paths,
    std::shared_ptr<CubicSpliner2D> &cubic_spliner_2D) {
  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath> &frenet_path = frenet_paths[i];

    // calculate x and y
    const std::vector<double> &points_s = frenet_path->points_s();

    for (int j = 0; j < frenet_path->points_s().size(); j++) {
      double point_s = points_s[j];

      std::tuple<double, double> position =
          cubic_spliner_2D->calculate_position(point_s);

      double point_x = std::get<0>(position);
      double point_y = std::get<1>(position);
      double yaw = cubic_spliner_2D->calculate_yaw(point_s);

      double point_d = frenet_path->points_d()[j];

      double frenet_x =
          point_x +
          point_d * cos(yaw + nif::common::constants::numeric::PI / 2.0);
      double frenet_y =
          point_y +
          point_d * sin(yaw + nif::common::constants::numeric::PI / 2.0);

      frenet_path->push_back_point_y(frenet_y);
      frenet_path->push_back_point_x(frenet_x);
    }

    // calculate yaw and delta_velocity
    const std::vector<double> &points_x = frenet_path->points_x();
    const std::vector<double> &points_y = frenet_path->points_y();

    assert(points_x.size() == points_y.size());

    if (points_x.empty() || points_x.size() < 2)
      continue;

    for (int j = 0; j < points_x.size() - 1; j++) {
      double delta_x = points_x[j + 1] - points_x[j];
      double delta_y = points_y[j + 1] - points_y[j];

      frenet_path->push_back_yaw(atan2(delta_y, delta_x));
      frenet_path->push_back_delta_velocity(
          sqrt(delta_x * delta_x + delta_y * delta_y));
    }

    frenet_path->push_back_yaw(frenet_path->yaw().back());
    frenet_path->push_back_delta_velocity(frenet_path->delta_velocity().back());

    // calculate curvature
    const std::vector<double> &yaw = frenet_path->yaw();
    const std::vector<double> &delta_velocity = frenet_path->delta_velocity();

    assert(yaw.size() == delta_velocity.size());
  }
}

bool FrenetPathGenerator::checkCollision(
    std::shared_ptr<FrenetPath> &frenet_path,
    std::vector<std::tuple<double, double>> &obstacles) {
  bool is_collision = false;

  for (int i = 0; i < int(obstacles.size()); i++) {
    double obstacle_x = std::get<0>(obstacles[i]);
    double obstacle_y = std::get<0>(obstacles[i]);

    const std::vector<double> &points_x = frenet_path->points_x();
    const std::vector<double> &points_y = frenet_path->points_y();

    for (int j = 0; j < int(points_x.size()); j++) {
      double distance =
          pow(points_x[j] - obstacle_x, 2) + pow(points_y[j] - obstacle_y, 2);

      if (distance < pow(constraint_param_robot_radius, 2)) {
        is_collision = true;
        break;
      }
    }
  }

  return is_collision;
}

void FrenetPathGenerator::checkValidity_w_constraint(
    std::vector<std::shared_ptr<FrenetPath>> &frenet_paths) {
  std::vector<std::shared_ptr<FrenetPath>> valid_frenet_paths;

  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath> frenet_path = frenet_paths[i];

    const std::vector<double> &first_derivative_s =
        frenet_path->first_derivative_s();
    const std::vector<double> &second_derivative_s =
        frenet_path->second_derivative_s();

    if (std::any_of(first_derivative_s.begin(), first_derivative_s.end(),
                    [&](double velocity) {
                      return (velocity > constraint_param_max_speed);
                    })) {
      continue;
    } else if (std::any_of(second_derivative_s.begin(),
                           second_derivative_s.end(), [&](double acceleration) {
                             return abs(acceleration) >
                                    constraint_param_max_accel;
                           })) {
      continue;
    }

    valid_frenet_paths.push_back(frenet_path);
  }
}

void FrenetPathGenerator::checkValidity_w_obstalceMap(
    std::vector<std::shared_ptr<FrenetPath>> &frenet_paths,
    std::vector<std::tuple<double, double>> &obstacles) {
  std::vector<std::shared_ptr<FrenetPath>> valid_frenet_paths;

  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath> frenet_path = frenet_paths[i];

    const std::vector<double> &first_derivative_s =
        frenet_path->first_derivative_s();
    const std::vector<double> &second_derivative_s =
        frenet_path->second_derivative_s();

    if (std::any_of(first_derivative_s.begin(), first_derivative_s.end(),
                    [&](double velocity) {
                      return velocity > constraint_param_max_speed;
                    })) {
      continue;
    } else if (std::any_of(second_derivative_s.begin(),
                           second_derivative_s.end(), [&](double acceleration) {
                             return abs(acceleration) >
                                    constraint_param_max_accel;
                           })) {
      continue;
    } else if (checkCollision(frenet_path, obstacles)) {
      continue;
    }

    valid_frenet_paths.push_back(frenet_path);
  }
}

std::tuple<std::shared_ptr<FrenetPath>,
           std::vector<std::shared_ptr<FrenetPath>>>
FrenetPathGenerator::calcOptimalFrenetPathByMode(
    FRENET_GEN_MODE gen_mode,
    std::shared_ptr<CubicSpliner2D> &ref_cubic_spliner_2D,
    double current_position_d, double current_position_s,
    double current_velocity_d, double current_velocity_s,
    double current_acceleration_d, double current_acceleration_s,
    // sign convention : left side to the reference path is negative
    double target_position_d,
    double target_position_min_d, // most left side
    double target_position_max_d, // most right side
    double target_velocity_d, double target_velocity_s,
    double target_acceleration_d, double target_acceleration_s,
    double planning_sampling_d, double planning_t, double planning_min_t,
    double planning_max_t, double planning_sampling_t, double dt) {
  double ref_splined_end_s = ref_cubic_spliner_2D->points_s().back();

  std::vector<std::shared_ptr<FrenetPath>> frenet_paths;

  if (gen_mode == FRENET_GEN_MODE::ONLY_SINGLE_FP) {
    std::shared_ptr<FrenetPath> frenet_path = genSingleFrenetPath(
        ref_splined_end_s, current_position_d, current_position_s,
        current_velocity_d, current_velocity_s, current_acceleration_d,
        current_acceleration_s, target_position_d, target_velocity_d,
        target_velocity_s, target_acceleration_d, target_acceleration_s,
        planning_t, dt);
    frenet_paths.push_back(frenet_path);
  } else if (gen_mode == FRENET_GEN_MODE::MULTIPLE_LONG_FPS) {
    frenet_paths = genMultipleLongiFrenetPaths(
        ref_splined_end_s, current_position_d, current_position_s,
        current_velocity_d, current_velocity_s, current_acceleration_d,
        current_acceleration_s, target_position_d, target_velocity_d,
        target_velocity_s, target_acceleration_d, target_acceleration_s,
        planning_min_t, planning_max_t, planning_sampling_t, dt);
  } else if (gen_mode == FRENET_GEN_MODE::MULTIPLE_LAT_FPS) {
    frenet_paths = genMultipleLateralFrenetPaths(
        ref_splined_end_s, current_position_d, current_position_s,
        current_velocity_d, current_velocity_s, current_acceleration_d,
        current_acceleration_s,
        target_position_min_d, // most left side
        target_position_max_d, // most right side
        target_velocity_d, target_velocity_s, target_acceleration_d,
        target_acceleration_s, planning_sampling_d, planning_t, dt);
  } else if (gen_mode == FRENET_GEN_MODE::MULTIPLE_LATLONG_FPS) {
    frenet_paths = genMultipleLateralLongiFrenetPaths(
        ref_splined_end_s, current_position_d, current_position_s,
        current_velocity_d, current_velocity_s, current_acceleration_d,
        current_acceleration_s, target_position_min_d, target_position_max_d,
        target_velocity_d, target_velocity_s, target_acceleration_d,
        target_acceleration_s, planning_sampling_d, planning_min_t,
        planning_max_t, planning_sampling_t, dt);
  } else {
    // Wrong mode
  }

  if (frenet_paths.empty() == false) {
    convertFrenetPathsInGlobal(frenet_paths, ref_cubic_spliner_2D);
    // checkValidity_w_constraint(frenet_paths);

    std::shared_ptr<FrenetPath> optimal_frenet_path;

    // // Find Frenet path with the minimum cost
    // double min_cost_total = frenet_paths[0]->cost_total();
    // optimal_frenet_path = frenet_paths[0];
    // for (int i = 1; i < frenet_paths.size(); i++) {
    //   std::shared_ptr<FrenetPath> frenet_path = frenet_paths[i];

    //   if (min_cost_total > frenet_path->cost_total()) {
    //     min_cost_total = frenet_path->cost_total();
    //     optimal_frenet_path = frenet_path;
    //   }
    // }
    return std::make_tuple(optimal_frenet_path, frenet_paths);
  } else {
    std::shared_ptr<FrenetPath> empty_path(new FrenetPath(-1.1, -1.0));
    return std::make_tuple(empty_path, frenet_paths);
  }
}

FrenetPathGenerator::CubicSpliner2DResult
FrenetPathGenerator::applyCubicSpliner_2d(std::vector<double> &points_x,
                                          std::vector<double> &points_y,
                                          double sampling_interval) {
  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
      new CubicSpliner2D(points_x, points_y));

  std::vector<double> cubic_spliner_x, cubic_spliner_y, cubic_spliner_yaw,
      cubic_spliner_curvature;

  double point_s = 0.0;
  double point_s_end = cubic_spliner_2D->points_s().back();

  while (point_s < point_s_end) {
    std::tuple<double, double> position =
        cubic_spliner_2D->calculate_position(point_s);

    double point_x = std::get<0>(position);
    double point_y = std::get<1>(position);

    double yaw = cubic_spliner_2D->calculate_yaw(point_s);

    double curvature = cubic_spliner_2D->calculate_curvature(point_s);

    cubic_spliner_x.push_back(point_x);
    cubic_spliner_y.push_back(point_y);
    cubic_spliner_yaw.push_back(yaw);
    cubic_spliner_curvature.push_back(curvature);

    point_s += sampling_interval;
  }
  return std::make_tuple(cubic_spliner_x, cubic_spliner_y, cubic_spliner_yaw,
                         cubic_spliner_2D);
}

FrenetPathGenerator::CubicSpliner2DResult
FrenetPathGenerator::applyCubicSpliner_2d_ros(nav_msgs::msg::Path &path_body,
                                              double sampling_interval) {

  vector<double> points_x;
  vector<double> points_y;

  for (auto &pt : path_body.poses) {
    points_x.push_back(pt.pose.position.x);
    points_y.push_back(pt.pose.position.y);
  }

  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
      new CubicSpliner2D(points_x, points_y));

  std::vector<double> cubic_spliner_x, cubic_spliner_y, cubic_spliner_yaw;

  double point_s = 0.0;
  double point_s_end = cubic_spliner_2D->points_s().back();

  while (point_s < point_s_end) {
    std::tuple<double, double> position =
        cubic_spliner_2D->calculate_position(point_s);

    double point_x = std::get<0>(position);
    double point_y = std::get<1>(position);

    double yaw = cubic_spliner_2D->calculate_yaw(point_s);

    double curvature = cubic_spliner_2D->calculate_curvature(point_s);

    cubic_spliner_x.push_back(point_x);
    cubic_spliner_y.push_back(point_y);
    cubic_spliner_yaw.push_back(yaw);

    point_s += sampling_interval;
  }
  return std::make_tuple(cubic_spliner_x, cubic_spliner_y, cubic_spliner_yaw,
                         cubic_spliner_2D);
}

// FrenetPathGenerator::CubicSpliner2DResult
// FrenetPathGenerator::applyCubicSpliner_2d(std::vector<double> &points_x,
//                                           std::vector<double> &points_y,
//                                           double sampling_interval) {
//   std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
//       new CubicSpliner2D(points_x, points_y));

//   std::vector<double> cubic_spliner_x, cubic_spliner_y, cubic_spliner_yaw,
//       cubic_spliner_curvature;

//   double point_s = 0.0;
//   double point_s_end = cubic_spliner_2D->points_s().back();

//   while (point_s < point_s_end) {
//     std::tuple<double, double> position =
//         cubic_spliner_2D->calculate_position(point_s);

//     double point_x = std::get<0>(position);
//     double point_y = std::get<1>(position);

//     double yaw = cubic_spliner_2D->calculate_yaw(point_s);

//     double curvature = cubic_spliner_2D->calculate_curvature(point_s);

//     cubic_spliner_x.push_back(point_x);
//     cubic_spliner_y.push_back(point_y);
//     cubic_spliner_yaw.push_back(yaw);
//     cubic_spliner_curvature.push_back(curvature);

//     point_s += sampling_interval;
//   }
//   return std::make_tuple(cubic_spliner_x, cubic_spliner_y, cubic_spliner_yaw,
//                          cubic_spliner_2D);
// }

FrenetPathGenerator::CubicSpliner3DResult
FrenetPathGenerator::applyCubicSpliner_3d(std::vector<double> &points_x,
                                          std::vector<double> &points_y,
                                          std::vector<double> &points_z,
                                          double sampling_interval) {
  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
      new CubicSpliner2D(points_x, points_y));

  std::shared_ptr<CubicSpliner> cubic_spliner_sz_ =
      std::shared_ptr<CubicSpliner>(
          new CubicSpliner(cubic_spliner_2D->points_ss(), points_z));

  std::vector<double> cubic_spliner_x;
  std::vector<double> cubic_spliner_y;
  std::vector<double> cubic_spliner_z;
  std::vector<double> cubic_spliner_yaw;
  std::vector<double> cubic_spliner_curvature;

  double point_s = 0.0;
  double point_s_end = cubic_spliner_2D->points_s().back();

  while (point_s < point_s_end) {
    std::tuple<double, double> position =
        cubic_spliner_2D->calculate_position(point_s);

    double point_x = std::get<0>(position);
    double point_y = std::get<1>(position);
    double point_z = cubic_spliner_sz_->calculate_zeroth_derivative(point_s);

    double yaw = cubic_spliner_2D->calculate_yaw(point_s);

    double curvature = cubic_spliner_2D->calculate_curvature(point_s);

    cubic_spliner_x.push_back(point_x);
    cubic_spliner_y.push_back(point_y);
    cubic_spliner_z.push_back(point_z);
    cubic_spliner_yaw.push_back(yaw);
    cubic_spliner_curvature.push_back(curvature);

    point_s += sampling_interval;
  }
  return std::make_tuple(cubic_spliner_x, cubic_spliner_y, cubic_spliner_z,
                         cubic_spliner_yaw, cubic_spliner_2D);
}

//////////////////////////////////////////////////////////////
#define MAX_SPEED 50.0 / 3.6 // maximum speed [m/s]
#define MAX_ACCEL 2.0        // maximum acceleration [m/ss]
#define MAX_CURVATURE 1.0    // maximum curvature [1/m]
// #define MAX_ROAD_WIDTH 1.0   // maximum road width [m]
#define MAX_ROAD_WIDTH 0.5 // maximum road width [m]
#define D_ROAD_W 0.5       // road width sampling length [m]
#define DT 0.2             // time tick [s]
#define MAX_T                                                                  \
  4.01 // max prediction time [m] ----------------> python code에서 floating \
       // point가 잘 못되서 +- 0.1을 해줌
#define MIN_T                                                                  \
  2.0 // min prediction time [m] ----------------> python code에서 floating \
                                // point가 잘 못되서 +- 0.1을 해줌
#define TARGET_SPEED 10.0 / 3.6 // target speed [m/s]
#define D_T_S 8.0 / 3.6         // target speed sampling length [m/s]
#define N_S_SAMPLE 1            // sampling number of target speed
#define ROBOT_RADIUS 2.0        // robot radius [m]

// cost weights
#define K_J 0.0
#define K_T 0.0
#define K_D 0.9 // 25
// #define K_LAT 1.0
#define K_LAT 1.0
#define K_LON 0.0
#define K_GRID 100 // grid map cost gain

std::vector<std::shared_ptr<FrenetPath>>
FrenetPathGenerator::generate_frenet_paths_v3(
    double current_position_d, double current_position_s,
    double current_velocity_d, double current_velocity_s,
    double current_acceleration_d, double min_t, double max_t, double dt,
    double left_margin = -MAX_ROAD_WIDTH, double right_margin = MAX_ROAD_WIDTH,
    double width_d = D_ROAD_W, int num_speed_sample = 1) {

  // assert(left_margin >= 0);
  // assert(right_margin <= 0);
  // assert(width_d <= 0);

  // left margin should be set negative
  // right margin should be set positive
  // width_d should be set negative

  std::vector<std::shared_ptr<FrenetPath>> frenet_paths;

  double expected_position_d = left_margin;
  while (expected_position_d <= right_margin) {
    double time = min_t;
    while (time < max_t) {
      std::shared_ptr<FrenetPath> frenet_path(new FrenetPath());

      std::shared_ptr<QuinticPolynomial> quintic_polynomial_lateral(
          new QuinticPolynomial(current_position_d, current_velocity_d,
                                current_acceleration_d, expected_position_d,
                                0.0, 0.0, time));

      double frenet_path_time_i = 0.0;
      while (frenet_path_time_i < time) {
        frenet_path->push_back_time(frenet_path_time_i);
        frenet_path_time_i += dt;
      }

      const std::vector<double> &frenet_path_time = frenet_path->time();
      for (auto itr = frenet_path_time.begin(); itr != frenet_path_time.end();
           itr++) {
        double frenet_path_time_i = *itr;

        double zeroth_derivative_d =
            quintic_polynomial_lateral->calculate_zeroth_derivative(
                frenet_path_time_i);
        double first_derivative_d =
            quintic_polynomial_lateral->calculate_first_derivative(
                frenet_path_time_i);
        double second_derivative_d =
            quintic_polynomial_lateral->calculate_second_derivative(
                frenet_path_time_i);
        double third_derivative_d =
            quintic_polynomial_lateral->calculate_third_derivative(
                frenet_path_time_i);

        frenet_path->pushBackDerivatives_d(
            zeroth_derivative_d, first_derivative_d, second_derivative_d,
            third_derivative_d);
      }

      double target_velocity_s = TARGET_SPEED - 0.0;
      while (target_velocity_s <= TARGET_SPEED + 0.0) {
        // double target_velocity_s = current_velocity_s - 0.0;
        // while (target_velocity_s <= current_velocity_s + 0.0) {
        std::shared_ptr<FrenetPath> frenet_path_target_velocity_s(
            new FrenetPath(*frenet_path));

        std::shared_ptr<QuarticPolynomial> quartic_polynomial_longitudinal(
            new QuarticPolynomial(current_position_s, current_velocity_s, 0.0,
                                  target_velocity_s, 0.0, time));

        const std::vector<double> &frenet_path_time = frenet_path->time();
        for (auto itr = frenet_path_time.begin(); itr != frenet_path_time.end();
             itr++) {
          double frenet_path_time_i = *itr;

          double zeroth_derivative_s =
              quartic_polynomial_longitudinal->calculate_zeroth_derivative(
                  frenet_path_time_i);
          double first_derivative_s =
              quartic_polynomial_longitudinal->calculate_first_derivative(
                  frenet_path_time_i);
          double second_derivative_s =
              quartic_polynomial_longitudinal->calculate_second_derivative(
                  frenet_path_time_i);
          double third_derivative_s =
              quartic_polynomial_longitudinal->calculate_third_derivative(
                  frenet_path_time_i);

          frenet_path_target_velocity_s->pushBackDerivatives_s(
              zeroth_derivative_s, first_derivative_s, second_derivative_s,
              third_derivative_s);
        }

        double jerk_d = 0;
        const std::vector<double> &third_derivative_d =
            frenet_path_target_velocity_s->third_derivative_d();
        for (auto itr = third_derivative_d.begin();
             itr != third_derivative_d.end(); itr++) {
          jerk_d += pow(*itr, 2);
        }

        double jerk_s = 0;
        const std::vector<double> &third_derivative_s =
            frenet_path_target_velocity_s->third_derivative_s();
        for (auto itr = third_derivative_s.begin();
             itr != third_derivative_s.end(); itr++) {
          jerk_s += pow(*itr, 2);
        }

        double delta_velocity =
            pow(TARGET_SPEED -
                    frenet_path_target_velocity_s->first_derivative_s().back(),
                2);

        // double delta_velocity =
        //     pow(current_velocity_s -
        //             frenet_path_target_velocity_s->first_derivative_s().back(),
        //         2);

        double cost_d = K_J * jerk_d + K_T * time +
                        K_D * pow(frenet_path->points_d().back(), 2);
        double cost_s = K_J * jerk_s + K_T * time + K_D * delta_velocity;
        double cost_total = K_LAT * cost_d + K_LON * cost_s;

        frenet_path_target_velocity_s->set_cost_d(cost_d);
        frenet_path_target_velocity_s->set_cost_s(cost_s);
        frenet_path_target_velocity_s->set_cost_total(cost_total);

        frenet_paths.push_back(frenet_path_target_velocity_s);

        target_velocity_s += D_T_S;
      }

      time += dt;
    }
    expected_position_d += width_d;
  }

  return frenet_paths;
}

void FrenetPathGenerator::calculate_global_paths(
    std::vector<std::shared_ptr<FrenetPath>> &frenet_paths,
    std::shared_ptr<CubicSpliner2D> &cubic_spliner_2D) {
  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath> &frenet_path = frenet_paths[i];

    // calculate x and y
    const std::vector<double> &points_s = frenet_path->points_s();
    for (int j = 0; j < frenet_path->points_s().size(); j++) {
      double point_s = points_s[j];

      std::tuple<double, double> position =
          cubic_spliner_2D->calculate_position(point_s);
      double point_x = std::get<0>(position);
      double point_y = std::get<1>(position);

      double yaw = cubic_spliner_2D->calculate_yaw(point_s);

      double point_d = frenet_path->points_d()[j];

      double frenet_x = point_x + point_d * cos(yaw + 3.14 / 2.0);
      double frenet_y = point_y + point_d * sin(yaw + 3.14 / 2.0);

      // if (frenet_x > 0.0) {
      frenet_path->push_back_point_y(frenet_y);
      frenet_path->push_back_point_x(frenet_x);
      // }
    }

    // calculate yaw and delta_velocity
    const std::vector<double> &points_x = frenet_path->points_x();
    const std::vector<double> &points_y = frenet_path->points_y();

    assert(points_x.size() == points_y.size());

    for (int j = 0; j < points_x.size() - 1; j++) {
      double delta_x = points_x[j + 1] - points_x[j];
      double delta_y = points_y[j + 1] - points_y[j];

      frenet_path->push_back_yaw(atan2(delta_y, delta_x));
      frenet_path->push_back_delta_velocity(
          sqrt(delta_x * delta_x + delta_y * delta_y));
    }
    frenet_path->push_back_yaw(frenet_path->yaw().back());
    frenet_path->push_back_delta_velocity(frenet_path->delta_velocity().back());

    // calculate curvature
    const std::vector<double> &yaw = frenet_path->yaw();
    const std::vector<double> &delta_velocity = frenet_path->delta_velocity();

    assert(yaw.size() == delta_velocity.size());

    for (int j = 0; j < yaw.size() - 1; j++) {
      double curvature = (yaw[j + 1] - yaw[j]) / delta_velocity[j];
      frenet_path->push_back_curvature(curvature);
    }
  }
}

std::tuple<std::shared_ptr<FrenetPath>,
           std::vector<std::shared_ptr<FrenetPath>>>
FrenetPathGenerator::calc_frenet_paths_v2(
    double current_position_d, double current_position_s,
    double current_velocity_d, double current_velocity_s,
    double current_acceleration_d,
    std::shared_ptr<CubicSpliner2D> &cubic_spliner_2D, double min_t = 2.0,
    double max_t = 4.01, double dt = 0.2, double left_margin = -1.0,
    double right_margin = 1.0, double width_d = 0.2, int num_speed_sample = 1) {

  std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
      generate_frenet_paths_v3(
          current_position_d, current_position_s, current_velocity_d,
          current_velocity_s, current_acceleration_d, min_t, max_t, dt,
          left_margin, right_margin, width_d, num_speed_sample);

  calculate_global_paths(frenet_paths, cubic_spliner_2D);

  if (frenet_paths.empty() == false) {
    std::shared_ptr<FrenetPath> optimal_frenet_path;

    // Find Frenet path with the minimum cost
    double min_cost_total = frenet_paths[0]->cost_total();
    optimal_frenet_path = frenet_paths[0];
    for (int i = 1; i < frenet_paths.size(); i++) {
      std::shared_ptr<FrenetPath> frenet_path = frenet_paths[i];

      if (min_cost_total > frenet_path->cost_total()) {
        min_cost_total = frenet_path->cost_total();
        optimal_frenet_path = frenet_path;
      }
    }

    return std::make_tuple(optimal_frenet_path, frenet_paths);
  } else {
    std::shared_ptr<FrenetPath> empty_path(new FrenetPath(-1.1, -1.0));
    return std::make_tuple(empty_path, frenet_paths);
  }
}
