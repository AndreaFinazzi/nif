#include "nif_opponent_prediction_nodes/frenet_path_generator.h"

#include <assert.h>

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
#define D_T_S 5.0 / 3.6         // target speed sampling length [m/s]
#define N_S_SAMPLE 0            // sampling number of target speed
#define ROBOT_RADIUS 2.0        // robot radius [m]

// cost weights
#define K_J 0.1
#define K_T 0.1
#define K_D 10 // 25
#define K_LAT 1.0
#define K_LON 1.0
#define K_GRID 100 // grid map cost gain

#define BLOCKING_COST_THRES 3

#define VISUALIZATION true

// math
#define PI 3.1415926535897

std::vector<std::shared_ptr<FrenetPath>>
FrenetPathGenerator::generate_frenet_paths(double current_position_d,
                                           double current_position_s,
                                           double current_velocity_d,
                                           double current_velocity_s,
                                           double current_acceleration_d,
                                           double min_t,
                                           double max_t,
                                           double dt) {
  std::vector<std::shared_ptr<FrenetPath>> frenet_paths;

  // double expected_position_d = -MAX_ROAD_WIDTH;
  double expected_position_d = 0.0;
  while (expected_position_d == 0.0) {
    double time = min_t;
    while (time < max_t) {
      std::shared_ptr<FrenetPath> frenet_path(new FrenetPath());

      std::shared_ptr<QuinticPolynomial> quintic_polynomial_lateral(
          new QuinticPolynomial(current_position_d,
                                current_velocity_d,
                                current_acceleration_d,
                                expected_position_d,
                                0.0,
                                0.0,
                                time));

      double frenet_path_time_i = 0.0;
      while (frenet_path_time_i < time) {
        frenet_path->push_back_time(frenet_path_time_i);
        frenet_path_time_i += dt;
      }

      const std::vector<double>& frenet_path_time = frenet_path->time();
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

        frenet_path->push_back_derivatives_d(zeroth_derivative_d,
                                             first_derivative_d,
                                             second_derivative_d,
                                             third_derivative_d);
      }

      double target_velocity_s = TARGET_SPEED - D_T_S * N_S_SAMPLE;
      while (target_velocity_s <= TARGET_SPEED + D_T_S * N_S_SAMPLE) {
        std::shared_ptr<FrenetPath> frenet_path_target_velocity_s(
            new FrenetPath(*frenet_path));

        std::shared_ptr<QuarticPolynomial> quartic_polynomial_longitudinal(
            new QuarticPolynomial(current_position_s,
                                  current_velocity_s,
                                  0.0,
                                  target_velocity_s,
                                  0.0,
                                  time));

        const std::vector<double>& frenet_path_time = frenet_path->time();
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

          frenet_path_target_velocity_s->push_back_derivatives_s(
              zeroth_derivative_s,
              first_derivative_s,
              second_derivative_s,
              third_derivative_s);
        }

        double jerk_d = 0;
        const std::vector<double>& third_derivative_d =
            frenet_path_target_velocity_s->third_derivative_d();
        for (auto itr = third_derivative_d.begin();
             itr != third_derivative_d.end();
             itr++) {
          jerk_d += pow(*itr, 2);
        }

        double jerk_s = 0;
        const std::vector<double>& third_derivative_s =
            frenet_path_target_velocity_s->third_derivative_s();
        for (auto itr = third_derivative_s.begin();
             itr != third_derivative_s.end();
             itr++) {
          jerk_s += pow(*itr, 2);
        }

        double delta_velocity =
            pow(TARGET_SPEED -
                    frenet_path_target_velocity_s->first_derivative_s().back(),
                2);

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

      time += DT;
    }
    expected_position_d += D_ROAD_W;
  }

  return frenet_paths;
}

std::vector<std::shared_ptr<FrenetPath>>
FrenetPathGenerator::generate_frenet_paths_v2(
    double current_position_d,
    double current_position_s,
    double current_velocity_d,
    double current_velocity_s,
    double current_acceleration_d,
    double min_t,
    double max_t,
    double dt,
    double left_margin = -MAX_ROAD_WIDTH,
    double right_margin = MAX_ROAD_WIDTH,
    double width_d = D_ROAD_W) {
  assert(left_margin >= 0);
  assert(right_margin <= 0);
  assert(width_d <= 0);
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
          new QuinticPolynomial(current_position_d,
                                current_velocity_d,
                                current_acceleration_d,
                                expected_position_d,
                                0.0,
                                0.0,
                                time));

      double frenet_path_time_i = 0.0;
      while (frenet_path_time_i < time) {
        frenet_path->push_back_time(frenet_path_time_i);
        frenet_path_time_i += dt;
      }

      const std::vector<double>& frenet_path_time = frenet_path->time();
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

        frenet_path->push_back_derivatives_d(zeroth_derivative_d,
                                             first_derivative_d,
                                             second_derivative_d,
                                             third_derivative_d);
      }

      double target_velocity_s = TARGET_SPEED - D_T_S * N_S_SAMPLE;
      while (target_velocity_s <= TARGET_SPEED + D_T_S * N_S_SAMPLE) {
        std::shared_ptr<FrenetPath> frenet_path_target_velocity_s(
            new FrenetPath(*frenet_path));

        std::shared_ptr<QuarticPolynomial> quartic_polynomial_longitudinal(
            new QuarticPolynomial(current_position_s,
                                  current_velocity_s,
                                  0.0,
                                  target_velocity_s,
                                  0.0,
                                  time));

        const std::vector<double>& frenet_path_time = frenet_path->time();
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

          frenet_path_target_velocity_s->push_back_derivatives_s(
              zeroth_derivative_s,
              first_derivative_s,
              second_derivative_s,
              third_derivative_s);
        }

        double jerk_d = 0;
        const std::vector<double>& third_derivative_d =
            frenet_path_target_velocity_s->third_derivative_d();
        for (auto itr = third_derivative_d.begin();
             itr != third_derivative_d.end();
             itr++) {
          jerk_d += pow(*itr, 2);
        }

        double jerk_s = 0;
        const std::vector<double>& third_derivative_s =
            frenet_path_target_velocity_s->third_derivative_s();
        for (auto itr = third_derivative_s.begin();
             itr != third_derivative_s.end();
             itr++) {
          jerk_s += pow(*itr, 2);
        }

        double delta_velocity =
            pow(TARGET_SPEED -
                    frenet_path_target_velocity_s->first_derivative_s().back(),
                2);

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
    std::vector<std::shared_ptr<FrenetPath>>& frenet_paths,
    std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D) {
  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath>& frenet_path = frenet_paths[i];

    if (!frenet_path->points_s().empty()) {
      // calculate x and y
      const std::vector<double>& points_s = frenet_path->points_s();
      for (int j = 0; j < frenet_path->points_s().size(); j++) {
        double point_s = points_s[j];

        std::tuple<double, double> position =
            cubic_spliner_2D->calculate_position(point_s);
        double point_x = std::get<0>(position);
        double point_y = std::get<1>(position);

        double yaw = cubic_spliner_2D->calculate_yaw(point_s);

        double point_d = frenet_path->points_d()[j];

        double frenet_x = point_x + point_d * cos(yaw + PI / 2.0);
        double frenet_y = point_y + point_d * sin(yaw + PI / 2.0);

        // if (frenet_x > 0.0) {
        frenet_path->push_back_point_y(frenet_y);
        frenet_path->push_back_point_x(frenet_x);
        // }
      }

      // calculate yaw and delta_velocity
      const std::vector<double>& points_x = frenet_path->points_x();
      const std::vector<double>& points_y = frenet_path->points_y();

      assert(points_x.size() == points_y.size());

      for (int j = 0; j < points_x.size() - 1; j++) {
        double delta_x = points_x[j + 1] - points_x[j];
        double delta_y = points_y[j + 1] - points_y[j];

        frenet_path->push_back_yaw(atan2(delta_y, delta_x));
        frenet_path->push_back_delta_velocity(
            sqrt(delta_x * delta_x + delta_y * delta_y));
      }
      frenet_path->push_back_yaw(frenet_path->yaw().back());
      frenet_path->push_back_delta_velocity(
          frenet_path->delta_velocity().back());

      // calculate curvature
      const std::vector<double>& yaw = frenet_path->yaw();
      const std::vector<double>& delta_velocity = frenet_path->delta_velocity();

      assert(yaw.size() == delta_velocity.size());

      for (int j = 0; j < yaw.size() - 1; j++) {
        double curvature = (yaw[j + 1] - yaw[j]) / delta_velocity[j];
        frenet_path->push_back_curvature(curvature);
      }
    }
  }
}

void FrenetPathGenerator::calculate_global_paths_w_current_pose(
    std::vector<std::shared_ptr<FrenetPath>>& frenet_paths,
    std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D,
    double current_yaw) {
  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath>& frenet_path = frenet_paths[i];

    // calculate x and y
    const std::vector<double>& points_s = frenet_path->points_s();
    for (int j = 0; j < frenet_path->points_s().size(); j++) {
      double point_s = points_s[j];

      std::tuple<double, double> position =
          cubic_spliner_2D->calculate_position(point_s);
      double point_x = std::get<0>(position);
      double point_y = std::get<1>(position);

      double yaw = cubic_spliner_2D->calculate_yaw(point_s);

      double point_d = frenet_path->points_d()[j];

      double frenet_x = point_x + point_d * cos(yaw + PI / 2.0);
      double frenet_y = point_y + point_d * sin(yaw + PI / 2.0);

      if (frenet_x > 0.0) {
        frenet_path->push_back_point_x(frenet_x);
        frenet_path->push_back_point_y(frenet_y);
      }
    }

    // calculate yaw and delta_velocity
    const std::vector<double>& points_x = frenet_path->points_x();
    const std::vector<double>& points_y = frenet_path->points_y();

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
    const std::vector<double>& yaw = frenet_path->yaw();
    const std::vector<double>& delta_velocity = frenet_path->delta_velocity();

    assert(yaw.size() == delta_velocity.size());

    for (int j = 0; j < yaw.size() - 1; j++) {
      double curvature = (yaw[j + 1] - yaw[j]) / delta_velocity[j];
      frenet_path->push_back_curvature(curvature);
    }
  }
}

bool FrenetPathGenerator::check_frenet_path_collision(
    std::shared_ptr<FrenetPath>& frenet_path,
    std::vector<std::tuple<double, double>>& obstacles) {
  bool is_collision = false;

  for (int i = 0; i < int(obstacles.size()); i++) {
    double obstacle_x = std::get<0>(obstacles[i]);
    double obstacle_y = std::get<0>(obstacles[i]);

    const std::vector<double>& points_x = frenet_path->points_x();
    const std::vector<double>& points_y = frenet_path->points_y();

    for (int j = 0; j < int(points_x.size()); j++) {
      double distance =
          pow(points_x[j] - obstacle_x, 2) + pow(points_y[j] - obstacle_y, 2);

      if (distance < pow(ROBOT_RADIUS, 2)) {
        is_collision = true;
        break;
      }
    }
  }

  return is_collision;
}

void FrenetPathGenerator::check_frenet_paths_validity_w_constraint(
    std::vector<std::shared_ptr<FrenetPath>>& frenet_paths) {
  std::vector<std::shared_ptr<FrenetPath>> valid_frenet_paths;

  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath> frenet_path = frenet_paths[i];

    const std::vector<double>& first_derivative_s =
        frenet_path->first_derivative_s();
    const std::vector<double>& second_derivative_s =
        frenet_path->second_derivative_s();

    if (std::any_of(first_derivative_s.begin(),
                    first_derivative_s.end(),
                    [](double velocity) { return velocity > MAX_SPEED; })) {
      continue;
    } else if (std::any_of(second_derivative_s.begin(),
                           second_derivative_s.end(),
                           [](double acceleration) {
                             return abs(acceleration) > MAX_ACCEL;
                           })) {
      continue;
    }

    valid_frenet_paths.push_back(frenet_path);
  }
}

void FrenetPathGenerator::check_frenet_paths_validity(
    std::vector<std::shared_ptr<FrenetPath>>& frenet_paths,
    std::vector<std::tuple<double, double>>& obstacles) {
  std::vector<std::shared_ptr<FrenetPath>> valid_frenet_paths;

  for (int i = 0; i < frenet_paths.size(); i++) {
    std::shared_ptr<FrenetPath> frenet_path = frenet_paths[i];

    const std::vector<double>& first_derivative_s =
        frenet_path->first_derivative_s();
    const std::vector<double>& second_derivative_s =
        frenet_path->second_derivative_s();

    if (std::any_of(first_derivative_s.begin(),
                    first_derivative_s.end(),
                    [](double velocity) { return velocity > MAX_SPEED; })) {
      continue;
    } else if (std::any_of(second_derivative_s.begin(),
                           second_derivative_s.end(),
                           [](double acceleration) {
                             return abs(acceleration) > MAX_ACCEL;
                           })) {
      continue;
    } else if (check_frenet_path_collision(frenet_path, obstacles)) {
      continue;
    }

    valid_frenet_paths.push_back(frenet_path);
  }
}

std::tuple<std::shared_ptr<FrenetPath>,
           std::vector<std::shared_ptr<FrenetPath>>>
FrenetPathGenerator::calculate_optimal_frenet_path(
    double current_position_d,
    double current_position_s,
    double current_velocity_d,
    double current_velocity_s,
    double current_acceleration_d,
    std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D,
    std::vector<std::tuple<double, double>>& obstacles,
    double min_t = 2.0,
    double max_t = 4.01,
    double dt = 0.2) {
  std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
      generate_frenet_paths(current_position_d,
                            current_position_s,
                            current_velocity_d,
                            current_velocity_s,
                            current_acceleration_d,
                            min_t,
                            max_t,
                            dt);

  calculate_global_paths(frenet_paths, cubic_spliner_2D);
  check_frenet_paths_validity(frenet_paths, obstacles);

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

std::tuple<std::shared_ptr<FrenetPath>,
           std::vector<std::shared_ptr<FrenetPath>>>
FrenetPathGenerator::calc_frenet_paths(
    double current_position_d,
    double current_position_s,
    double current_velocity_d,
    double current_velocity_s,
    double current_acceleration_d,
    std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D,
    double min_t = 2.0,
    double max_t = 4.01,
    double dt = 0.2,
    double left_margin = -1.0,
    double right_margin = 1.0,
    double width_d = 0.2) {
  std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
      generate_frenet_paths_v2(current_position_d,
                               current_position_s,
                               current_velocity_d,
                               current_velocity_s,
                               current_acceleration_d,
                               min_t,
                               max_t,
                               dt,
                               left_margin,
                               right_margin,
                               width_d);

  // std::vector<std::shared_ptr<FrenetPath>> frenet_paths =
  //     generate_frenet_paths_v2(current_position_d, current_position_s,
  //                              current_velocity_d, current_velocity_s,
  //                              current_acceleration_d, min_t, max_t, dt);

  calculate_global_paths(frenet_paths, cubic_spliner_2D);

  if (frenet_paths.empty() == false) {
    std::shared_ptr<FrenetPath> optimal_frenet_path;

    // Find Frenet path with the minimum cost
    double min_cost_total = frenet_paths[0]->cost_total();
    optimal_frenet_path = frenet_paths[0];
    for (int i = 0; i < frenet_paths.size(); i++) {
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

FrenetPathGenerator::CubicSpliner2DResult
FrenetPathGenerator::apply_cubic_spliner(std::vector<double>& points_x,
                                         std::vector<double>& points_y,
                                         double spline_interval = 1.0) {
  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
      new CubicSpliner2D(points_x, points_y));

  std::vector<double> cubic_spliner_x;
  std::vector<double> cubic_spliner_y;
  std::vector<double> cubic_spliner_yaw;
  std::vector<double> cubic_spliner_curvature;

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

    point_s += spline_interval;
  }
  return std::make_tuple(cubic_spliner_x,
                         cubic_spliner_y,
                         cubic_spliner_yaw,
                         cubic_spliner_curvature,
                         cubic_spliner_2D);
}

FrenetPathGenerator::CubicSpliner2DResult
FrenetPathGenerator::apply_cubic_spliner_w_current_pose(
    std::vector<double>& points_x,
    std::vector<double>& points_y,
    double cur_heading_rad) {
  std::shared_ptr<CubicSpliner2D> cubic_spliner_2D(
      new CubicSpliner2D(points_x, points_y));

  std::vector<double> cubic_spliner_x;
  std::vector<double> cubic_spliner_y;
  std::vector<double> cubic_spliner_yaw;
  std::vector<double> cubic_spliner_curvature;

  double point_s = 0.0;
  double point_s_end = cubic_spliner_2D->points_s().back();
  while (point_s < point_s_end) {
    std::tuple<double, double> position =
        cubic_spliner_2D->calculate_position(point_s);
    double point_x = std::get<0>(position);
    double point_y = std::get<1>(position);

    // + - sign should be checked
    // double yaw = cubic_spliner_2D->calculate_yaw(point_s) + cur_heading_rad;
    double yaw = cubic_spliner_2D->calculate_yaw(point_s);

    double curvature = cubic_spliner_2D->calculate_curvature(point_s);

    cubic_spliner_x.push_back(point_x);
    cubic_spliner_y.push_back(point_y);
    cubic_spliner_yaw.push_back(yaw);
    cubic_spliner_curvature.push_back(curvature);

    // point_s += 0.1;
    point_s += 1;
  }
  return std::make_tuple(cubic_spliner_x,
                         cubic_spliner_y,
                         cubic_spliner_yaw,
                         cubic_spliner_curvature,
                         cubic_spliner_2D);
}