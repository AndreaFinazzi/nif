#include "nif_opponent_prediction_nodes/frenet_path.h"

#include <assert.h>

FrenetPath::FrenetPath(FrenetPath &frenet_path)
    : points_d_(frenet_path.points_d()),
      first_derivative_d_(frenet_path.first_derivative_d()),
      second_derivative_d_(frenet_path.second_derivative_d()),
      third_derivative_d_(frenet_path.third_derivative_d()),
      points_s_(frenet_path.points_s()),
      first_derivative_s_(frenet_path.first_derivative_s()),
      second_derivative_s_(frenet_path.second_derivative_s()),
      third_derivative_s_(frenet_path.third_derivative_s()),
      time_(frenet_path.time()), points_x_(frenet_path.points_x()),
      points_y_(frenet_path.points_y()), yaw_(frenet_path.yaw()),
      curvature_(frenet_path.curvature()),
      delta_velocity_(frenet_path.delta_velocity()),
      cost_d_(frenet_path.cost_d()), cost_s_(frenet_path.cost_s()),
      cost_total_(frenet_path.cost_total()),
      blocking_flg_(frenet_path.blocking_flg()),
      blocking_idx_(frenet_path.blocking_idx()) {}

void FrenetPath::push_back_derivatives_d(double zeroth_derivative,
                                         double first_derivative,
                                         double second_derivative,
                                         double third_derivative)
{
  points_d_.push_back(zeroth_derivative);
  first_derivative_d_.push_back(first_derivative);
  second_derivative_d_.push_back(second_derivative);
  third_derivative_d_.push_back(third_derivative);
}

void FrenetPath::push_back_derivatives_s(double zeroth_derivative,
                                         double first_derivative,
                                         double second_derivative,
                                         double third_derivative)
{
  points_s_.push_back(zeroth_derivative);
  first_derivative_s_.push_back(first_derivative);
  second_derivative_s_.push_back(second_derivative);
  third_derivative_s_.push_back(third_derivative);
}