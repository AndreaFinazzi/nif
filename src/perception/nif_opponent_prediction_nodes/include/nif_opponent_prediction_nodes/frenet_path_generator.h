#ifndef FRENET_PATH_GENERATOR_H_
#define FRENET_PATH_GENERATOR_H_

#include "nav_msgs/msg/path.hpp"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>
#include <float.h>
#include <memory>
#include <tuple>
#include <vector>

#include "cubic_spliner_2D.h"
#include "frenet_path.h"

class FrenetPathGenerator {
public:
  typedef std::tuple<std::vector<double>,
                     std::vector<double>,
                     std::vector<double>,
                     std::vector<double>,
                     std::shared_ptr<CubicSpliner2D>>
      CubicSpliner2DResult;

  typedef std::tuple<std::vector<double>,
                     std::vector<double>,
                     std::vector<double>,
                     std::vector<double>,
                     std::vector<double>,
                     std::shared_ptr<CubicSpliner2D>>
      CubicSpliner2DResult_w_progress;

  FrenetPathGenerator() {}
  ~FrenetPathGenerator() {}

  std::tuple<std::shared_ptr<FrenetPath>,
             std::vector<std::shared_ptr<FrenetPath>>>
  calculate_optimal_frenet_path(
      double current_position_d,
      double current_position_s,
      double current_velocity_d,
      double current_velocity_s,
      double current_acceleration_d,
      std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D,
      std::vector<std::tuple<double, double>>& obstacles,
      double min_t,
      double max_t,
      double dt);

  std::tuple<std::shared_ptr<FrenetPath>,
             std::vector<std::shared_ptr<FrenetPath>>>
  calc_frenet_paths(double current_position_d,
                    double current_position_s,
                    double current_velocity_d,
                    double current_velocity_s,
                    double current_acceleration_d,
                    std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D,
                    double min_t,
                    double max_t,
                    double dt,
                    double left_margin,
                    double right_margin,
                    double width_d);

  CubicSpliner2DResult apply_cubic_spliner(std::vector<double>& points_x,
                                           std::vector<double>& points_y,
                                           double spline_interval);

  CubicSpliner2DResult_w_progress
  apply_cubic_spliner_from_nav_path(nav_msgs::msg::Path& path_,
                                    double spline_interval);

  CubicSpliner2DResult
  apply_cubic_spliner_w_current_pose(std::vector<double>& points_x,
                                     std::vector<double>& points_y,
                                     double cur_heading_rad);

protected:
  std::vector<std::shared_ptr<FrenetPath>>
  generate_frenet_paths(double current_position_d,
                        double current_position_s,
                        double current_velocity_d,
                        double current_velocity_s,
                        double current_acceleration_d,
                        double min_t,
                        double max_t,
                        double dt);

  std::vector<std::shared_ptr<FrenetPath>>
  generate_frenet_paths_v2(double current_position_d,
                           double current_position_s,
                           double current_velocity_d,
                           double current_velocity_s,
                           double current_acceleration_d,
                           double min_t,
                           double max_t,
                           double dt,
                           double left_margin,
                           double right_margin,
                           double width_d);

  void
  calculate_global_paths(std::vector<std::shared_ptr<FrenetPath>>& frenet_paths,
                         std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D);

  void calculate_global_paths_w_current_pose(
      std::vector<std::shared_ptr<FrenetPath>>& frenet_paths,
      std::shared_ptr<CubicSpliner2D>& cubic_spliner_2D,
      double current_yaw);

  bool check_frenet_path_collision(
      std::shared_ptr<FrenetPath>& frenet_path,
      std::vector<std::tuple<double, double>>& obstacles);
  void check_frenet_paths_validity(
      std::vector<std::shared_ptr<FrenetPath>>& frenet_paths,
      std::vector<std::tuple<double, double>>& obstacles);

  void check_frenet_paths_validity_w_constraint(
      std::vector<std::shared_ptr<FrenetPath>>& frenet_paths);
};

#endif // FRENET_PATH_GENERATOR_H_