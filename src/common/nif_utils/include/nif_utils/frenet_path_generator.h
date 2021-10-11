#ifndef FRENET_PATH_GENERATOR_H_
#define FRENET_PATH_GENERATOR_H_

#include <assert.h>
#include <float.h>
#include <memory>
#include <tuple>
#include <vector>

#include "cubic_spliner_2D.h"
#include "frenet_path.h"
#include "nav_msgs/msg/path.hpp"
#include "quartic_polynomial.h"
#include "quintic_polynomial.h"

namespace Frenet
{

    enum FRENET_GEN_MODE
    {
        ONLY_SINGLE_FP,
        MULTIPLE_LONG_FPS,
        MULTIPLE_LAT_FPS,
        MULTIPLE_LATLONG_FPS,
    };

    class FrenetPathGenerator
    {
    public:
        typedef std::tuple<std::vector<double>, std::vector<double>,
                           std::vector<double>, std::shared_ptr<CubicSpliner2D>>
            CubicSpliner2DResult;

        typedef std::tuple<std::vector<double>, std::vector<double>,
                           std::vector<double>, std::vector<double>,
                           std::shared_ptr<CubicSpliner2D>>
            CubicSpliner3DResult;

        FrenetPathGenerator() {}
        FrenetPathGenerator(string path_contraints_yaml_file_path,
                            string cost_param_yaml_file_path);

        ~FrenetPathGenerator() {}

        void loadPathConstraints(string path_contraints_yaml_file_path);
        void loadCostParam(string cost_param_yaml_file_path);

        std::tuple<std::shared_ptr<FrenetPath>,
                   std::vector<std::shared_ptr<FrenetPath>>>
        calcOptimalFrenetPath(double current_position_d, double current_position_s,
                              double current_velocity_d, double current_velocity_s,
                              double current_acceleration_d,
                              std::shared_ptr<CubicSpliner2D> &cubic_spliner_2D,
                              std::vector<std::tuple<double, double>> &obstacles,
                              double min_t, double max_t, double dt);

        std::tuple<std::shared_ptr<FrenetPath>,
                   std::vector<std::shared_ptr<FrenetPath>>>
        calcOptimalFrenetPathByMode(
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
            double target_acceleration_d = 0.0, double target_acceleration_s = 0.0,
            double planning_sampling_d = 1.0, double planning_t = 2.0,
            double planning_min_t = 2.0, double planning_max_t = 4.0,
            double planning_sampling_t = 1.0, double dt = 0.2);

        CubicSpliner2DResult applyCubicSpliner_2d(std::vector<double> &points_x,
                                                  std::vector<double> &points_y,
                                                  double sampling_interval = 0.1);
        CubicSpliner2DResult applyCubicSpliner_2d_ros(nav_msgs::msg::Path &path_body,
                                                      double sampling_interval = 0.1);
        CubicSpliner3DResult applyCubicSpliner_3d(std::vector<double> &points_x,
                                                  std::vector<double> &points_y,
                                                  std::vector<double> &points_z,
                                                  double sampling_interval = 0.1);

        std::shared_ptr<FrenetPath>
        genSingleFrenetPath(double ref_splined_end_s, double current_position_d,
                            double current_position_s, double current_velocity_d,
                            double current_velocity_s, double current_acceleration_d,
                            double current_acceleration_s, double target_position_d,
                            double target_velocity_d, double target_velocity_s,
                            double target_acceleration_d = 0.0,
                            double target_acceleration_s = 0.0,
                            double planning_t = 2.0, double dt = 0.2);

        std::vector<std::shared_ptr<FrenetPath>> genMultipleLongiFrenetPaths(
            double ref_splined_end_s, double current_position_d,
            double current_position_s, double current_velocity_d,
            double current_velocity_s, double current_acceleration_d,
            double current_acceleration_s, double target_position_d,
            double target_velocity_d, double target_velocity_s,
            double target_acceleration_d = 0.0, double target_acceleration_s = 0.0,
            double planning_min_t = 2.0, double planning_max_t = 4.0,
            double planning_sampling_t = 1.0, double dt = 0.2);

        std::vector<std::shared_ptr<FrenetPath>> genMultipleLateralFrenetPaths(
            double ref_splined_end_s, double current_position_d,
            double current_position_s, double current_velocity_d,
            double current_velocity_s, double current_acceleration_d,
            double current_acceleration_s,
            // sign convention : left side to the reference path is negative
            double target_position_min_d, // most left side
            double target_position_max_d, // most right side
            double target_velocity_d, double target_velocity_s,
            double target_acceleration_d = 0.0, double target_acceleration_s = 0.0,
            double planning_sampling_d = 1.0, double planning_t = 2.0,
            double dt = 0.2);

        std::vector<std::shared_ptr<FrenetPath>> genMultipleLateralLongiFrenetPaths(
            double ref_splined_end_s, double current_position_d,
            double current_position_s, double current_velocity_d,
            double current_velocity_s, double current_acceleration_d,
            double current_acceleration_s,
            // sign convention : left side to the reference path is negative
            double target_position_min_d, // most left side
            double target_position_max_d, // most right side
            double target_velocity_d, double target_velocity_s,
            double target_acceleration_d = 0.0, double target_acceleration_s = 0.0,
            double planning_sampling_d = 1.0, double planning_min_t = 2.0,
            double planning_max_t = 4.0, double planning_sampling_t = 1.0,
            double dt = 0.2);

        void convertFrenetPathsInGlobal(
            std::vector<std::shared_ptr<FrenetPath>> &frenet_paths,
            std::shared_ptr<CubicSpliner2D> &cubic_spliner_2D);
        bool checkCollision(std::shared_ptr<FrenetPath> &frenet_path,
                            std::vector<std::tuple<double, double>> &obstacles);
        void checkValidity_w_obstalceMap(
            std::vector<std::shared_ptr<FrenetPath>> &frenet_paths,
            std::vector<std::tuple<double, double>> &obstacles);
        void checkValidity_w_constraint(
            std::vector<std::shared_ptr<FrenetPath>> &frenet_paths);

        void
        calculate_global_paths(std::vector<std::shared_ptr<FrenetPath>> &frenet_paths,
                               std::shared_ptr<CubicSpliner2D> &cubic_spliner_2D);

        std::vector<std::shared_ptr<FrenetPath>>
        generate_frenet_paths_v3(double current_position_d, double current_position_s,
                                 double current_velocity_d, double current_velocity_s,
                                 double current_acceleration_d, double min_t,
                                 double max_t, double dt, double left_margin,
                                 double right_margin, double width_d,
                                 int num_speed_sample);

        std::tuple<std::shared_ptr<FrenetPath>,
                   std::vector<std::shared_ptr<FrenetPath>>>
        calc_frenet_paths_v2(double current_position_d, double current_position_s,
                             double current_velocity_d, double current_velocity_s,
                             double current_acceleration_d,
                             std::shared_ptr<CubicSpliner2D> &cubic_spliner_2D,
                             double min_t, double max_t, double dt,
                             double left_margin, double right_margin, double width_d,
                             int num_speed_sample);

    protected:
        // path constraints
        double constraint_param_max_speed;
        double constraint_param_max_accel;
        double constraint_param_max_curvature;
        double constraint_param_robot_radius;

        // cost weights
        double cost_param_k_j;
        double cost_param_k_t;
        double cost_param_k_d;
        double cost_param_k_lat;
        double cost_param_k_lon;

        string m_path_contraints_yaml_file_path;
        string m_cost_param_yaml_file_path;
    };
} // namespace Frenet

#endif // FRENET_PATH_GENERATOR_H_