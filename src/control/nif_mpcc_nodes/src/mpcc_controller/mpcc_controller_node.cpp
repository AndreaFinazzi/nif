// // Copyright 2019 Alexander Liniger

// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at

// //     http://www.apache.org/licenses/LICENSE-2.0

// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.
// ///////////////////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////

#include <math.h>
#include <memory>
#include "mpcc_controller/Tests/constratins_test.h"
#include "mpcc_controller/Tests/cost_test.h"
#include "mpcc_controller/Tests/model_integrator_test.h"
#include "mpcc_controller/Tests/spline_test.h"
#include "mpcc_controller/MPC/mpc.h"
#include "mpcc_controller/Model/integrator.h"
#include "mpcc_controller/Params/track.h"
#include <nlohmann/json.hpp>

#include "low_pass_filter.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/brake2_report.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <pwd.h>
#include <sys/types.h>
#include <unistd.h>

#include "nif_common/constants.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

struct passwd *pw = getpwuid(getuid());

const char *homedir = pw->pw_dir;

using std::placeholders::_1;
using std::placeholders::_2;
using namespace mpcc;

rclcpp::Node::SharedPtr g_node = nullptr;

// class MPC;
struct EulerAngles
{
  double roll, pitch, yaw;
};

geometry_msgs::msg::Quaternion
ToQuaternion(double yaw, double pitch,
             double roll) // yaw (Z), pitch (Y), roll (X)
{
  // Abbreviations for the various angular functions
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

EulerAngles ToEulerAngles(geometry_msgs::msg::Quaternion q)
{
  EulerAngles angles;

  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
  double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
  angles.roll = std::atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q.w * q.y - q.z * q.x);
  if (std::abs(sinp) >= 1)
    angles.pitch =
        std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    angles.pitch = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  angles.yaw = std::atan2(siny_cosp, cosy_cosp);

  return angles;
}

class MpccRos : public rclcpp::Node
{
public:
  MpccRos() : Node("mpcc_controller")
  {
    mpcc_dir = ament_index_cpp::get_package_share_directory("mpcc_ros2") + "/";

    std::string mpcc_config_file_default = mpcc_dir + "Params/config.json";

    this->declare_parameter<std::string>("mpc_config_file", mpcc_config_file_default);

    this->declare_parameter<bool>("use_mpc_planner", false);
    this->declare_parameter<bool>("use_game", false);
    this->declare_parameter<double>("max_brake_pressure", 100.0);
    this->declare_parameter<double>("min_brake_pressure", 10.0);
    this->declare_parameter<double>("steer_to_SWA", 140 / 14.4);
    this->declare_parameter<std::string>("map_frame_id", "map");
    this->declare_parameter<std::string>("ego_frame_id", "center_of_gravity");

    this->get_parameter("mpc_config_file", mpc_config_file_);

    this->get_parameter("use_mpc_planner", use_mpc_planner_);
    this->get_parameter("use_game", use_game_);
    this->get_parameter("max_brake_pressure", max_brake_pressure_);
    this->get_parameter("min_brake_pressure", min_brake_pressure_);
    this->get_parameter("map_frame_id", map_frame_id_);
    this->get_parameter("ego_frame_id", ego_frame_id_);
    this->get_parameter("steer_to_SWA", steer_to_SWA_);

    use_test_sim_ = false;
    steer_to_SWA_ = 140 / 14.4;

    // Sub & Pub
    ego_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/localization/ekf/odom", rclcpp::QoS(1),
        std::bind(&MpccRos::stateCallback, this, _1));
    steering_report_sub_ =
        this->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
            "/raptor_dbw_interface/steering_report", rclcpp::QoS(1),
            std::bind(&MpccRos::steeringReportCallback, this, _1));
    accel_pedal_report_sub_ =
        this->create_subscription<raptor_dbw_msgs::msg::AcceleratorPedalReport>(
            "/raptor_dbw_interface/accelerator_report", rclcpp::QoS(1),
            std::bind(&MpccRos::accelPedalReportCallback, this, _1));
    brake_2_report_sub_ =
        this->create_subscription<raptor_dbw_msgs::msg::Brake2Report>(
            "/raptor_dbw_interface/brake_2_report", rclcpp::QoS(1),
            std::bind(&MpccRos::brake2ReportCallback, this, _1));
    planning_output_sub_ =
        this->create_subscription<nav_msgs::msg::Path>(
            "/planning/path_global", nif::common::constants::QOS_PLANNING,
            std::bind(&MpccRos::planningPathCallback, this, _1));

    control_pub_ =
        this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "mpc/control", rclcpp::QoS(1));
    pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("mpc/sim_pose",
                                                                rclcpp::QoS(1));
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "debug/mpc/center_path", rclcpp::QoS(1));
    bound_in_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "debug/mpc/inner_bound", rclcpp::QoS(1));
    bound_out_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "debug/mpc/outer_bound", rclcpp::QoS(1));
    sol_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "debug/mpc/solution_trajectory", rclcpp::QoS(1));
    track_boundary_in_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "debug/mpc/track_boundary_in", rclcpp::QoS(1));
    track_boundary_out_pub_ = this->create_publisher<nav_msgs::msg::Path>(
        "debug/mpc/track_boundary_out", rclcpp::QoS(1));
    visualization_publisher =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "debug/mpc/considering_pathes", rclcpp::QoS(1));

    mpcInit();
  }

  void visualize_multiple_paths(
      std::vector<ArcLengthSpline> &paths,
      std::vector<Eigen::VectorXd> &paths_phi,
      rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
          visualization_publisher,
      double r, double g, double b)
  {

    visualization_msgs::msg::MarkerArray marker_array;
    for (int i = 0; i < paths.size(); i++)
    {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      // marker.header.stamp = rclcpp::Time();
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.orientation.w = 1;
      marker.scale.x = 1;

      marker.color.a = 1;
      marker.color.r = 1.0 * (i + 1.0) / double(paths.size());
      marker.color.b = 0;
      marker.color.g = 0;
      // marker.lifetime = ros::Duration(3);

      for (int j = 0; j < paths[i].path_data_.X.size(); j++)
      {
        geometry_msgs::msg::Point point;
        point.x = paths[i].path_data_.X[j];
        point.y = paths[i].path_data_.Y[j];
        marker.points.push_back(point);
      }
      marker_array.markers.push_back(marker);
    }
    if (paths.size() == 0)
    {
      // get rid of red stripes if there is no opponent
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 0;
      marker.pose.position.y = 0;
      marker.pose.orientation.w = 1;
      marker.scale.x = 1;

      marker.color.a = 1;
      marker.color.r = 1.0;
      marker.color.b = 0;
      marker.color.g = 0;

      geometry_msgs::msg::Point point;
      marker.points.clear();
      marker.points.push_back(point);

      marker_array.markers.clear();
      marker_array.markers.push_back(marker);
    }
    visualization_publisher->publish(marker_array);
  }

  void mpcInit()
  {
    std::string ns = MpccRos::get_namespace();
    std::string file = mpc_config_file_;

    std::ifstream iConfig(mpc_config_file_.c_str());

    RCLCPP_INFO(rclcpp::get_logger("mpcc_controller"), "Openning config at %s",
                file.c_str());

    iConfig >> jsonConfig;
    PathToJson json_paths{mpcc_dir + std::string(jsonConfig["model_path"]),
                          mpcc_dir + std::string(jsonConfig["cost_path"]),
                          mpcc_dir + std::string(jsonConfig["bounds_path"]),
                          mpcc_dir + std::string(jsonConfig["track_path"]),
                          mpcc_dir +
                              std::string(jsonConfig["normalization_path"])};

    ////////////////////////////////////////
    /////// MPC class initialization ///////
    ////////////////////////////////////////

    Track track = Track(json_paths.track_path);
    TrackPos track_xy = track.getTrack();
    // Track track_inital_left = Track(json_paths_initial_left.track_path);
    // TrackPos track_xy_inital_left = track_inital_left.getTrack();
    // Track track_inital_right = Track(json_paths_initial_right.track_path);
    // TrackPos track_xy_inital_right = track_inital_right.getTrack();
    // Track track_m25 = Track(json_paths_m25.track_path);
    // TrackPos track_xy_m25 = track_m25.getTrack();
    // Track track_m40 = Track(json_paths_m40.track_path);
    // TrackPos track_xy_m40 = track_m40.getTrack();

    mpc = std::make_shared<MPC>(jsonConfig["n_sqp"], jsonConfig["n_reset"],
                                jsonConfig["sqp_mixing"], jsonConfig["Ts"],
                                json_paths);

    phi_0 = std::atan2(track_xy.Y(1) - track_xy.Y(0),
                       track_xy.X(1) - track_xy.X(0));

    x0 = {track_xy.X(0),
          track_xy.Y(0),
          phi_0,
          jsonConfig["v0"],
          0,
          0,
          0,
          0.5,
          0,
          jsonConfig["v0"]};

    ////////////////////////////////////////////
    /////// End MPC class initialization ///////
    ////////////////////////////////////////////

    mpc->ns_ = ns;

    Ts_ = jsonConfig["Ts"];
    sim_Ts_ = Ts_;

    center_path_.header.frame_id = map_frame_id_.c_str();
    bound_in_.header.frame_id = map_frame_id_.c_str();
    bound_out_.header.frame_id = map_frame_id_.c_str();
    sol_trajectory_.header.frame_id = map_frame_id_.c_str();
    track_boundary_in_.header.frame_id = map_frame_id_.c_str();
    track_boundary_out_.header.frame_id = map_frame_id_.c_str();

    center_path_.poses.clear();
    for (int i = 0; i < track_xy.X.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = track_xy.X(i);
      pose.pose.position.y = track_xy.Y(i);

      center_path_.poses.push_back(pose);
    }
    bound_in_.poses.clear();
    for (int i = 0; i < track_xy.X_inner.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = track_xy.X_inner(i);
      pose.pose.position.y = track_xy.Y_inner(i);

      bound_in_.poses.push_back(pose);
    }
    bound_out_.poses.clear();
    for (int i = 0; i < track_xy.X_outer.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = track_xy.X_outer(i);
      pose.pose.position.y = track_xy.Y_outer(i);

      bound_out_.poses.push_back(pose);
    }

    if (use_test_sim_)
    {
      runTestSim();
    }

    std::cout << "MPC init end" << std::endl;
  }

  void stateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    x_.X = msg->pose.pose.position.x;
    x_.Y = msg->pose.pose.position.y;

    EulerAngles angles;
    angles = ToEulerAngles(msg->pose.pose.orientation);

    x_.phi = angles.yaw;
    x_.vx = msg->twist.twist.linear.x;
    x_.vy = msg->twist.twist.linear.y;
    x_.r = msg->twist.twist.angular.z;
    x_.vs = sqrt(x_.vx * x_.vx + x_.vy * x_.vy); // u_.dVs;

    // NOTE : solve mpc problem when the planned output is subscribed. Moved to planningPathCallback
    // runControlLoop(x_);
    // publishControl(u_);
    // publishTrack();
  }

  /**
   * @brief Convert SWA to front wheel angle in radians.
   * @todo check units/values
   * @todo check steer_to_SWA_
   */
  void steeringReportCallback(
      const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg)
  {
    x_.delta = msg->steering_wheel_angle / steer_to_SWA_;
  }

  /**
   * @brief Assign the accel pedal value as throttle. 0 when brake is also 0
   * @todo check if pedal_output is the current position of pedal
   * @todo check if the value is between 0 - 1
   */
  void accelPedalReportCallback(
      const raptor_dbw_msgs::msg::AcceleratorPedalReport::SharedPtr msg)
  {
    accel_value_ = msg->pedal_output * 0.01;
    if (accel_value_ > 1 || accel_value_ < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("mpcc_controller"),
                  "Accel pedal value invalid(larger than 1 or smaller than 0)");
    }
    if (accel_value_ > 0)
    {
      x_.D = accel_value_;
    }
    else if (brake_value_ == 0)
    {
      x_.D = 0; // coasting
    }
  }

  /**
   * @brief Assign the brake value as negative throttle. 0 when accel is also 0
   * @todo check if the value is between 0 - 1
   * @todo uncomment the warning
   */
  void brake2ReportCallback(
      const raptor_dbw_msgs::msg::Brake2Report::SharedPtr msg)
  {
    double average_brake_pressure =
        (msg->front_brake_pressure + msg->rear_brake_pressure) / 2.0;
    double brake_ratio = (average_brake_pressure - min_brake_pressure_) /
                         (max_brake_pressure_ - min_brake_pressure_);
    brake_value_ = brake_ratio;
    // if (brake_value_ > 1 || brake_value_ < 0)
    if (brake_value_ < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("mpcc_controller"),
                  "Brake value invalid(larger than 1 or smaller than 0). Check "
                  "min/max brake pressure parameter.");
    }

    if (brake_value_ > 0)
    {
      x_.D = -brake_value_;
    }
    else if (accel_value_ == 0)
    {
      x_.D = 0; // coasting
    }
  }

  void planningPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    std::cout << "path global callback" << std::endl;
    planning_result_ = *msg;
    if (msg->poses.size() != 0)
    {
      planning_sub_flg_ = true;
      mpc->setTrack(*msg);
      runControlLoop(x_);
      publishControl(u_);
      publishTrack();
    }
    else
    {
      planning_sub_flg_ = false;
    }
  }

  void setThrottleClipping(const State &x_ego, double &throttle_command,
                           bool is_overtake_mode)
  {
    if (is_overtake_mode)
    {
      if (throttle_command >= 0.85 || x_ego.vx < 130 / 3.6)
      {
        throttle_command = 1.0;
      }
    }
    if (x_ego.vx < 120 / 3.6 && abs(x_ego.delta) < 0.5)
    {
      throttle_command = 1.0;
    }
  }

  void runControlLoop(State &x)
  {

    MPCReturn mpc_sol = mpc->runMPC(x);
    u_ = mpc_sol.u0;
    mpc_horizon_ = mpc_sol.mpc_horizon;
  }

  std::tuple<ArcLengthSpline, Eigen::VectorXd>
  runOppoControlLoop(State &x_oppo,
                     std::vector<ArcLengthSpline> &considering_paths,
                     std::vector<Eigen::VectorXd> &considering_paths_phi)
  {

    oppo_mpc->setMultiplePredictions(considering_paths, considering_paths_phi);

    // Change raceline for initial starting
    // oppo_mpc->changeTrack(initial_raceline_);
    oppo_mpc->track_ = mpc->track_;

    MPCReturn mpc_sol = oppo_mpc->runMPC(x_oppo);

    mpc_horizon_ = mpc_sol.mpc_horizon;
    // log.push_back(mpc_sol);
    Eigen::VectorXd X(mpc_horizon_.size());
    Eigen::VectorXd Y(mpc_horizon_.size());
    Eigen::VectorXd PHI(mpc_horizon_.size());
    for (int i = 0; i < mpc_horizon_.size(); i++)
    {
      X(i) = mpc_horizon_.at(i).xk.X;
      Y(i) = mpc_horizon_.at(i).xk.Y;
      PHI(i) = mpc_horizon_.at(i).xk.phi;
    }
    ArcLengthSpline oppo_solution;
    oppo_solution.path_data_.X = X;
    oppo_solution.path_data_.Y = Y;

    return std::make_tuple(oppo_solution, PHI);
  }

  void publishControl(Input u)
  {
    u_sig_.dD += u.dD;
    u_sig_.dDelta += u.dDelta;
    u_sig_.dVs += u.dVs;

    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.drive.steering_angle =
        (x_.delta + u.dDelta * Ts_ * 1.5); // Front wheel angle (deg)
    msg.drive.steering_angle_velocity = u.dDelta;
    msg.drive.speed = x_.vs + u.dVs * Ts_;
    msg.drive.acceleration = u.dVs; // m/s^2
    control_pub_->publish(msg);
  }

  void runTestSim()
  {
    Integrator integrator = Integrator(jsonConfig["Ts"], json_paths_);
    for (int i = 0; i < jsonConfig["n_sim"]; i++)
    {
      if (rclcpp::ok())
      {
        MPCReturn mpc_sol = mpc->runMPC(x0);
        u_ = mpc_sol.u0;
        mpc_horizon_ = mpc_sol.mpc_horizon;
        log.push_back(mpc_sol);
        x0 = integrator.simTimeStep(x0, mpc_sol.u0, jsonConfig["Ts"]);
        x_ = x0;

        nav_msgs::msg::Odometry msg;
        msg.header.frame_id = map_frame_id_.c_str();
        msg.child_frame_id = ego_frame_id_.c_str();
        msg.pose.pose.position.x = x0.X;
        msg.pose.pose.position.y = x0.Y;

        geometry_msgs::msg::Quaternion q;
        q = ToQuaternion(x0.phi, 0., 0.);
        msg.pose.pose.orientation.x = q.x;
        msg.pose.pose.orientation.y = q.y;
        msg.pose.pose.orientation.z = q.z;
        msg.pose.pose.orientation.w = q.w;

        pose_pub_->publish(msg);

        publishControl(u_);
        publishTrack();
      }
    }
  }

  void publishTrack()
  {
    path_pub_->publish(center_path_);
    bound_in_pub_->publish(bound_in_);
    bound_out_pub_->publish(bound_out_);

    sol_trajectory_.poses.clear();
    // track_boundary_in_.markers.clear();
    // track_boundary_out_.markers.clear();
    track_boundary_in_.poses.clear();
    track_boundary_out_.poses.clear();

    sol_trajectory_.header.stamp = rclcpp::Node::now();

    // std::cout << "MPC horizon size: " << mpc_horizon_.size() << std::endl;
    for (int i = 0; i < N; i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = mpc_horizon_.at(i).xk.X;
      pose.pose.position.y = mpc_horizon_.at(i).xk.Y;
      pose.pose.orientation = ToQuaternion(mpc_horizon_.at(i).xk.phi, 0., 0.);
      sol_trajectory_.poses.push_back(pose);

      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.pose.position.x = mpc->pos_inner_[i][0];
      pose_in.pose.position.y = mpc->pos_inner_[i][1];
      track_boundary_in_.poses.push_back(pose_in);

      geometry_msgs::msg::PoseStamped pose_out;
      pose_out.pose.position.x = mpc->pos_outer_[i][0];
      pose_out.pose.position.y = mpc->pos_outer_[i][1];
      track_boundary_out_.poses.push_back(pose_out);
    }
    sol_trajectory_pub_->publish(sol_trajectory_);
    track_boundary_in_pub_->publish(track_boundary_in_);
    track_boundary_out_pub_->publish(track_boundary_out_);
  }

  void updatePathVisualization()
  {
    center_path_.poses.clear();
    for (int i = 0; i < mpc->track_.path_data_.X.size(); i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = mpc->track_.path_data_.X(i);
      pose.pose.position.y = mpc->track_.path_data_.Y(i);

      center_path_.poses.push_back(pose);
    }
  }

  void publishPredictions(const std::vector<ArcLengthSpline> tracks)
  {
    sol_trajectory_.poses.clear();

    sol_trajectory_.header.stamp = rclcpp::Node::now();

    for (int i = 0; i < N; i++)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = mpc_horizon_.at(i).xk.X;
      pose.pose.position.y = mpc_horizon_.at(i).xk.Y;
      pose.pose.orientation = ToQuaternion(mpc_horizon_.at(i).xk.phi, 0., 0.);
      sol_trajectory_.poses.push_back(pose);
    }
    sol_trajectory_pub_->publish(sol_trajectory_);
  }

  void getSteerToSWARatio(double current_SWA_i, double &calculated_sswa_ratio)
  {
    double current_SWA = abs(current_SWA_i);
    if (current_SWA < 140. * 3.14 / 180.)
    {
      calculated_sswa_ratio = 10.;
    }
    else
    {
      if (current_SWA < 160. * 3.14 / 180.)
      {
        // double ya = (14.0 + 14.43)/2. * 3.14 / 180.;
        // double yb = (20.0 + 23.64)/2. * 3.14 / 180.;
        double ya = (14.0) / 2. * 3.14 / 180.;
        double yb = (20.0) / 2. * 3.14 / 180.;
        double xa = 140.0 * 3.14 / 180.;
        double xb = 160.0 * 3.14 / 180.;
        double interpolate_ratio = (current_SWA - xa) / (xb - xa);
        double interpolated_steer = ya + interpolate_ratio * (yb - ya);
        calculated_sswa_ratio = current_SWA / interpolated_steer;
      }
      else
      {
        // double ya = (20.0 + 23.64)/2. * 3.14 / 180.;
        // double yb = (23.64 + 25.0)/2. * 3.14 / 180.;
        double ya = (20.0) / 2. * 3.14 / 180.;
        double yb = (23.64) / 2. * 3.14 / 180.;
        double xa = 160.0 * 3.14 / 180.;
        double xb = 200.0 * 3.14 / 180.;
        double interpolate_ratio = (current_SWA - xa) / (xb - xa);
        double interpolated_steer = ya + interpolate_ratio * (yb - ya);
        calculated_sswa_ratio = current_SWA / interpolated_steer;
      }
    }
  }

  geometry_msgs::msg::Quaternion
  ToQuaternion(double yaw, double pitch,
               double roll) // yaw (Z), pitch (Y), roll (X)
  {
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
  }

  void localToGlobal(const State &x_ego, State &x_oppo)
  {
    // local to global: [Path_global, 1]^T = [Rot, Trans; 0(dim), 1] *
    // [Path_local, 1]^T global to local: [Path_local, 1]^T = [Rot^T,
    // -Rot^T*Trans; 0(dim), 1] * [Path_global, 1]^T
    double distance_to_collision =
        sqrt(x_oppo.X * x_oppo.X + x_oppo.Y * x_oppo.Y);
    double x_oppo_X_tmp, x_oppo_Y_tmp;
    x_oppo_X_tmp = x_ego.X + distance_to_collision *
                                 cos(x_ego.phi + atan2(x_oppo.Y, x_oppo.X));
    x_oppo_Y_tmp = x_ego.Y + distance_to_collision *
                                 sin(x_ego.phi + atan2(x_oppo.Y, x_oppo.X));
    x_oppo.X = x_oppo_X_tmp;
    x_oppo.Y = x_oppo_Y_tmp;
    double x_oppo_vx, x_oppo_vy;
    x_oppo_vx = x_oppo.vx * cos(x_oppo.phi) + x_oppo.vy * sin(x_oppo.phi);
    x_oppo_vy = -x_oppo.vx * sin(x_oppo.phi) + x_oppo.vy * cos(x_oppo.phi);
    x_oppo.vx = x_oppo_vx;
    x_oppo.vy = x_oppo_vy;
    x_oppo.phi = x_ego.phi + x_oppo.phi;
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr
      steering_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::AcceleratorPedalReport>::SharedPtr
      accel_pedal_report_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::Brake2Report>::SharedPtr
      brake_2_report_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
      planning_output_sub_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
      control_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bound_in_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr bound_out_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sol_trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_boundary_in_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr track_boundary_out_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_publisher;

  std::shared_ptr<MPC> mpc;
  std::shared_ptr<MPC> oppo_mpc;
  double phi_0;
  State x0;
  State oppo_x0;
  State x_;
  Input u_;
  Input u_sig_; // sum of u_
  std::list<MPCReturn> log;
  std::array<OptVariables, N + 1> mpc_horizon_;

  nav_msgs::msg::Path center_path_;
  nav_msgs::msg::Path bound_in_;
  nav_msgs::msg::Path bound_out_;
  nav_msgs::msg::Path sol_trajectory_;
  nav_msgs::msg::Path track_boundary_in_;
  nav_msgs::msg::Path track_boundary_out_;

  geometry_msgs::msg::Point last_position_;

  json jsonConfig;
  json jsonConfig_overtake;
  json jsonConfig_follow;
  json jsonConfig_initial_left;
  json jsonConfig_initial_right;
  json jsonConfig_m25;
  json jsonConfig_m40;
  json oppo_jsonConfig;
  PathToJson json_paths_;

  bool use_mpc_planner_;
  bool use_game_;
  bool use_test_sim_;
  std::string map_frame_id_;
  std::string ego_frame_id_;

  std::string mpcc_dir;
  std::string mpc_config_file_;

  // c_terrain_manager *obj;
  int initial_raceline_;
  TrackPos track_xy_;
  TrackPos track_xy_inital_left_;
  TrackPos track_xy_inital_right_;

  double Ts_;
  double oppo_Ts_;
  double sim_Ts_;
  double oppo_sim_Ts_;
  double steer_to_SWA_;
  double accel_value_;
  double brake_value_;
  double max_brake_pressure_;
  double min_brake_pressure_;

  // planning output
  nav_msgs::msg::Path planning_result_;
  bool planning_sub_flg_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node_mpcc = std::make_shared<MpccRos>();
  auto period = std::chrono::milliseconds(100);
  rclcpp::Rate r(period);
  while (true)
  {
    rclcpp::spin(node_mpcc);
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
