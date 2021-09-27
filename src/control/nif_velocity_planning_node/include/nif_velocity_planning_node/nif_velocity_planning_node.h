/**
 * @file   velocity_planner_node.cpp
 * @author Seungwook Lee, Hyunki Seong
 * @date   2020-10-16, (revised) 2020-12
 * @brief  class implementation of waypoint curvature + vehicle dynamics based
 * velocity planning
 * @arg    input      : Path, Odometry(ego state)(optional)
 *         output     : speed command, planned speed array(optional)
 *         parameters : acceleration limits, longitudinal time lag, max speed.
 * @param  steering_ratio double
 * @param  speed_max double
 * @param  lat_accel_max double
 * @param  lon_accel_max double
 * @param  lon_accel_min double
 *
 * @note   ***IMPORTANT ASSUMPTION***
 *         Path should start from behind of or right next to ego vehicle.
 *
 */

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nif_common/constants.h"
#include "nif_common/types.h"
#include "nif_common/vehicle_model.h"
#include "nif_common_nodes/i_base_synchronized_node.h"
#include "nif_vehicle_dynamics_manager/kalman.h"
#include "nif_vehicle_dynamics_manager/tire_manager.hpp"
#include "nif_velocity_planning_node/low_pass_filter.h"
#include "raptor_dbw_msgs/msg/steering_report.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using nif_msgs::msg::MissionStatus;

namespace nif {
namespace control {
//! Update Rate (in Hz)
const double update_rate_hz = 100.;

class VelocityPlannerNode : public nif::common::IBaseSynchronizedNode {
public:
  VelocityPlannerNode(
      const std::string &node_name,
      const std::chrono::microseconds &timer_period,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : IBaseSynchronizedNode(node_name, common::NodeType::CONTROL,
                              timer_period, options) {

    // Publishers
    des_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "out_desired_velocity", nif::common::constants::QOS_CONTROL_CMD);
    diag_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/velocity_planner/diagnostic", 1);

    // Subscribers
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "in_reference_path", nif::common::constants::QOS_PLANNING,
        std::bind(&VelocityPlannerNode::pathCallback, this,
                  std::placeholders::_1));
    velocity_sub_ =
        this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
            "in_wheel_speed_report", nif::common::constants::QOS_SENSOR_DATA,
            std::bind(&VelocityPlannerNode::velocityCallback, this,
                      std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "in_imu_data", nif::common::constants::QOS_SENSOR_DATA,
        std::bind(&VelocityPlannerNode::imuCallback, this,
                  std::placeholders::_1));
    steering_sub_ =
        this->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
            "in_steering_report", nif::common::constants::QOS_SENSOR_DATA,
            std::bind(&VelocityPlannerNode::steerCallback, this,
                      std::placeholders::_1));
    error_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "in_control_error", nif::common::constants::QOS_DEFAULT,
        std::bind(&VelocityPlannerNode::errorCallback, this,
                  std::placeholders::_1));

    // ROS Params
    this->declare_parameter("vel_plan_enabled", true); // for turn on/off
    this->declare_parameter("debug_mode", false);
    // Curv-based planner
    this->declare_parameter(
        "des_vel_mps",
        27.0); // [m/s]. 11.11m/s (40km/h), 13.89m/s (50km/h), 16.67 (60km/h)
    this->declare_parameter("lpf_curve_f", 8.0); // lower --> smoother & delayed
    this->declare_parameter("lpf_curve_dt", 0.01); // for Low pass filter
    this->declare_parameter("lpf_curve_x0", 0.0);  // for Low pass filter
    // Safety timeouts for odometry and the path (set negative to ignore)
    this->declare_parameter("odometry_timeout_sec", 0.1);
    this->declare_parameter("path_timeout_sec", 0.5);
    // Safe path distance
    // - min safe distance. safe_dist_gain:=0.
    this->declare_parameter("path_dist_min", 8.0);
    // - max safe distance. safe_dist_gain:=1.
    this->declare_parameter("path_dist_max", 30.0);
    // - time to collision(arrive) until safe path dist
    this->declare_parameter("ttc_thres", 2.0);

    // Read in misc. parameters
    m_vel_plan_enabled = this->get_parameter("vel_plan_enabled").as_bool();
    m_debug_mode = this->get_parameter("debug_mode").as_bool();
    m_max_vel_mps = this->get_parameter("des_vel_mps").as_double();
    m_lpf_curve_f = this->get_parameter("lpf_curve_f").as_double();
    m_lpf_curve_dt = this->get_parameter("lpf_curve_dt").as_double();
    m_lpf_curve_x0 = this->get_parameter("lpf_curve_x0").as_double();
    m_odometry_timeout_sec =
        this->get_parameter("odometry_timeout_sec").as_double();
    m_path_timeout_sec = this->get_parameter("path_timeout_sec").as_double();
    m_path_dist_min = this->get_parameter("path_dist_min").as_double();
    m_path_dist_max = this->get_parameter("path_dist_max").as_double();
    m_ttc_thres = this->get_parameter("ttc_thres").as_double();

    if (m_odometry_timeout_sec <= 0. || m_path_timeout_sec <= 0.) {
      RCLCPP_ERROR(this->get_logger(),
                   "path and ego_odometry timeouts must be greater than zero. "
                   "Got m_odometry_timeout_sec: %f; m_path_timeout_sec: %f",
                   m_odometry_timeout_sec, m_path_timeout_sec);
      throw std::range_error("Parameter out of range.");
    }

    m_odometry_timeout =
        rclcpp::Duration(0, m_odometry_timeout_sec * 1000000000);
    m_path_timeout = rclcpp::Duration(0, m_path_timeout_sec * 1000000000);

    // Parameter check
    parameters_callback_handle = this->add_on_set_parameters_callback(std::bind(
        &VelocityPlannerNode::parametersCallback, this, std::placeholders::_1));
  };

private:
  /**
   * Main control loop. the timer is declared in IBaseSynchronizedNode
   */
  void run() override {
    nif::common::NodeStatusCode node_status = common::NODE_ERROR;
    auto now = this->now();
    bool valid_path =
        has_path &&
        !m_current_path.poses
             .empty() && // TODO too loose, require a minimum length.
        now - m_path_update_time < m_path_timeout;

    bool valid_odom =
        this->hasEgoOdometry() &&
        now - this->getEgoOdometryUpdateTime() < m_odometry_timeout;
    bool valid_tracking_result = false;

    // Variables
    double desired_velocity_mps = 0.0;                 // [m/s]
    double current_velocity_mps = m_current_speed_mps; // [m/s]
    double current_steer_rad = m_current_steer_rad;    // [rad]

    // Perform Velocity Planning if path is good
    if (m_vel_plan_enabled) {
      // Publish planned desired velocity
      if (valid_path && valid_odom) {
        valid_tracking_result = true;
        // Get current accelerations
        // double a_lon = m_current_imu.linear_acceleration.x;
        // double a_lat = m_current_imu.linear_acceleration.y;
        // double yaw_rate = m_current_imu.angular_velocity.z;

        // Get curvature & cumul. distance
        // - get curvature array (+curv: CCW rotation / -curv: CW rotation)
        int path_len = m_current_path.poses.size();
        std::vector<double> curv_array(path_len, 0.0); // for curvature
        std::vector<double> dist_array(path_len, 0.0); // for cumul. distance
        std::vector<double> lpf_curv_array(path_len, 0.0);
        // - get curv. & dist. array (+curv: CCW rotation / -curv: CW rotation)
        getCurvatureDistArray(curv_array, dist_array);
        // // - get curvature array (+curv: CCW rotation / -curv: CW rotation)
        // getCurvatureArray(curv_array);

        // - apply low pass filter
        lpf_curve.getFilteredArray(curv_array, lpf_curv_array);
        // - get instantaneous curvature
        double kappa = lpf_curv_array[0];
        // - get path distance
        double path_dist = dist_array[path_len - 1];

        // Get Lateral Acceleration Limit using Vehicle dynamics manager
        double a_lat_max = m_tire_manager.ComputeLateralAccelLimit(
            m_a_x_kf, m_a_y_kf, m_yaw_rate_kf, current_steer_rad,
            current_velocity_mps);
        // double a_lat_max = m_tire_manager.ComputeLateralAccelLimit(
        //     a_lon, a_lat, yaw_rate, current_steer_rad, current_velocity_mps);

        // Compute maximum velocity
        if (abs(kappa) <= m_CURVATURE_MINIMUM) {
          // Handling too small curvature
          desired_velocity_mps = m_max_vel_mps;
        } else {
          // Compute velocity using centripetal acceleration equation
          // F = m*v^2 / R; v^2 = a * R; v = sqrt(a * R) = sqrt(a / kappa)
          desired_velocity_mps = sqrt(abs(a_lat_max) / abs(kappa));
          desired_velocity_mps = std::min(desired_velocity_mps, m_max_vel_mps);
        }
        // Decrease desired velocity w.r.t. lateral error
        double error_ratio = std::min(abs(m_error_y_lpf), m_ERROR_Y_MAX) /
                             m_ERROR_Y_MAX;          // [0.0~1.0] ratio
        double error_gain = 1.0 - 0.5 * error_ratio; // [1.0~0.5] gain

        // Decrease desired velocity w.r.t. Safe path distance
        double path_dist_safe =
            m_path_dist_min + m_current_speed_mps * m_ttc_thres;
        path_dist_safe = std::max(m_path_dist_min + 0.001,
                                  std::min(path_dist_safe, m_path_dist_max));

        double safe_dist_gain =
            (path_dist - m_path_dist_min) / (path_dist_safe - m_path_dist_min);
        safe_dist_gain = std::max(0.0, std::min(safe_dist_gain, 1.0));

        // Safe path gain + Error gain
        desired_velocity_mps =
            error_gain * safe_dist_gain * desired_velocity_mps;
        // desired_velocity_mps = error_gain * desired_velocity_mps;

        // Publish diagnose message
        publishDiagnostic(kappa, abs(a_lat_max), error_gain, safe_dist_gain,
                          path_dist_safe, m_a_x_kf, m_a_y_kf, m_yaw_rate_kf);

      } else {
        //! Publish zero desired velocity
        //! TODO: safety stop should be triggered
        //! TODO: should be improved.
        desired_velocity_mps = 0.0;
      }
    } else {
      // manual control case
      desired_velocity_mps = m_max_vel_mps;
    }

    if (!this->hasSystemStatus() ||
        (this->getSystemStatus().autonomy_status.lateral_autonomy_enabled ||
         this->getSystemStatus()
             .autonomy_status.longitudinal_autonomy_enabled) &&
            !(valid_path && valid_odom)) {

      node_status = common::NODE_ERROR;
    } else {
      node_status = common::NODE_OK;
    }
    // Maximum velocity limit
    desired_velocity_mps = std::min(desired_velocity_mps, m_max_vel_mps);

    // Publish planned desired velocity
    publishPlannedVelocity(desired_velocity_mps);
    this->setNodeStatus(node_status);
  }

  void afterSystemStatusCallback() override {
    // CHECK MISSION CONDITIONS
    if (this->hasSystemStatus()) {
      this->m_max_vel_mps =
          this->getSystemStatus().mission_status.max_velocity_mps;
    }
  }

  void getCurvatureArray(std::vector<double> &curvature_array) {
    // Iterate computeCurvature to get curvature for each point in path.
    std::vector<double> vec(m_current_path.poses.size(), 0.0);
    for (int i = 1; i < m_current_path.poses.size() - 1; i++) {
      vec[i] =
          computeCurvature(m_current_path.poses[i - 1], m_current_path.poses[i],
                           m_current_path.poses[i + 1]);
    }
    // first & last index
    vec[0] = vec[1];
    vec[m_current_path.poses.size() - 1] = vec[m_current_path.poses.size() - 2];
    curvature_array = vec;
  }

  void getCurvatureDistArray(std::vector<double> &curv_array,
                             std::vector<double> &dist_array) {
    // Iterate computeCurvature to get curvature for each point in path.
    double path_size = m_current_path.poses.size();

    std::vector<double> vec_curv(path_size, 0.0);
    std::vector<double> vec_dist(path_size, 0.0);

    for (int i = 1; i < path_size - 1; i++) {
      // - compute curvature
      vec_curv[i] =
          computeCurvature(m_current_path.poses[i - 1], m_current_path.poses[i],
                           m_current_path.poses[i + 1]);
      // - compute cumulative distance
      vec_dist[i] =
          vec_dist[i - 1] +
          computeDistance(m_current_path.poses[i - 1], m_current_path.poses[i]);
    }
    // first & last index
    // - curvature
    vec_curv[0] = vec_curv[1];
    vec_curv[path_size - 1] = vec_curv[path_size - 2];
    // - cumulative distance
    vec_dist[path_size - 1] =
        vec_dist[path_size - 2] +
        computeDistance(m_current_path.poses[path_size - 2],
                        m_current_path.poses[path_size - 1]);
    curv_array = vec_curv;
    dist_array = vec_dist;
  }

  double computeCurvature(geometry_msgs::msg::PoseStamped &pose_i_prev,
                          geometry_msgs::msg::PoseStamped &pose_i,
                          geometry_msgs::msg::PoseStamped &pose_i_next) {

    // Calcalate curvature(+/) of a line with discrete points(3)
    // References:
    // https://ed-matrix.com/mod/page/view.php?id=2771
    // https://www.skedsoft.com/books/maths-for-engineers-1/radius-of-curvature-in-parametric-form
    double x_1 = pose_i_prev.pose.position.x;
    double x_2 = pose_i.pose.position.x;
    double x_3 = pose_i_next.pose.position.x;

    double y_1 = pose_i_prev.pose.position.y;
    double y_2 = pose_i.pose.position.y;
    double y_3 = pose_i_next.pose.position.y;

    double s_12 = sqrt((x_1 - x_2) * (x_1 - x_2) + (y_1 - y_2) * (y_1 - y_2));
    double s_23 = sqrt((x_2 - x_3) * (x_2 - x_3) + (y_2 - y_3) * (y_2 - y_3));
    double s_13 = sqrt((x_1 - x_3) * (x_1 - x_3) + (y_1 - y_3) * (y_1 - y_3));

    double curvature;

    if (s_12 * s_23 * s_13 == 0) {
      curvature = m_CURVATURE_MINIMUM;
    } else {
      double x_d = (x_3 - x_1) / s_13;
      double x_d12 = (x_2 - x_1) / s_12;
      double x_d23 = (x_3 - x_2) / s_23;
      double x_dd = (x_d23 - x_d12) / s_13;

      double y_d = (y_3 - y_1) / s_13;
      double y_d12 = (y_2 - y_1) / s_12;
      double y_d23 = (y_3 - y_2) / s_23;
      double y_dd = (y_d23 - y_d12) / s_13;

      if (x_d * x_d + y_d * y_d == 0) {
        curvature = m_CURVATURE_MINIMUM;
      } else {
        // curvature = abs(x_d * y_dd - y_d * x_dd) / pow(sqrt(x_d * x_d + y_d *
        // y_d), 3.0);
        curvature =
            (x_d * y_dd - y_d * x_dd) / pow(sqrt(x_d * x_d + y_d * y_d), 3.0);
      }
    }

    return curvature;
  }

  double computeDistance(geometry_msgs::msg::PoseStamped pose_i_prev,
                         geometry_msgs::msg::PoseStamped pose_i) {
    double x_1 = pose_i_prev.pose.position.x;
    double x_2 = pose_i.pose.position.x;
    double y_1 = pose_i_prev.pose.position.y;
    double y_2 = pose_i.pose.position.y;

    return sqrt(pow(x_1 - x_2, 2.0) + pow(y_1 - y_2, 2.0));
  }

  /** ROS Publishing Interface **/
  void publishPlannedVelocity(double desired_velocity_mps) {
    std_msgs::msg::Float32 des_vel;
    des_vel.data = desired_velocity_mps;
    des_vel_pub_->publish(des_vel);
  }

  void publishDiagnostic(double curvature, double a_lat_max, double error_gain,
                         double safe_dist_gain, double safe_dist, double a_x_kf,
                         double a_y_kf, double yaw_rate_kf) {
    // Diagnostic: {curvature, a_lat_max}
    std_msgs::msg::Float32MultiArray diagnostic;
    diagnostic.data.push_back(curvature);
    diagnostic.data.push_back(a_lat_max);
    diagnostic.data.push_back(error_gain);
    diagnostic.data.push_back(safe_dist_gain);
    diagnostic.data.push_back(safe_dist);

    diagnostic.data.push_back(a_x_kf);
    diagnostic.data.push_back(a_y_kf);
    diagnostic.data.push_back(yaw_rate_kf);

    diag_pub_->publish(diagnostic);
  }

  /** ROS Callbacks / Subscription Interface **/
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    has_path = true;
    m_path_update_time = this->now();
    // lqr_tracking_idx_ = 0; // Reset Tracking
    m_current_path = *msg;
  }

  void velocityCallback(
      const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
    m_current_speed_mps = (msg->rear_left + msg->rear_right) * 0.5 *
                          nif::common::constants::KPH2MS;
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    m_current_imu = *msg;
    // kalman filter initialization
    if (!kalman_init) {
      kf_a_lat.init(2);
      kf_a_lat.setProcessNoise(0.1, 0.01);
      kf_a_lat.setMeasurementNoise(0.4);

      kf_a_lon.init(2);
      kf_a_lon.setProcessNoise(0.1, 0.01);
      kf_a_lon.setMeasurementNoise(0.4);

      kf_yaw_rate.init(2);
      kf_yaw_rate.setProcessNoise(0.05, 0.0025);
      kf_yaw_rate.setMeasurementNoise(0.1);

      kalman_init = true;
    }
    // kalman filtering
    auto now = rclcpp::Clock().now();
    float dt = secs(now - m_imu_update_time);
    // - prediction
    kf_a_lat.predict(dt);
    kf_a_lon.predict(dt);
    kf_yaw_rate.predict(dt);
    // - get filtered values
    m_a_x_kf = kf_a_lon.get();
    m_a_y_kf = kf_a_lat.get();
    m_yaw_rate_kf = kf_yaw_rate.get();
    // - correction
    kf_a_lon.correct((float)(msg->linear_acceleration.x));
    kf_a_lat.correct((float)(msg->linear_acceleration.y));
    kf_yaw_rate.correct((float)(msg->angular_velocity.z));

    m_imu_update_time = now;
  }

  void
  steerCallback(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg) {
    m_current_steer_rad = msg->steering_wheel_angle *
                          nif::common::constants::RAD2DEG /
                          m_steer_ratio; // tire angle [deg --> rad]
  }

  void errorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // Lateral error (m_error_y) and Filtered lateral error (m_error_y_lpf)
    m_error_y = msg->data[0];
    // Apply low pass filter
    lpf_error.getFilteredValue(m_error_y, m_error_y_lpf);
  }

  rcl_interfaces::msg::SetParametersResult
  parametersCallback(const std::vector<rclcpp::Parameter> &vector) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "";
    for (const auto &param : vector) {
      //! Check desired velocity should not be too large value
      if (param.get_name() == "des_vel_mps") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE &&
            param.as_double() <= 300 / 3.6) {
          this->m_max_vel_mps = param.as_double();
          result.successful = true;
        } else {
          result.successful = false;
          result.reason = "des_vel_mps must be a double below (200 / 3.6).";
        }
      } else if (param.get_name() == "vel_plan_enabled") {
        if (param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL &&
            this->m_current_speed_mps <= 0.1) {
          this->m_vel_plan_enabled = param.as_bool();
          result.successful = true;
        } else {
          result.successful = false;
          result.reason = "vel_plan_enabled must be a boolean, and the current "
                          "speed must be lower than 0.1 m/s .";
        }
      }
    }
    return result;
  }

  // Kalman filters
  KalmanFilter kf_a_lat;
  KalmanFilter kf_a_lon;
  KalmanFilter kf_yaw_rate;

  //! Input Data
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr
      velocity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr
      steering_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr error_sub_;

  //! Publisher
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr des_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr diag_pub_;

  //! Update Timers
  rclcpp::TimerBase::SharedPtr control_timer_;

  //! for rosparam checking
  OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle;

  //! Track when certain variables have been updated
  rclcpp::Time m_path_update_time;
  rclcpp::Time m_imu_update_time = rclcpp::Clock().now();
  bool has_path = false;

  //! Load vehicle dynamics manager
  TireManager m_tire_manager;
  //! Low pass filter
  low_pass_filter lpf_curve =
      low_pass_filter(m_lpf_curve_dt, m_lpf_curve_f, m_lpf_curve_x0);
  low_pass_filter lpf_error = low_pass_filter(0.04, 2.0, 0.0);

  //! Vehicle Params
  double m_L = nif::common::vehicle_param::VEH_WHEEL_BASE;  // wheelbase
  double m_total = nif::common::vehicle_param::VEH_MASS_KG; // total mass [kg]
  double m_steer_ratio =
      nif::common::vehicle_param::STEERING_RATIO; // steer ratio

  //! Current Vehicle State
  nav_msgs::msg::Path m_current_path;
  sensor_msgs::msg::Imu m_current_imu;
  double m_current_steer_rad{};   // [rad]
  double m_current_speed_mps = 0; // [m/s]
  double m_error_y = 0;           // [m]
  double m_error_y_lpf = 0;       // [m]

  bool kalman_init = false;
  double m_a_x_kf = 0.0;      // longitudinal accel from KF
  double m_a_y_kf = 0.0;      // lateral accel from KF
  double m_yaw_rate_kf = 0.0; // yaw rate from KF

  //! Params for Velocity planning
  std::vector<double> m_velocity_profile;
  std::vector<double> m_SpeedProfile;
  std::vector<double> m_SpeedProfile_raw;
  std::vector<double> m_CurveSpeedProfile;
  std::vector<double> m_CurveSpeedProfile_raw;

  //! Misc. Parameters (see notes in constructor)
  //! bool use_tire_velocity_;
  bool m_vel_plan_enabled;
  bool m_debug_mode;
  double m_max_vel_mps;
  double m_lpf_curve_f;
  double m_lpf_curve_dt;
  double m_lpf_curve_x0;

  double m_odometry_timeout_sec;
  rclcpp::Duration m_odometry_timeout = rclcpp::Duration(1, 0);
  double m_path_timeout_sec;
  rclcpp::Duration m_path_timeout = rclcpp::Duration(1, 0);

  double m_path_dist_min;
  double m_path_dist_max;
  double m_ttc_thres;

  double m_CURVATURE_MINIMUM = 0.000001;
  double m_ERROR_Y_MAX = 4.0; // halving des_vel point. Width of IMS track: 12 m

  double secs(rclcpp::Time t) {
    return static_cast<double>(t.seconds()) +
           static_cast<double>(t.nanoseconds()) * 1e-9;
  }
  double secs(rclcpp::Duration t) {
    return static_cast<double>(t.seconds()) +
           static_cast<double>(t.nanoseconds()) * 1e-9;
  }

}; /* class VelocityPlannerNode */
} // namespace control
} // namespace nif