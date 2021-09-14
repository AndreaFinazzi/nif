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
#include "nif_vehicle_dynamics_manager/tire_manager.hpp"
#include "nif_velocity_planning_node/low_pass_filter.h"
#include "raptor_dbw_msgs/msg/steering_report.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nif_common/vehicle_model.h"

namespace nif{
    namespace control{
        //! Update Rate (in Hz)
        const double update_rate_hz = 100.;

        class VelocityPlannerNode : public rclcpp::Node {
        public:
            VelocityPlannerNode(const std::string &node_name,
                                const std::chrono::microseconds &timer_period,
                                const rclcpp::NodeOptions &options = rclcpp::NodeOptions{}) : Node(node_name) {

                // Publishers
                des_vel_pub_ = this->create_publisher<std_msgs::msg::Float32>(
                    "velocity_planner/des_vel", nif::common::constants::QOS_CONTROL_CMD);

                // Subscribers
                path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
                    "bvs_controller/target_path", nif::common::constants::QOS_PLANNING,
                        std::bind(&VelocityPlannerNode::pathCallback, this,
                                  std::placeholders::_1));
                odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                    "bvs_localization/ltp_odom", nif::common::constants::QOS_EGO_ODOMETRY,
                        std::bind(&VelocityPlannerNode::odometryCallback, this,
                                  std::placeholders::_1));
                velocity_sub_ =
                        this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
                            "raptor_dbw_interface/wheel_speed_report", nif::common::constants::QOS_SENSOR_DATA,
                                std::bind(&VelocityPlannerNode::velocityCallback, this,
                                          std::placeholders::_1));
                imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                    "novatel_bottom/imu/data", nif::common::constants::QOS_SENSOR_DATA,
                        std::bind(&VelocityPlannerNode::imuCallback, this,
                                  std::placeholders::_1));
                steering_sub_ =
                        this->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
                            "raptor_dbw_interface/steering_report", nif::common::constants::QOS_SENSOR_DATA,
                                std::bind(&VelocityPlannerNode::steerCallback, this,
                                          std::placeholders::_1));
                error_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                    "bvs_controller/lqr_error", nif::common::constants::QOS_DEFAULT,
                        std::bind(&VelocityPlannerNode::errorCallback, this,
                                  std::placeholders::_1));

                //! Timer to execute control at 25Hz
                control_timer_ = this->create_wall_timer(timer_period,
                        std::bind(&VelocityPlannerNode::velocityPlanning, this));

                // ROS Params
                this->declare_parameter("vel_plan_enabled", true); // for turn on/off
                this->declare_parameter("debug_mode", false);
                // Curv-based planner
                this->declare_parameter("max_vel", 33.33);   // [m/s]
                this->declare_parameter("lpf_curve_f", 8.0); // lower --> smoother & delayed
                this->declare_parameter("lpf_curve_dt", 0.01); // for Low pass filter
                this->declare_parameter("lpf_curve_x0", 0.0);  // for Low pass filter
                // Safety timeouts for odometry and the path (set negative to ignore)
                this->declare_parameter("odometry_timeout_sec", 0.1);
                this->declare_parameter("path_timeout_sec", 0.5);

                // Read in misc. parameters
                m_vel_plan_enabled = this->get_parameter("vel_plan_enabled").as_bool();
                m_debug_mode = this->get_parameter("debug_mode").as_bool();
                m_max_vel = this->get_parameter("max_vel").as_double();
                m_lpf_curve_f = this->get_parameter("lpf_curve_f").as_double();
                m_lpf_curve_dt = this->get_parameter("lpf_curve_dt").as_double();
                m_lpf_curve_x0 = this->get_parameter("lpf_curve_x0").as_double();
                m_odometry_timeout_sec =
                        this->get_parameter("odometry_timeout_sec").as_double();
                m_path_timeout_sec = this->get_parameter("path_timeout_sec").as_double();
            };

            void velocityPlanning() {
                auto now = rclcpp::Clock().now();
                bool valid_path = !m_current_path.poses.empty() &&
                                  (secs(now - m_path_update_time) < m_path_timeout_sec ||
                                   m_path_timeout_sec < 0.0);
                bool valid_odom = secs(now - m_odom_update_time) < m_odometry_timeout_sec ||
                                  m_odometry_timeout_sec < 0.0;
                bool valid_tracking_result = false;

                // Variables
                double desired_velocity = 0.0;                // [m/s]
                double current_velocity = m_current_speed_ms; // [m/s]
                double current_steer = m_current_steer;       // [rad]

                // Perform Velocity Planning if path is good
                if (m_vel_plan_enabled) {
                    // Publish planned desired velocity
                    if (valid_path && valid_odom) {
                        valid_tracking_result = true;
                        nav_msgs::msg::Path current_path = m_current_path;

                        // Get current accelerations
                        double a_lon = m_current_imu.linear_acceleration.x;
                        double a_lat = m_current_imu.linear_acceleration.y;
                        double yaw_rate = m_current_imu.angular_velocity.z;

                        // Get curvature
                        // - get curvature array (+curv: CCW rotation / -curv: CW rotation)
                        int path_len = current_path.poses.size();
                        std::vector<double> curv_array(path_len, 0.0);
                        std::vector<double> lpf_curv_array(path_len, 0.0);
                        getCurvatureArray(curv_array);
                        // - apply low pass filter
                        lpf_curve.getFilteredArray(curv_array, lpf_curv_array);
                        // - get instantaneous curvature
                        double kappa = lpf_curv_array[0];

                        // Get Lateral Acceleration Limit using Vehicle dynamics manager
                        double a_lat_max = m_tire_manager.ComputeLateralAccelLimit(
                                a_lon, a_lat, yaw_rate, current_steer, current_velocity);

                        // Compute maximum velocity
                        if (abs(kappa) <= m_CURVATURE_MINIMUM) {
                            // Handling too small curvature
                            desired_velocity = m_max_vel;
                        } else {
                            // Compute velocity using centripetal acceleration equation
                            // F = m*v^2 / R; v^2 = a * R; v = sqrt(a * R) = sqrt(a / kappa)
                            desired_velocity = sqrt(abs(a_lat_max) / abs(kappa));
                            desired_velocity = std::min(desired_velocity, m_max_vel);
                        }
                        // Decrease desired velocity w.r.t. lateral error
                        // double error_ratio = std::min(abs(m_error_y), m_ERROR_Y_MAX) /
                        //                      m_ERROR_Y_MAX;          // [0.0~1.0] ratio
                        double error_ratio = std::min(abs(m_error_y_lpf), m_ERROR_Y_MAX) /
                                             m_ERROR_Y_MAX;          // [0.0~1.0] ratio
                        double error_gain = 1.0 - 0.5 * error_ratio; // [1.0~0.5] gain
                        desired_velocity = error_gain * desired_velocity;

                    } else {
                        //! Publish zero desired velocity
                        //! TODO: safety stop should be triggered
                        //! TODO: should be improved.
                        desired_velocity = 0.0;
                    }
                } else {
                    // manual controll case
                    desired_velocity = m_max_vel;
                }
                // Maximum velocity limit
                desired_velocity = std::min(desired_velocity, m_max_vel);

                // Publish planned desired velocity
                publishPlannedVelocity(desired_velocity);
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
                        // curvature = abs(x_d * y_dd - y_d * x_dd) / pow(sqrt(x_d * x_d + y_d * y_d), 3.0);
                        curvature =
                                (x_d * y_dd - y_d * x_dd) / pow(sqrt(x_d * x_d + y_d * y_d), 3.0);
                    }
                }

                return curvature;
            }

            /** ROS Publishing Interface **/
            void publishPlannedVelocity(double desired_velocity_mps) {
                std_msgs::msg::Float32 des_vel;
                des_vel.data = desired_velocity_mps;
                des_vel_pub_->publish(des_vel);
            }

            /** ROS Callbacks / Subscription Interface **/
            void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
                m_path_update_time = rclcpp::Clock().now();
                // lqr_tracking_idx_ = 0; // Reset Tracking
                m_current_path = *msg;
            }

            void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
                m_odom_update_time = rclcpp::Clock().now();
                m_current_odometry = *msg;
            }

            void velocityCallback(
                    const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
                m_current_speed_ms = (msg->rear_left + msg->rear_right) * 0.5 *
                                     nif::common::constants::KPH2MS;
            }

            void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
                m_current_imu = *msg;
            }

            void
            steerCallback(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg) {
                m_current_steer = msg->steering_wheel_angle *
                                  nif::common::constants::RAD2DEG /
                                  m_steer_ratio; // tire angle [deg --> rad]
            }

            void errorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
                // Lateral error (m_error_y) and Filtered lateral error (m_error_y_lpf)
                m_error_y = msg->data[0];
                // Apply low pass filter
                lpf_error.getFilteredValue(m_error_y, m_error_y_lpf);
            }

        private:
            //! Input Data
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
            rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr
                    velocity_sub_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
            rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr
                    steering_sub_;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr steering_cmd_sub_;
            rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr error_sub_;

            //! Publisher
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr des_vel_pub_;

            //! Update Timers
            rclcpp::TimerBase::SharedPtr control_timer_;

            //! Track when certain variables have been updated
            rclcpp::Time m_path_update_time;
            rclcpp::Time m_odom_update_time;

            //! Load vehicle dynamics manager
            TireManager m_tire_manager;
            //! Low pass filter
            low_pass_filter lpf_curve =
                    low_pass_filter(m_lpf_curve_dt, m_lpf_curve_f, m_lpf_curve_x0);
            low_pass_filter lpf_error = low_pass_filter(0.04, 2.0, 0.0);

            //! Vehicle Params
            double m_L = nif::common::vehicle_param::VEH_WHEEL_BASE;                     // wheelbase
            double m_total = nif::common::vehicle_param::VEH_MASS_KG;           // total mass [kg]
            double m_steer_ratio = nif::common::vehicle_param::STEERING_RATIO; // steer ratio

            //! Current Vehicle State
            nav_msgs::msg::Path m_current_path;
            nav_msgs::msg::Odometry m_current_odometry;
            sensor_msgs::msg::Imu m_current_imu;
            double m_current_steer{};        // [rad]
            double m_current_speed_ms = 0; // [m/s]
            double m_error_y = 0;          // [m]
            double m_error_y_lpf = 0;      // [m]

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
            double m_max_vel;
            double m_lpf_curve_f;
            double m_lpf_curve_dt;
            double m_lpf_curve_x0;
            double m_odometry_timeout_sec;
            double m_path_timeout_sec;

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
    }
}