/**
 * @brief path following node
 **/

// Application
#include "bvs_control/lqr/lateral_lqr.h"
#include "bvs_control/utils/lateral_lqr_ros.h"
#include "bvs_control/utils/pure_pursuit_tracker.h"

// ROS
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/bool.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"
#include "deep_orange_msgs/msg/joystick_command.hpp"


namespace bvs_control {

//! Conversion factor from radians to degrees
const double rad2deg = 57.2957795131;
//! Converstion factor from kph to m/s
const double kph2ms = 1.0/3.6;
//! Update Rate (in Hz)
const double update_rate_hz = 25.;

class PathFollowerNode : public rclcpp::Node {
public:
    PathFollowerNode() : Node("PathFollowerNode") {
        // Debug Publishers
        lqr_command_valid_pub_ = this->create_publisher<std_msgs::msg::Bool>("bvs_controller/tracking_valid", 1);
        lqr_steering_command_pub_ = this->create_publisher<std_msgs::msg::Float32>("bvs_controller/lqr_command", 1);
        track_distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("bvs_controller/track_distance", 1);
        lqr_tracking_point_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("bvs_controller/track_point", 1);
        lqr_error_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("bvs_controller/lqr_error", 1);

        // Command Publishers
        steering_command_pub_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/steering_cmd", 1);
        throttle_command_pub_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/accelerator_cmd", 1);
        brake_command_pub_ = this->create_publisher<std_msgs::msg::Float32>("/joystick/brake_cmd", 1);
        gear_command_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/joystick/gear_cmd", 1);

        // setup QOS to be best effort
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        qos.best_effort();

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("bvs_controller/target_path",
            1, std::bind(&PathFollowerNode::pathCallback, this, std::placeholders::_1));
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("bvs_localization/ltp_odom",
            1, std::bind(&PathFollowerNode::odometryCallback, this, std::placeholders::_1));
        velocity_sub_ = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>("/raptor_dbw_interface/wheel_speed_report",
            1, std::bind(&PathFollowerNode::velocityCallback, this, std::placeholders::_1));
        joystick_sub_ = this->create_subscription<deep_orange_msgs::msg::JoystickCommand>("/joystick/command",
            qos, std::bind(&PathFollowerNode::joystickCallback, this, std::placeholders::_1));

        // Gives memory exceptions with my test rosbag :(
        // pt_report_sub_ = this->create_subscription<deep_orange_msgs::msg::PtReport>("/raptor_dbw_interface/pt_report",
        //     1, std::bind(&PathFollowerNode::ptReportCallback, this, std::placeholders::_1));

        //! Timer to execute control at 25Hz
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000./update_rate_hz)), std::bind(&PathFollowerNode::executeControl, this));

        this->declare_parameter("lqr_config_file", "");
        // Automatically boot with lat_autonomy_enabled
        this->declare_parameter("lat_autonomy_enabled", false);
        // Max Steering Angle in Degrees
        this->declare_parameter("max_steering_angle_deg", 20.0);
        // Degrees at which to automatically revert to the override
        this->declare_parameter("steering_auto_override_deg", 4.0);
        // convert from degress to steering units (should be 1 - 1 ?)
        this->declare_parameter("steering_units_multiplier", 1.0);
        // Minimum pure pursuit tracking distance
        this->declare_parameter("pure_pursuit_min_dist_m", 4.0);
        // Maximimum pure pursuit tracking distance
        this->declare_parameter("pure_pursuit_max_dist_m", 8.);
        // Factor to increase the pure pursuit tracking distance as a function of speed (m/s)
        this->declare_parameter("pure_pursuit_k_vel_m_ms", 0.75);
        // Use tire speed instead of gps velocity estimate
        this->declare_parameter("use_tire_velocity", true);
        // Safety timeouts for odometry and the path (set negative to ignore)
        this->declare_parameter("odometry_timeout_sec", 0.1);
        this->declare_parameter("path_timeout_sec", 0.5);
        // Limit the max change in the steering signal over time
        this->declare_parameter("steering_max_ddeg_dt", 5.0);

        // Create Lateral LQR Controller from yaml file
        std::string lqr_config_file = this->get_parameter("lqr_config_file").as_string();
        RCLCPP_INFO(get_logger(), "Loading control params: %s", lqr_config_file.c_str());
        lateral_lqr_ = lqr::LateralLQR::newPtr(lqr_config_file);

        // Read in misc. parameters
        max_steering_angle_deg_ = this->get_parameter("max_steering_angle_deg").as_double();
        steering_auto_override_deg_ = this->get_parameter("steering_auto_override_deg").as_double();
        steering_units_multiplier_ = this->get_parameter("steering_units_multiplier").as_double();
        pure_pursuit_min_dist_m_ = this->get_parameter("pure_pursuit_min_dist_m").as_double();
        pure_pursuit_max_dist_m_ = this->get_parameter("pure_pursuit_max_dist_m").as_double();
        pure_pursuit_k_vel_m_ms_ = this->get_parameter("pure_pursuit_k_vel_m_ms").as_double();
        use_tire_velocity_ = this->get_parameter("use_tire_velocity").as_bool();
        odometry_timeout_sec_ = this->get_parameter("odometry_timeout_sec").as_double();
        path_timeout_sec_ = this->get_parameter("path_timeout_sec").as_double();
        steering_max_ddeg_dt_ = this->get_parameter("steering_max_ddeg_dt").as_double();
    };

    void executeControl() {
        auto now = rclcpp::Clock().now();
        bool lateral_tracking_enabled = this->get_parameter("lat_autonomy_enabled").as_bool();
        bool valid_path = current_path_.poses.size() > 0 && (secs(now - path_update_time_) < path_timeout_sec_ || path_timeout_sec_ < 0.0);
        bool valid_odom = secs(now - odom_update_time_) < odometry_timeout_sec_ || odometry_timeout_sec_ < 0.0;
        bool valid_tracking_result = false;

        double steering_angle = 0.0;
        // Perform Tracking if path is good
        if (valid_path && valid_odom) {
            valid_tracking_result = true;

            auto state = utils::LQRState(current_odometry_);
            if(use_tire_velocity_)
                state(2,0) = current_speed_ms_;

            // Compute the tracking distance (and ensure it is within a valid range)
            double track_distance = pure_pursuit_min_dist_m_ + pure_pursuit_k_vel_m_ms_ * state(2,0);
            if (track_distance > pure_pursuit_max_dist_m_) track_distance = pure_pursuit_max_dist_m_;
            if (track_distance < pure_pursuit_min_dist_m_) track_distance = pure_pursuit_min_dist_m_;

            // Track on the trajectory
            double target_distance = 0.0;
            bool target_reached_end = false;
            utils::track(current_path_.poses, current_odometry_, track_distance, lqr_tracking_idx_, target_distance, target_reached_end);

            // Run LQR :)
            auto goal = utils::LQRGoal(current_path_.poses[lqr_tracking_idx_]);
            auto error = lateral_lqr_->computeError(state, goal);
            steering_angle = lateral_lqr_->process(state, goal) * rad2deg;

            // Make sure steering angle is within range
            if(steering_angle > max_steering_angle_deg_) steering_angle = max_steering_angle_deg_;
            if(steering_angle < -max_steering_angle_deg_) steering_angle = -max_steering_angle_deg_;

            // Smooth and publish diagnostics
            utils::smoothSignal(steering_angle, last_steering_command_, steering_max_ddeg_dt_, 1./update_rate_hz);
            publishSteeringDiagnostics(true, steering_angle, track_distance, current_path_.poses[lqr_tracking_idx_], error);
        }

        // Check / Process Overrides
        if(std::abs(override_steering_target_) > std::abs(steering_auto_override_deg_) || !lateral_tracking_enabled) {
            steering_angle = override_steering_target_;
        } else if(!valid_tracking_result) {
            steering_angle = 0.;
        }
        last_steering_command_ = steering_angle;

        // Publish Steering Commands
        publishSteering(steering_angle);
    }

    /** ROS Publishing Interface **/
    void publishSteering(double steering_command_deg) {
        std_msgs::msg::Float32 command;
        command.data = steering_command_deg;
        steering_command_pub_->publish(command);
    }

    void publishSteeringDiagnostics(
        bool lqr_command_valid,
        double lqr_steering_command,
        double track_distance,
        geometry_msgs::msg::PoseStamped lqr_track_point,
        lqr::LateralLQR::ErrorMatrix lqr_err
    ) {
        std_msgs::msg::Bool command_valid_msg;
        command_valid_msg.data = lqr_command_valid;
        lqr_command_valid_pub_->publish(command_valid_msg);

        std_msgs::msg::Float32 steering_command_msg;
        steering_command_msg.data = lqr_steering_command;
        lqr_steering_command_pub_->publish(steering_command_msg);

        std_msgs::msg::Float32 track_distance_msg;
        track_distance_msg.data = track_distance;
        track_distance_pub_->publish(track_distance_msg);

        lqr_tracking_point_pub_->publish(lqr_track_point);
        lqr_error_pub_->publish(utils::ROSError(lqr_err));
    }

    /** ROS Callbacks / Subscription Interface **/
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        path_update_time_ = rclcpp::Clock().now();
        lqr_tracking_idx_ = 0; // Reset Tracking
        current_path_ = *msg;
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_update_time_ = rclcpp::Clock().now();
        current_odometry_ = *msg;
    }

    void velocityCallback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg) {
        current_speed_ms_ = (msg->rear_left + msg->rear_right)*0.5*kph2ms;
    }

    void joystickCallback(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {
        override_steering_target_ = msg->steering_cmd;
        override_throttle_target_ = msg->accelerator_cmd;
        override_brake_target_ = msg->brake_cmd;
        override_gear_target_ = msg->gear_cmd;
    }

    void ptReportCallback(const deep_orange_msgs::msg::PtReport::SharedPtr msg) {
        current_gear_ = msg->current_gear;
        current_engine_speed_ = msg->engine_rpm;
        engine_running_ = ( msg->engine_rpm > 500 ) ? true : false;
    }

private:
    //! Debug Interface
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lqr_command_valid_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lqr_steering_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr track_distance_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr lqr_tracking_point_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr lqr_error_pub_;
    //! Command Publishers
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr brake_command_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gear_command_pub_;
    //! Input Data
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr velocity_sub_;
    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr joystick_sub_;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr pt_report_sub_;

    //! Update Timers
    rclcpp::TimerBase::SharedPtr control_timer_;

    //! Track when certain variables have been updated
    rclcpp::Time path_update_time_;
    rclcpp::Time odom_update_time_;

    //! Lateral LQR Controller
    lqr::LateralLQR::Ptr lateral_lqr_;

    //! Current Vehicle State
    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Odometry current_odometry_;
    double current_speed_ms_;
    uint8_t current_gear_;
    uint8_t current_engine_speed_;
    bool engine_running_;
    double last_steering_command_;

    //! LQR Tracking State
    unsigned int lqr_tracking_idx_;

    //! Manual Overrides for when auto mode is disabled
    double override_steering_target_;
    double override_brake_target_;
    double override_throttle_target_;
    uint8_t override_gear_target_;

    //! Misc. Parameters (see notes in constructor)
    double max_steering_angle_deg_;
    double steering_auto_override_deg_;
    double steering_units_multiplier_;
    double pure_pursuit_min_dist_m_;
    double pure_pursuit_max_dist_m_;
    double pure_pursuit_k_vel_m_ms_;
    bool use_tire_velocity_;
    double odometry_timeout_sec_;
    double path_timeout_sec_;
    double steering_max_ddeg_dt_;

    double secs(rclcpp::Time t) {
        return static_cast<double>(t.nanoseconds())*1e-9;
    }
    double secs(rclcpp::Duration t) {
        return static_cast<double>(t.nanoseconds())*1e-9;
    }
}; /* class PathFollowerNode */

} /* namespace bvs_control */

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bvs_control::PathFollowerNode>());
    rclcpp::shutdown();
    return 0;
}
