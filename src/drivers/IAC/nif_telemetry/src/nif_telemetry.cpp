#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nif_common/constants.h"
#include "rclcpp/rclcpp.hpp"
#include "raptor_dbw_msgs/msg/wheel_speed_report.hpp"
#include "raptor_dbw_msgs/msg/steering_report.hpp"
#include "deep_orange_msgs/msg/ct_report.hpp"
#include "deep_orange_msgs/msg/pt_report.hpp"
#include "deep_orange_msgs/msg/misc_report.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nif_msgs/msg/system_status.hpp"
#include "nif_msgs/msg/telemetry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nif_msgs/msg/localization_status.hpp"

using namespace std::chrono_literals;

class Telemetry : public rclcpp::Node
{
  public:
    Telemetry()
    : Node("telemetry")
    {
      // setup QOS to be best effort
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos.best_effort();
      pub_system_status = this->create_publisher<nif_msgs::msg::SystemStatus>("/nif_telemetry/system_status", qos);
      pub_telemetry = this->create_publisher<nif_msgs::msg::Telemetry>("/nif_telemetry/telemetry", qos);
      pub_reference_path = this->create_publisher<nav_msgs::msg::Path>("/nif_telemetry/path_global", qos);
      // pub_perception_summary = this->create_publisher<?>("/nif_telemetry/perception_summary", qos);

      sub_ct_report = this->create_subscription<deep_orange_msgs::msg::CtReport>(
        "/raptor_dbw_interface/ct_report", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::ct_report_callback, this, std::placeholders::_1));
      sub_pt_report = this->create_subscription<deep_orange_msgs::msg::PtReport>(
        "/raptor_dbw_interface/pt_report", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::pt_report_callback, this, std::placeholders::_1));
      sub_misc_report_do = this->create_subscription<deep_orange_msgs::msg::MiscReport>(
        "/raptor_dbw_interface/misc_report_do", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::misc_report_do_callback, this, std::placeholders::_1));
      sub_system_status = this->create_subscription<nif_msgs::msg::SystemStatus>(
        "/system/status", nif::common::constants::QOS_INTERNAL_STATUS, std::bind(&Telemetry::system_status_callback, this, std::placeholders::_1));
      sub_wheel_speed_report = this->create_subscription<raptor_dbw_msgs::msg::WheelSpeedReport>(
        "/raptor_dbw_interface/wheel_speed_report", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::wheel_speed_callback, this, std::placeholders::_1));
      sub_steeering_report = this->create_subscription<raptor_dbw_msgs::msg::SteeringReport>(
        "/raptor_dbw_interface/steering_report", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::steering_report_callback, this, std::placeholders::_1));
      
      sub_localization_status = this->create_subscription<nif_msgs::msg::LocalizationStatus>(
        "/aw_localization/ekf/status", nif::common::constants::QOS_INTERNAL_STATUS, std::bind(&Telemetry::localization_status_callback, this, std::placeholders::_1));
      sub_ego_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        "/aw_localization/ekf/odom", nif::common::constants::QOS_EGO_ODOMETRY, std::bind(&Telemetry::ego_odometry_callback, this, std::placeholders::_1));
      
      sub_command_steering = this->create_subscription<std_msgs::msg::Float32>(
        "/joystick/steering_cmd", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::command_steering_callback, this, std::placeholders::_1));
      sub_command_accelerator = this->create_subscription<std_msgs::msg::Float32>(
        "/joystick/accelerator_cmd", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::command_accelerator_callback, this, std::placeholders::_1));
      sub_command_brake = this->create_subscription<std_msgs::msg::Float32>(
        "/joystick/brake_cmd", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::command_brake_callback, this, std::placeholders::_1));
      sub_command_desired_velocity = this->create_subscription<std_msgs::msg::Float32>(
        "/velocity_planner/des_vel", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::command_desired_velocity_callback, this, std::placeholders::_1));
      sub_command_gear = this->create_subscription<std_msgs::msg::UInt8>(
        "/joystick/gear_cmd", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::command_gear_callback, this, std::placeholders::_1));
        // sub_perception_result

      sub_control_lqr_error = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/control_joint_lqr/lqr_error", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::control_lqr_error_callback, this, std::placeholders::_1));


      sub_reference_path = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/graph/path_global", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::reference_path_callback, this, std::placeholders::_1));

      timer_ = this->create_wall_timer(
        100ms, std::bind(&Telemetry::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      pub_system_status->publish(msg_system_status);
      pub_telemetry->publish(msg_telemetry);

      if (!in_reference_path.poses.empty()) {
        nav_msgs::msg::Path path_sampled{};

        path_sampled.header = std::move(in_reference_path.header);
        unsigned int step_size =  floor(this->in_reference_path.poses.size() / 10);

        for (int i = 0; i < this->in_reference_path.poses.size(); i += step_size) 
        {
          path_sampled.poses.push_back(this->in_reference_path.poses[i]);
        }
        // Always include last point
        path_sampled.poses.push_back(this->in_reference_path.poses[this->in_reference_path.poses.size() - 1]);

        pub_reference_path->publish(path_sampled);
      }
    }
    void ct_report_callback(const deep_orange_msgs::msg::CtReport::SharedPtr msg)
    {
      // Stamp for misc_report is taken from misc_report_do
        msg_telemetry.misc_report.ct_state = msg->ct_state;
    }
    void pt_report_callback(const deep_orange_msgs::msg::PtReport::SharedPtr msg)
    {
        msg_telemetry.pt_report.stamp = msg->stamp;
        msg_telemetry.pt_report.current_gear = msg->current_gear;
        msg_telemetry.pt_report.engine_coolant_temperature = msg->engine_coolant_temperature;
        msg_telemetry.pt_report.engine_oil_temperature = msg->engine_oil_temperature;
        msg_telemetry.pt_report.engine_on_status = msg->engine_on_status;
        msg_telemetry.pt_report.fuel_level = msg->fuel_level;
        msg_telemetry.pt_report.fuel_pressure = msg->fuel_pressure;
        msg_telemetry.pt_report.vehicle_speed_kmph = msg->vehicle_speed_kmph;
    }
    void misc_report_do_callback(const deep_orange_msgs::msg::MiscReport::SharedPtr msg)
    {
        msg_telemetry.misc_report.stamp = msg->stamp;
        msg_telemetry.misc_report.sys_state = msg->sys_state;
        msg_telemetry.misc_report.battery_voltage = msg->battery_voltage;
    }
    void system_status_callback(const nif_msgs::msg::SystemStatus::SharedPtr msg)
    {
        msg_system_status = *msg;
    }
    void ego_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        msg_telemetry.localization.odometry.header = std::move(msg->header);
        msg_telemetry.localization.odometry.pose = std::move(msg->pose.pose);
    }
    void wheel_speed_callback(const raptor_dbw_msgs::msg::WheelSpeedReport::SharedPtr msg)
    {
        auto speed_kph = (msg->front_left + msg->front_right) / 2;
        msg_telemetry.kinematic.wheel_speed_mps = speed_kph / 3.6;
    }
    void steering_report_callback(const raptor_dbw_msgs::msg::SteeringReport::SharedPtr msg)
    {
        msg_telemetry.kinematic.stamp = msg->header.stamp;
        msg_telemetry.kinematic.steering_wheel_angle_deg = msg->steering_wheel_angle;
    }
    void localization_status_callback(const nif_msgs::msg::LocalizationStatus::SharedPtr msg)
    {
        msg_telemetry.localization.stamp = msg->stamp;
        msg_telemetry.localization.uncertainty = msg->uncertainty;
        msg_telemetry.localization.localization_status_code = msg->localization_status_code;
    }
    void command_steering_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      msg_telemetry.control.stamp = this->now();
      msg_telemetry.control.steering_cmd = msg->data;
    }
    void command_accelerator_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      msg_telemetry.control.accelerator_cmd = msg->data;
    }
    void command_brake_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      msg_telemetry.control.brake_cmd = msg->data;
    }
    void command_desired_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      msg_telemetry.control.desired_velocity_mps = msg->data;
    }
    void command_gear_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
      msg_telemetry.control.gear_cmd = msg->data;
    }    
    void control_lqr_error_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      if (!msg->data.empty())
        msg_telemetry.control.crosstrack_error = msg->data[0];
    }
    void reference_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
      in_reference_path = std::move(*msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nif_msgs::msg::SystemStatus>::SharedPtr pub_system_status;
    rclcpp::Publisher<nif_msgs::msg::Telemetry>::SharedPtr pub_telemetry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_reference_path;

    rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr sub_ct_report;
    rclcpp::Subscription<deep_orange_msgs::msg::PtReport>::SharedPtr sub_pt_report;
    rclcpp::Subscription<deep_orange_msgs::msg::MiscReport>::SharedPtr sub_misc_report_do;
    rclcpp::Subscription<nif_msgs::msg::SystemStatus>::SharedPtr sub_system_status;

    rclcpp::Subscription<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr sub_wheel_speed_report;
    rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr sub_steeering_report;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ego_odometry;
    rclcpp::Subscription<nif_msgs::msg::LocalizationStatus>::SharedPtr sub_localization_status;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_command_steering;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_command_accelerator;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_command_brake;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_command_desired_velocity;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_command_gear;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_control_lqr_error;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_reference_path;

    nif_msgs::msg::SystemStatus msg_system_status;
    nif_msgs::msg::Telemetry msg_telemetry;

    nav_msgs::msg::Path in_reference_path;
    nav_msgs::msg::Path msg_reference_path;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry>());
  rclcpp::shutdown();
  return 0;
}
