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
#include "deep_orange_msgs/msg/tire_report.hpp"
#include <deep_orange_msgs/msg/joystick_command.hpp>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nif_msgs/msg/system_status.hpp"
#include "nif_msgs/msg/telemetry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nif_msgs/msg/localization_status.hpp"
#include <visualization_msgs/msg/marker_array.hpp>

// UDP stuff
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
using namespace boost::asio;

using namespace std::chrono_literals;

const unsigned short int OUT_DATA_FRAME_SIZE = 100;
const unsigned short int IN_DATA_FRAME_SIZE = 25;


class Telemetry : public rclcpp::Node
{
  public:
    Telemetry()
    : Node("telemetry"),
      send_telemetry_socket(io_service_main),
      recv_basestation_socket(io_service_main)
    {
      this->declare_parameter("enable_udp", true);
      this->declare_parameter("ego_ip", "10.42.4.200");
      this->declare_parameter("bst_ip","10.42.4.79");

      this->declare_parameter("ego_port", 23531);
      this->declare_parameter("bst_port",23431);

      // setup UDP interfaces
      this->udp_enabled = this->get_parameter("enable_udp").as_bool();

      send_telemetry_ip = this->get_parameter("bst_ip").as_string();
      send_telemetry_port = this->get_parameter("bst_port").as_int();
      send_telemetry_socket.open(ip::udp::v4());
      send_telemetry_endpoint = ip::udp::endpoint(
          ip::address::from_string(send_telemetry_ip), send_telemetry_port);
      RCLCPP_INFO(this->get_logger(),
                  "Established connection to send telemetry to : %s:%u",
                  send_telemetry_ip.c_str(), send_telemetry_port);

      recv_basestation_ip = this->get_parameter("ego_ip").as_string();
      recv_basestation_port = this->get_parameter("ego_port").as_int();;


      // setup QOS to be best effort
      auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
      qos.best_effort();
      pub_system_status = this->create_publisher<nif_msgs::msg::SystemStatus>("/nif_telemetry/system_status", qos);
      pub_telemetry = this->create_publisher<nif_msgs::msg::Telemetry>("/nif_telemetry/telemetry", qos);
      pub_reference_path = this->create_publisher<nav_msgs::msg::Path>("/nif_telemetry/path_global", qos);
      pub_perception_result = this->create_publisher<visualization_msgs::msg::MarkerArray>("/nif_telemetry/perception_result", qos);

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
        "/control_joint_lqr/desired_velocity_mps", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::command_desired_velocity_callback, this, std::placeholders::_1));
      sub_command_gear = this->create_subscription<std_msgs::msg::UInt8>(
        "/joystick/gear_cmd", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::command_gear_callback, this, std::placeholders::_1));

      sub_control_lqr_error = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/control_joint_lqr/lqr_error", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::control_lqr_error_callback, this, std::placeholders::_1));

      sub_reference_path = this->create_subscription<nav_msgs::msg::Path>(
        "/planning/dynamic/vis/traj_global", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::reference_path_callback, this, std::placeholders::_1));

      sub_perception_result = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/clustered_markers", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::perception_result_callback, this, std::placeholders::_1));

      sub_tire_report = this->create_subscription<deep_orange_msgs::msg::TireReport>(
        "/raptor_dbw_interface/tire_report", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::tire_report_callback, this, std::placeholders::_1));

      sub_wall_distance_inner = this->create_subscription<std_msgs::msg::Float32>(
        "/detected_inner_distance", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::wall_distance_inner_callback, this, std::placeholders::_1));
      sub_wall_distance_outer = this->create_subscription<std_msgs::msg::Float32>(
        "/detected_outer_distance", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::wall_distance_outer_callback, this, std::placeholders::_1));

      sub_oppo_prediction_path = this->create_subscription<nav_msgs::msg::Path>(
        "/oppo/vis/prediction", nif::common::constants::QOS_SENSOR_DATA, std::bind(&Telemetry::oppo_prediction_path_callback, this, std::placeholders::_1));


    if (this->udp_enabled) {

    //  Republish joystick command from UDP
      pub_joystick_command =
          this->create_publisher<deep_orange_msgs::msg::JoystickCommand>(
              "/joystick/command", rclcpp::QoS{10});

      // timer which handles sending to base station
      send_bs_timer_ = this->create_wall_timer(
          50ms, std::bind(&Telemetry::send_bs_callback, this));

      // timer which handles receiving from base station
      rec_bs_timer_ = this->create_wall_timer(
          5ms, std::bind(&Telemetry::rec_bs_callback, this));
      recv_basestation_socket.open(ip::udp::v4());
      recv_basestation_endpoint =
          ip::udp::endpoint(ip::address::from_string(recv_basestation_ip),
                            recv_basestation_port);
      recv_basestation_socket.bind(recv_basestation_endpoint);
      recv_basestation_socket.non_blocking(true);
      send_telemetry_endpoint = ip::udp::endpoint(
          ip::address::from_string(send_telemetry_ip), send_telemetry_port);
      RCLCPP_INFO(
          this->get_logger(),
          "Established connection to receive basestation commands on : %s:%u",
          recv_basestation_ip.c_str(), recv_basestation_port);
    }

      timer_ = this->create_wall_timer(
        50ms, std::bind(&Telemetry::timer_callback, this));
    }

  private:
    void send_bs_callback() {
        // double lqr_error = -1.0;
        // if (has_error) {
        //     lqr_error = static_cast<double>(msg_lqr_error.data[0]);
        // }
        // data frame structure for telemtry to basestation message
        double data_frame[OUT_DATA_FRAME_SIZE] = {
          // -------  MSG TELEMETRY 
            /* 00 */ static_cast<double>(msg_telemetry.misc_report.stamp.nanosec),
            /* 01 */ static_cast<double>(msg_telemetry.misc_report.sys_state),
            /* 02 */ static_cast<double>(msg_telemetry.misc_report.ct_state),
            /* 03 */ static_cast<double>(msg_telemetry.misc_report.battery_voltage),
            /* 04 */ static_cast<double>(msg_telemetry.pt_report.stamp.nanosec),
            /* 05 */ static_cast<double>(msg_telemetry.pt_report.fuel_pressure),
            /* 06 */ static_cast<double>(msg_telemetry.pt_report.engine_oil_temperature),
            /* 07 */ static_cast<double>(msg_telemetry.pt_report.engine_coolant_temperature),
            /* 08 */ static_cast<double>(msg_telemetry.pt_report.engine_on_status),
            /* 09 */ static_cast<double>(msg_telemetry.pt_report.current_gear),
            /* 10 */ static_cast<double>(msg_telemetry.pt_report.vehicle_speed_kmph),
            /* 11 */ static_cast<double>(msg_telemetry.localization.stamp.nanosec),
            /* 12 */ static_cast<double>(msg_telemetry.localization.odometry.pose.position.x),
            /* 13 */ static_cast<double>(msg_telemetry.localization.odometry.pose.position.y),
            /* 14 */ static_cast<double>(msg_telemetry.localization.uncertainty),
            /* 15 */ static_cast<double>(msg_telemetry.localization.localization_status_code),
            /* 16 */ static_cast<double>(msg_telemetry.localization.detected_inner_distance),
            /* 17 */ static_cast<double>(msg_telemetry.localization.detected_outer_distance),
            /* 18 */ static_cast<double>(msg_telemetry.localization.pos_type_0),
            /* 19 */ static_cast<double>(msg_telemetry.localization.pos_type_1),
            /* 20 */ static_cast<double>(msg_telemetry.localization.heading),
            /* 21 */ static_cast<double>(msg_telemetry.control.stamp.nanosec),
            /* 22 */ static_cast<double>(msg_telemetry.control.steering_cmd),
            /* 23 */ static_cast<double>(msg_telemetry.control.brake_cmd),
            /* 24 */ static_cast<double>(msg_telemetry.control.accelerator_cmd),
            /* 25 */ static_cast<double>(msg_telemetry.control.gear_cmd),
            /* 26 */ static_cast<double>(msg_telemetry.control.desired_velocity_mps),
            /* 27 */ static_cast<double>(msg_telemetry.control.crosstrack_error),
            /* 28 */ static_cast<double>(msg_telemetry.kinematic.stamp.nanosec),
            /* 29 */ static_cast<double>(msg_telemetry.kinematic.wheel_speed_mps),
            /* 30 */ static_cast<double>(msg_telemetry.kinematic.steering_wheel_angle_deg),
            /* 31 */ static_cast<double>(msg_telemetry.tires.stamp.nanosec),
            /* 32 */ static_cast<double>(msg_telemetry.tires.temp_front_right),
            /* 33 */ static_cast<double>(msg_telemetry.tires.temp_front_left),
            /* 34 */ static_cast<double>(msg_telemetry.tires.temp_rear_right),
            /* 35 */ static_cast<double>(msg_telemetry.tires.temp_rear_left),
          // -------  MSG SYSTEM STATUS 
            /* 36 */ static_cast<double>(msg_system_status.header.stamp.nanosec),
            /* 37 */ static_cast<double>(msg_system_status.autonomy_status.longitudinal_autonomy_enabled),
            /* 38 */ static_cast<double>(msg_system_status.autonomy_status.lateral_autonomy_enabled),
            /* 39 */ static_cast<double>(msg_system_status.autonomy_status.emergency_mode_enabled),
            /* 40 */ static_cast<double>(msg_system_status.health_status.system_failure),
            /* 41 */ static_cast<double>(msg_system_status.health_status.communication_failure),
            /* 42 */ static_cast<double>(msg_system_status.health_status.localization_failure),
            /* 43 */ static_cast<double>(msg_system_status.health_status.commanded_stop),
            /* 44 */ static_cast<double>(msg_system_status.health_status.system_status_code),
            /* 45 */ static_cast<double>(msg_system_status.mission_status.stamp_last_update.nanosec),
            /* 46 */ static_cast<double>(msg_system_status.mission_status.track_flag),
            /* 47 */ static_cast<double>(msg_system_status.mission_status.veh_flag),
            /* 48 */ static_cast<double>(msg_system_status.mission_status.mission_status_code),
            /* 49 */ static_cast<double>(msg_system_status.mission_status.max_velocity_mps),
          // -------  MSG REFERENCE PATH 
            /* 50 */ static_cast<double>(msg_reference_path.header.stamp.nanosec),
            /* 51 */ msg_reference_path.poses.size() > 0 ? static_cast<double>(msg_reference_path.poses[0].pose.position.x) : 0.0,
            /* 52 */ msg_reference_path.poses.size() > 0 ? static_cast<double>(msg_reference_path.poses[0].pose.position.y) : 0.0,
            /* 53 */ msg_reference_path.poses.size() > 1 ? static_cast<double>(msg_reference_path.poses[1].pose.position.x) : 0.0,
            /* 54 */ msg_reference_path.poses.size() > 1 ? static_cast<double>(msg_reference_path.poses[1].pose.position.y) : 0.0,
            /* 55 */ msg_reference_path.poses.size() > 2 ? static_cast<double>(msg_reference_path.poses[2].pose.position.x) : 0.0,
            /* 56 */ msg_reference_path.poses.size() > 2 ? static_cast<double>(msg_reference_path.poses[2].pose.position.y) : 0.0,
            /* 57 */ msg_reference_path.poses.size() > 3 ? static_cast<double>(msg_reference_path.poses[3].pose.position.x) : 0.0,
            /* 58 */ msg_reference_path.poses.size() > 3 ? static_cast<double>(msg_reference_path.poses[3].pose.position.y) : 0.0,
            /* 59 */ msg_reference_path.poses.size() > 4 ? static_cast<double>(msg_reference_path.poses[4].pose.position.x) : 0.0,
            /* 60 */ msg_reference_path.poses.size() > 4 ? static_cast<double>(msg_reference_path.poses[4].pose.position.y) : 0.0,
            /* 61 */ msg_reference_path.poses.size() > 5 ? static_cast<double>(msg_reference_path.poses[5].pose.position.x) : 0.0,
            /* 62 */ msg_reference_path.poses.size() > 5 ? static_cast<double>(msg_reference_path.poses[5].pose.position.y) : 0.0,
            /* 63 */ msg_reference_path.poses.size() > 6 ? static_cast<double>(msg_reference_path.poses[6].pose.position.x) : 0.0,
            /* 64 */ msg_reference_path.poses.size() > 6 ? static_cast<double>(msg_reference_path.poses[6].pose.position.y) : 0.0,
            /* 65 */ msg_reference_path.poses.size() > 7 ? static_cast<double>(msg_reference_path.poses[7].pose.position.x) : 0.0,
            /* 66 */ msg_reference_path.poses.size() > 7 ? static_cast<double>(msg_reference_path.poses[7].pose.position.y) : 0.0,
            /* 67 */ msg_reference_path.poses.size() > 8 ? static_cast<double>(msg_reference_path.poses[8].pose.position.x) : 0.0,
            /* 68 */ msg_reference_path.poses.size() > 8 ? static_cast<double>(msg_reference_path.poses[8].pose.position.y) : 0.0,
          // -------  MSG PERCEPTION RESULT 
            /* 69 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[0].pose.position.x) : 0.0,
            /* 70 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[0].pose.position.y) : 0.0,
            /* 71 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[0].pose.orientation.w) : 0.0,
            /* 72 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[0].pose.orientation.z) : 0.0,
            /* 73 */ msg_perception_result.markers.size() > 1 ? static_cast<double>(msg_perception_result.markers[1].pose.position.x) : 0.0,
            /* 74 */ msg_perception_result.markers.size() > 1 ? static_cast<double>(msg_perception_result.markers[1].pose.position.y) : 0.0,
            /* 75 */ msg_perception_result.markers.size() > 1 ? static_cast<double>(msg_perception_result.markers[1].pose.orientation.w) : 0.0,
            /* 76 */ msg_perception_result.markers.size() > 1 ? static_cast<double>(msg_perception_result.markers[1].pose.orientation.z) : 0.0,
            /* 77 */ msg_perception_result.markers.size() > 2 ? static_cast<double>(msg_perception_result.markers[2].pose.position.x) : 0.0,
            /* 78 */ msg_perception_result.markers.size() > 2 ? static_cast<double>(msg_perception_result.markers[2].pose.position.y) : 0.0,
            /* 79 */ msg_perception_result.markers.size() > 2 ? static_cast<double>(msg_perception_result.markers[2].pose.orientation.w) : 0.0,
            /* 80 */ msg_perception_result.markers.size() > 2 ? static_cast<double>(msg_perception_result.markers[2].pose.orientation.z) : 0.0,
            /* 81 */ msg_perception_result.markers.size() > 3 ? static_cast<double>(msg_perception_result.markers[3].pose.position.x) : 0.0,
            /* 82 */ msg_perception_result.markers.size() > 3 ? static_cast<double>(msg_perception_result.markers[3].pose.position.y) : 0.0,
            /* 83 */ msg_perception_result.markers.size() > 3 ? static_cast<double>(msg_perception_result.markers[3].pose.orientation.w) : 0.0,
            /* 84 */ msg_perception_result.markers.size() > 3 ? static_cast<double>(msg_perception_result.markers[3].pose.orientation.z) : 0.0,
            /* 85 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[4].pose.position.x) : 0.0,
            /* 86 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[4].pose.position.y) : 0.0,
            /* 87 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[4].pose.orientation.w) : 0.0,
            /* 88 */ msg_perception_result.markers.size() > 0 ? static_cast<double>(msg_perception_result.markers[4].pose.orientation.z) : 0.0,
            /* 89 */ static_cast<double>(msg_telemetry.localization.odometry.pose.orientation.z),
            /* 90 */ static_cast<double>(msg_telemetry.localization.odometry.pose.orientation.w),
            /* 91 */ static_cast<double>(msg_oppo_prediction_path.header.stamp.nanosec),
            /* 92 */ msg_oppo_prediction_path.poses.size() > 0 ? static_cast<double>(msg_oppo_prediction_path.poses[0].pose.position.x) : 0.0,
            /* 93 */ msg_oppo_prediction_path.poses.size() > 0 ? static_cast<double>(msg_oppo_prediction_path.poses[0].pose.position.y) : 0.0,
            /* 94 */ msg_oppo_prediction_path.poses.size() > 1 ? static_cast<double>(msg_oppo_prediction_path.poses[1].pose.position.x) : 0.0,
            /* 95 */ msg_oppo_prediction_path.poses.size() > 1 ? static_cast<double>(msg_oppo_prediction_path.poses[1].pose.position.y) : 0.0,
            /* 96 */ msg_oppo_prediction_path.poses.size() > 2 ? static_cast<double>(msg_oppo_prediction_path.poses[2].pose.position.x) : 0.0,
            /* 97 */ msg_oppo_prediction_path.poses.size() > 2 ? static_cast<double>(msg_oppo_prediction_path.poses[2].pose.position.y) : 0.0,
            /* 98 */ msg_oppo_prediction_path.poses.size() > 3 ? static_cast<double>(msg_oppo_prediction_path.poses[3].pose.position.x) : 0.0,
            /* 99 */ msg_oppo_prediction_path.poses.size() > 3 ? static_cast<double>(msg_oppo_prediction_path.poses[3].pose.position.y) : 0.0
            };
        boost::system::error_code err;
        send_telemetry_socket.send_to(buffer(data_frame, sizeof(data_frame)),
                                      send_telemetry_endpoint, 0, err);
    }

    void rec_bs_callback() {
        boost::system::error_code err;
        // receive data from basestation
        boost::array<double, IN_DATA_FRAME_SIZE> recv_basestation_buffer;
        // read all messages until none is there anymore
        int i = 0;
        while (recv_basestation_socket.receive_from(
                   boost::asio::buffer(recv_basestation_buffer),
                   recv_basestation_endpoint, 0, err) != 0) {
            i++;
        }
        // only publish when a new topic was received
        // this is required such that the timeouts still work
        if (i > 0) {
            msg_joystick_command.counter = recv_basestation_buffer[0];
            msg_joystick_command.emergency_stop = recv_basestation_buffer[1];
            msg_joystick_command.joy_enable = recv_basestation_buffer[2];
            msg_joystick_command.steering_cmd = recv_basestation_buffer[3];
            msg_joystick_command.brake_cmd = recv_basestation_buffer[4];
            msg_joystick_command.accelerator_cmd = recv_basestation_buffer[5];
            msg_joystick_command.gear_cmd = recv_basestation_buffer[6];
            msg_joystick_command.stamp.sec = recv_basestation_buffer[7];
            msg_joystick_command.stamp.nanosec = recv_basestation_buffer[8];

            // msg_rc_to_ct.track_flag = recv_basestation_buffer[7];
            // msg_rc_to_ct.rolling_counter = recv_basestation_buffer[8];
            // msg_rc_to_ct.veh_flag = recv_basestation_buffer[9];

            // TODO: Add vehicle condition to rc_to_ct

            // msg_base_to_car_summary.track_flag = msg_rc_to_ct.track_flag;
            // // TODO: Add vehicle codition to base_to_car_summary

            // auto_enabled_msg.data =
            //     static_cast<bool>(recv_basestation_buffer[10]);
            // max_speed_msg.data =
            //     static_cast<double>(recv_basestation_buffer[11]);

            // msg_joystick_command.joystick_healthy =
            //     (static_cast<int>(recv_basestation_buffer[12]) == 1);

            // publish everything on ros2
            pub_joystick_command->publish(msg_joystick_command);
            // pub_rc_to_ct->publish(msg_rc_to_ct);
            // pub_flag_report->publish(msg_base_to_car_summary);

            // autoPub->publish(auto_enabled_msg);
            // maxSpeedPub->publish(max_speed_msg);
        }
    }
    double tire_array_average(const deep_orange_msgs::msg::TireReport::_fr_tire_temperature_type & in) 
    {
      if (in.empty()) return -100.0;

      double average = 0.0;
      for (double && elem : in) {
        average += elem;
      }
      return average / in.size();
    }
    void timer_callback()
    {
      pub_system_status->publish(msg_system_status);

      // Get tires temp average
      msg_telemetry.tires.stamp = in_tire_report.stamp;
      msg_telemetry.tires.temp_front_left = tire_array_average(in_tire_report.fl_tire_temperature);
      msg_telemetry.tires.temp_front_right = tire_array_average(in_tire_report.fr_tire_temperature);
      msg_telemetry.tires.temp_rear_left = tire_array_average(in_tire_report.rl_tire_temperature);
      msg_telemetry.tires.temp_rear_right = tire_array_average(in_tire_report.rr_tire_temperature);

      pub_telemetry->publish(msg_telemetry);

      // Reference path
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

// TODO temporary to ease UDP 
        msg_reference_path = std::move(path_sampled);
      }

      // Prediction path
      if (!in_oppo_prediction_path.poses.empty()) {
        nav_msgs::msg::Path path_sampled{};

        path_sampled.header = std::move(in_oppo_prediction_path.header);
        unsigned int step_size =  floor(this->in_oppo_prediction_path.poses.size() / 10);

        for (int i = 0; i < this->in_oppo_prediction_path.poses.size(); i += step_size) 
        {
          path_sampled.poses.push_back(this->in_oppo_prediction_path.poses[i]);
        }
        // Always include last point
        path_sampled.poses.push_back(this->in_oppo_prediction_path.poses[this->in_oppo_prediction_path.poses.size() - 1]);

        pub_reference_path->publish(path_sampled);

// TODO temporary to ease UDP 
        msg_oppo_prediction_path = std::move(path_sampled);
      }


      // Perception result
      pub_perception_result->publish(msg_perception_result);
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
        msg_telemetry.localization.pos_type_0 = msg->pos_type_0;
        msg_telemetry.localization.pos_type_1 = msg->pos_type_1;
        msg_telemetry.localization.heading = msg->heading;
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
    void oppo_prediction_path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
      in_oppo_prediction_path = std::move(*msg);
    }
    void perception_result_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
      msg_perception_result = std::move(*msg);

      for (auto &&marker : msg_perception_result.markers)
      {
        // Fake marker orientation as temp solution
        marker.pose.orientation = msg_telemetry.localization.odometry.pose.orientation;
      }
    }
    void tire_report_callback(const deep_orange_msgs::msg::TireReport::SharedPtr msg)
    {
      in_tire_report = std::move(*msg);
    }
    void wall_distance_inner_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      msg_telemetry.localization.detected_inner_distance = msg->data;
    }    
    void wall_distance_outer_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
      msg_telemetry.localization.detected_outer_distance = msg->data;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr send_bs_timer_;
    rclcpp::TimerBase::SharedPtr rec_bs_timer_;

    rclcpp::Publisher<nif_msgs::msg::SystemStatus>::SharedPtr pub_system_status;
    rclcpp::Publisher<nif_msgs::msg::Telemetry>::SharedPtr pub_telemetry;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_reference_path;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_perception_result;

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
    
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_perception_result;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_oppo_prediction_path;

    rclcpp::Subscription<deep_orange_msgs::msg::TireReport>::SharedPtr sub_tire_report;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_wall_distance_inner;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_wall_distance_outer;

    // FROM UDP
    rclcpp::Publisher<deep_orange_msgs::msg::JoystickCommand>::SharedPtr pub_joystick_command;
    
    
    nif_msgs::msg::SystemStatus msg_system_status;
    nif_msgs::msg::Telemetry msg_telemetry;

    nav_msgs::msg::Path in_reference_path;
    nav_msgs::msg::Path msg_reference_path;
    
    nav_msgs::msg::Path in_oppo_prediction_path;
    nav_msgs::msg::Path msg_oppo_prediction_path;
    visualization_msgs::msg::MarkerArray msg_perception_result;

    deep_orange_msgs::msg::JoystickCommand msg_joystick_command;

    deep_orange_msgs::msg::TireReport in_tire_report;

    // udp stuff
    bool udp_enabled = true;

    io_service io_service_main;
    ip::udp::socket send_telemetry_socket;
    ip::udp::socket recv_basestation_socket;
    ip::udp::endpoint send_telemetry_endpoint;
    ip::udp::endpoint recv_basestation_endpoint;
    std::string send_telemetry_ip;
    unsigned int send_telemetry_port;
    std::string recv_basestation_ip;
    unsigned int recv_basestation_port;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Telemetry>());
  rclcpp::shutdown();
  return 0;
}
