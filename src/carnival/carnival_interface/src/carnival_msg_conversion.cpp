#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "nif_common/types.h"
#include <raptor_dbw_msgs/msg/accelerator_pedal_report.hpp>
#include <raptor_dbw_msgs/msg/wheel_speed_report.hpp>
#include <raptor_dbw_msgs/msg/brake_report.hpp>

#include <raptor_dbw_msgs/msg/steering_cmd.hpp>
#include <raptor_dbw_msgs/msg/steering_report.hpp>
#include <raptor_dbw_msgs/msg/gear_cmd.hpp>
#include "rclcpp/rclcpp.hpp"
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <lcmrosmsg/msg/eurecar_can_t.hpp>

using namespace std::chrono_literals;

class CarnivalInterface : public rclcpp::Node
{
  public:
    CarnivalInterface()
    : Node("carnival_inerface_node")
    {
    // carnival control command publisher (IAC stack -> carnival)
        /*
        m_throtle_pub = this->create_publisher<CARNIVAL_THROTLE_COMMAND_TYPES>("topic", 10);
        m_brake_pub = this->create_publisher<CARNIVAL_BRAKEE_COMMAND_TYPES>("carnival_brake_cmd", 10);
        m_gear_pub = this->create_publisher<CARNIVAL_GEAR_COMMAND_TYPES>("carnival_gear_cmd", 10);
        m_steering_pub = this->create_publisher<CARNIVAL__COMMAND_TYPES>("carnival_steering_cmd", 10);*/

        m_control_cmd_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/Ackermann/command/auto", 10);

    // carnival health report publisher (carnival -> IAC stack)
        m_wheel_speed_report_pub = this->create_publisher<raptor_dbw_msgs::msg::WheelSpeedReport>("/raptor_dbw_interface/wheel_speed_report", 10);
        m_pt_report_pub = this->create_publisher<deep_orange_msgs::msg::PtReport>("/raptor_dbw_interface/pt_report", 10);
        m_steering_report_pub = this->create_publisher<raptor_dbw_msgs::msg::SteeringReport>("/raptor_dbw_interface/steering_report", 10);
    
    // carnival control command subscriber (IAC stack -> carnival)
        m_steering_cmd_sub = this-> create_subscription<raptor_dbw_msgs::msg::SteeringCmd>(
            "/raptor_dbw_interface/steering_cmd", rclcpp::QoS{10},
            [this](raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg) {steering_cmd_callback(msg);});
        m_velocity_cmd_sub = this-> create_subscription<std_msgs::msg::Float32>(
            "/velocity_planner/des_vel", rclcpp::QoS{10},
            [this](std_msgs::msg::Float32::SharedPtr msg) {velocity_cmd_callback(msg);});

    // IAC control command subscriber (canival -> IAC stack)
        m_ackermann_sub = this-> create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/Ackermann/veh_state", rclcpp::QoS{10},
            [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {ackerman_report_callback(msg);});
        m_cant_sub = this-> create_subscription<lcmrosmsg::msg::EurecarCanT>(
            "/cant", rclcpp::QoS{10},
            [this](lcmrosmsg::msg::EurecarCanT::SharedPtr msg) {cant_report_callback(msg);});
    // timer
        timer_ = this->create_wall_timer(
        10ms, std::bind(&CarnivalInterface::timer_callback, this));
    }

  private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_control_cmd_pub;
    rclcpp::Publisher<raptor_dbw_msgs::msg::WheelSpeedReport>::SharedPtr m_wheel_speed_report_pub;
    rclcpp::Publisher<deep_orange_msgs::msg::PtReport>::SharedPtr m_pt_report_pub;
    rclcpp::Publisher<raptor_dbw_msgs::msg::SteeringReport>::SharedPtr m_steering_report_pub;
    
    rclcpp::Subscription<raptor_dbw_msgs::msg::SteeringCmd>::SharedPtr m_steering_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_velocity_cmd_sub;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr m_ackermann_sub;
    rclcpp::Subscription<lcmrosmsg::msg::EurecarCanT>::SharedPtr m_cant_sub;


    rclcpp::TimerBase::SharedPtr timer_;
    raptor_dbw_msgs::msg::WheelSpeedReport m_wheel_speed_report_msg;
    deep_orange_msgs::msg::PtReport m_pt_report_msg;
    raptor_dbw_msgs::msg::SteeringReport m_steering_report_msg;
    ackermann_msgs::msg::AckermannDrive m_control_cmd_msg;

    void ackerman_report_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg){
        m_steering_report_msg.header = msg->header;
        m_steering_report_msg.steering_wheel_angle = msg->drive.steering_angle * nif::common::constants::RAD2DEG; //radian to degree
        
        m_wheel_speed_report_msg.header = msg->header;
        m_wheel_speed_report_msg.front_left = msg->drive.speed; //Just use speed (m/s), not important
        m_wheel_speed_report_msg.front_right = msg->drive.speed;
        m_wheel_speed_report_msg.rear_left = msg->drive.speed;
        m_wheel_speed_report_msg.rear_right = msg->drive.speed;
        
        m_pt_report_msg.vehicle_speed_kmph = msg->drive.speed; 
    }

    void cant_report_callback(const lcmrosmsg::msg::EurecarCanT::SharedPtr msg){
        m_pt_report_msg.engine_rpm = std::min((double)500 + m_pt_report_msg.vehicle_speed_kmph * 50, 7000.0);
        m_pt_report_msg.engine_on_status = true;
        m_pt_report_msg.current_gear = 4;
    }

    void steering_cmd_callback(const raptor_dbw_msgs::msg::SteeringCmd::SharedPtr msg){
        m_control_cmd_msg.drive.steering_angle = msg->angle_cmd; 
    }

    void velocity_cmd_callback(const std_msgs::msg::Float32::SharedPtr msg){
        m_control_cmd_msg.drive.speed = msg->data;
    }

    void timer_callback()
    {
      //auto message = std_msgs::msg::String();
      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      m_wheel_speed_report_pub->publish(m_wheel_speed_report_msg);
      m_pt_report_pub->publish(m_pt_report_msg);
      m_steering_report_pub->publish(m_steering_report_msg);
      m_control_cmd_pub->publish(m_control_cmd_msg);
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CarnivalInterface>());
  rclcpp::shutdown();
  return 0;
}