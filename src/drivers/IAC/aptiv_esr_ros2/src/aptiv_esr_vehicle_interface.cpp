#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle1.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle2.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle3.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle4.hpp"
#include "delphi_esr_msgs/msg/esr_vehicle5.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "novatel_oem7_msgs/msg/corrimu.hpp"

using namespace std::chrono_literals;

class ESRVehicleInterface : public rclcpp::Node
{
  public:
    ESRVehicleInterface()
    : Node("esr_vehicle_interface")
    {
        publisher_vehicle1 = this->create_publisher<delphi_esr_msgs::msg::EsrVehicle1>("esr_vehicle1", 1);
        publisher_vehicle2 = this->create_publisher<delphi_esr_msgs::msg::EsrVehicle2>("esr_vehicle2", 1);
        publisher_vehicle3 = this->create_publisher<delphi_esr_msgs::msg::EsrVehicle3>("esr_vehicle3", 1);
        publisher_vehicle4 = this->create_publisher<delphi_esr_msgs::msg::EsrVehicle4>("esr_vehicle4", 1);
        publisher_vehicle5 = this->create_publisher<delphi_esr_msgs::msg::EsrVehicle5>("esr_vehicle5", 1);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&ESRVehicleInterface::timer_callback, this));
        sub_imu = this->create_subscription<novatel_oem7_msgs::msg::CORRIMU>(
            "corrimu", 1, std::bind(&ESRVehicleInterface::on_sub_imu, this, std::placeholders::_1)); 
        sub_velocity = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
            "best_vel", 1, std::bind(&ESRVehicleInterface::on_sub_velocity, this, std::placeholders::_1)); 
    }

  private:
    void timer_callback()
    {
        // units need clarification
        delphi_esr_msgs::msg::EsrVehicle1 mes;
        mes.vehicle_speed = velocity_mps; 
        mes.speed_direction = 0; 
        mes.yaw_rate = yaw_rate_radps * 360/6.28;
        mes.yaw_rate_valid = 1;
        publisher_vehicle1->publish(mes);

        // publish all other messages with default value to make the driver happy
        delphi_esr_msgs::msg::EsrVehicle2 mes2;
        mes2.radar_cmd_radiate = 1; 
        mes2.maximum_tracks = 63; 
        mes2.vehicle_speed_valid = 1;
        publisher_vehicle2->publish(mes2);

        // delphi_esr_msgs::msg::EsrVehicle3 mes3;
        // mes3.radar_fov_lr = 30; 
        // mes3.radar_fov_mr = 120;
        // publisher_vehicle3->publish(mes3);
        // delphi_esr_msgs::msg::EsrVehicle4 mes4;
        // publisher_vehicle4->publish(mes4);
        // delphi_esr_msgs::msg::EsrVehicle5 mes5;
        // publisher_vehicle5->publish(mes5);
    }
    void on_sub_imu(const novatel_oem7_msgs::msg::CORRIMU::SharedPtr msg)
    {
        yaw_rate_radps = msg->yaw_rate;
    }
    void on_sub_velocity(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg)
    {
        velocity_mps = msg->hor_speed;
    }

    // ros2 interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrVehicle1>::SharedPtr publisher_vehicle1;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrVehicle2>::SharedPtr publisher_vehicle2;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrVehicle3>::SharedPtr publisher_vehicle3;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrVehicle4>::SharedPtr publisher_vehicle4;
    rclcpp::Publisher<delphi_esr_msgs::msg::EsrVehicle5>::SharedPtr publisher_vehicle5;
    rclcpp::Subscription<novatel_oem7_msgs::msg::CORRIMU>::SharedPtr sub_imu;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr sub_velocity;

    // internal data used to publish whats required by the RADAR 
    double velocity_mps; 
    double yaw_rate_radps; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ESRVehicleInterface>());
  rclcpp::shutdown();
  return 0;
}