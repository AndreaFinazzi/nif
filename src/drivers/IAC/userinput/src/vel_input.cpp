#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class UserInput : public rclcpp::Node
{
  public:
    UserInput()
    : Node("velocity_publisher")
    {
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("velocity_input", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&UserInput::timer_callback, this));
    }

  private:

    void timer_callback()
    {
      float v;
      bool confirm;
      auto velocity = std_msgs::msg::Float32();
      std::cout << "Enter velocity [0.00-50.00]:";
      std::cin >> v;
      if(v>50.00 || v< 0.00){
        throw std::domain_error{"Invalid velocity"};
      }
      std::cout << "Are you sure " << v << " is the velocity you want? [0/1]";
      std::cin >>confirm;
      if(confirm){
        velocity.data = v;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", v);
        publisher_->publish(velocity);
      }
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    // size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UserInput>());
  rclcpp::shutdown();
  return 0;
}