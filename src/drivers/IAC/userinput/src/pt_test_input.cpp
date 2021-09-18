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
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PtTest : public rclcpp::Node
{
  public:
    PtTest()
    : Node("pt_test")
    {
      publisher_p_gain_acc = this->create_publisher<std_msgs::msg::Float32>("p_gain_acc", 10);
      publisher_i_gain_acc = this->create_publisher<std_msgs::msg::Float32>("i_gain_acc", 10);
      publisher_p_gain_eg = this->create_publisher<std_msgs::msg::Float32>("p_gain_eg", 10);
      publisher_acc_u_limit = this->create_publisher<std_msgs::msg::Float32>("acc_u_limit", 10);
      publisher_acc_l_limit = this->create_publisher<std_msgs::msg::Float32>("acc_l_limit", 10);
      publisher_v_demand_l_limit = this->create_publisher<std_msgs::msg::Float32>("v_demand_l_limit", 10);
      publisher_v_demand_u_limit = this->create_publisher<std_msgs::msg::Float32>("v_demand_u_limit", 10);
      publisher_velocity_demand_rate_u_limit = this->create_publisher<std_msgs::msg::Float32>("velocity_demand_rate_u_limit", 10);
      publisher_velocity_demand_rate_l_limit = this->create_publisher<std_msgs::msg::Float32>("velocity_demand_rate_l_limit", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&PtTest::timer_callback, this));
    }

  private:

    void timer_callback()
    {
      float p_gain_acc, i_gain_acc, p_gain_eg, 
      acc_u_limit,acc_l_limit ,v_demand_l_limit, v_demand_u_limit,
      velocity_demand_rate_u_limit, velocity_demand_rate_l_limit;
      int choice;
      auto p_gain_acc_1 = std_msgs::msg::Float32();
      auto i_gain_acc_1 = std_msgs::msg::Float32();
      auto p_gain_eg_1 = std_msgs::msg::Float32();
      auto acc_u_limit_1 = std_msgs::msg::Float32();
      auto acc_l_limit_1 = std_msgs::msg::Float32();
      auto v_demand_l_limit_1 = std_msgs::msg::Float32();
      auto v_demand_u_limit_1 = std_msgs::msg::Float32();
      auto velocity_demand_rate_u_limit_1 = std_msgs::msg::Float32();
      auto velocity_demand_rate_l_limit_1 = std_msgs::msg::Float32();

      p_gain_acc_1.data = 0.75;
      i_gain_acc_1.data = 0.30;
      p_gain_eg_1.data = 0.10;
      acc_u_limit_1.data = 7;
      acc_l_limit_1.data = -20;
      v_demand_l_limit_1.data = 0;
      v_demand_u_limit_1.data = 62;
      velocity_demand_rate_u_limit_1.data = 1;
      velocity_demand_rate_l_limit_1.data = -3;

      publisher_p_gain_acc->publish(p_gain_acc_1);
      publisher_i_gain_acc->publish(i_gain_acc_1);
      publisher_p_gain_eg->publish(p_gain_eg_1); 
      publisher_acc_u_limit->publish(acc_u_limit_1);
      publisher_acc_l_limit->publish(acc_l_limit_1);
      publisher_v_demand_l_limit->publish(v_demand_l_limit_1); 
      publisher_v_demand_u_limit->publish(v_demand_u_limit_1); 
      publisher_velocity_demand_rate_u_limit->publish(velocity_demand_rate_u_limit_1); 
      publisher_velocity_demand_rate_l_limit->publish(velocity_demand_rate_l_limit_1);

      // std::cout << "Enter the parameter you want to change: " << std::endl <<"1 - p_gain_acc \n2 - i_gain_acc\n 3 - p_gain_eg" << std::endl;
      // std::cout <<"4 - acc_u_limit\n  5 - acc_l_limit\n 6 - v_demand_l_limit " << std:: endl;
      // std::cout << "7 - v_demand_u_limit\n 8 - velocity_demand_rate_u_limit\n 9 - velocity_demand_rate_l_limit " << std::endl;
      
      // std::cin >> choice;
      // if(choice == 1){
      //     std::cout << "Enter the value of p_gain_acc";
      //     std::cin >> p_gain_acc;
      //     p_gain_acc_1.data = p_gain_acc;
      //     publisher_p_gain_acc->publish(p_gain_acc_1);
      // }
      // else if(choice == 2){
      //     std::cout << "Enter the value of i_gain_acc";
      //     std::cin >> i_gain_acc;
      //     i_gain_acc_1.data = i_gain_acc;
      //     publisher_i_gain_acc->publish(i_gain_acc_1);
      // }
      // else if(choice == 3){
      //     std::cout << "Enter the value of p_gain_eg";
      //     std::cin >> p_gain_eg;  
      //     p_gain_eg_1.data = p_gain_eg; 
      //     publisher_p_gain_eg->publish(p_gain_eg_1); 
      // }
      // else if(choice == 4){
      //     std::cout << "Enter the value of acc_u_limit";
      //     std::cin >> acc_u_limit;  
      //     acc_u_limit_1.data = acc_u_limit; 
      //     publisher_acc_u_limit->publish(acc_u_limit_1); 
      // }
      // else if(choice == 5){
      //     std::cout << "Enter the value of acc_l_limit";
      //     std::cin >> acc_l_limit;  
      //     acc_l_limit_1.data = acc_l_limit; 
      //     publisher_acc_l_limit->publish(acc_l_limit_1); 
      // }
      // else if(choice == 6){
      //     std::cout << "Enter the value of v_demand_l_limit";
      //     std::cin >> v_demand_l_limit;  
      //     v_demand_l_limit_1.data = v_demand_l_limit; 
      //     publisher_v_demand_l_limit->publish(v_demand_l_limit_1); 
      // }
      // else if(choice == 7){
      //     std::cout << "Enter the value of v_demand_u_limit";
      //     std::cin >> v_demand_u_limit;  
      //     v_demand_u_limit_1.data = v_demand_u_limit; 
      //     publisher_v_demand_u_limit->publish(v_demand_u_limit_1); 
      // }
      // else if(choice == 8){
      //     std::cout << "Enter the value of velocity_demand_rate_u_limit";
      //     std::cin >> velocity_demand_rate_u_limit;  
      //     velocity_demand_rate_u_limit_1.data = velocity_demand_rate_u_limit; 
      //     publisher_velocity_demand_rate_u_limit->publish(velocity_demand_rate_u_limit_1); 
      // }
      // else if(choice == 9){
      //     std::cout << "Enter the value of velocity_demand_rate_l_limit";
      //     std::cin >> velocity_demand_rate_l_limit;  
      //     velocity_demand_rate_l_limit_1.data = velocity_demand_rate_l_limit; 
      //     publisher_velocity_demand_rate_l_limit->publish(velocity_demand_rate_l_limit_1); 
      // }
       
    
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_p_gain_acc;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_i_gain_acc;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_p_gain_eg;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_acc_u_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_acc_l_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_v_demand_l_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_v_demand_u_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velocity_demand_rate_u_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velocity_demand_rate_l_limit;
    // size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PtTest>());
  rclcpp::shutdown();
  return 0;
}