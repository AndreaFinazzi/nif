#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float32.hpp"
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/pt_report.hpp>
#include <deep_orange_msgs/msg/joystick_command.hpp>
#include <iostream>
using namespace std::chrono_literals;
using std::placeholders::_1;

class Diagnostics : public rclcpp::Node
{
  public:
    Diagnostics()
    : Node("emergency_diagnostics"), count_(0)
    {
        // setup QOS to be best effort
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        qos.best_effort();
        publisher_joy_emergency = this->create_publisher<std_msgs::msg::Bool>("/vehicle/emergency_joystick", 10);
        publisher_hb_emergency = this->create_publisher<std_msgs::msg::Bool>("/vehicle/emergency_heartbeat", 10);
        coordinated_stop_publisher = this->create_publisher<std_msgs::msg::Bool>("/vehicle/coordinated_stop", 10);
        diagnostic_hb_publisher = this->create_publisher<std_msgs::msg::Int32>("/diagnostics/heartbeat", 10);
        timer_ = this->create_wall_timer(
        50ms, std::bind(&Diagnostics::timer_callback, this));
        subscriber_joystick = this->create_subscription<deep_orange_msgs::msg::JoystickCommand>(
        "/joystick/command", qos, std::bind(&Diagnostics::counter_callback, this, _1));
        subscriber_cte = this->create_subscription<std_msgs::msg::Float32>(
        "/vehicle/lateral_error", 10, std::bind(&Diagnostics::cte_callback, this, _1));
        subscriber_trajectory = this->create_subscription<autoware_auto_msgs::msg::Trajectory>(
        "/planning/trajectory", 10, std::bind(&Diagnostics::trajectory_callback, this, _1));
        subscriber_ct_status = this->create_subscription<deep_orange_msgs::msg::CtReport>(
        "/raptor_dbw_interface/ct_report", 10, std::bind(&Diagnostics::ct_status_callback, this, _1));
    }

    private:
    
    void timer_callback()
    {
        auto message_joy = std_msgs::msg::Bool();
        auto message_hb = std_msgs::msg::Bool();
        message_joy.data = false;
        message_hb.data = false;
        if(prev_counter != counter){
            t--;
            if(t < 0) {
                t = 0;
            }
            if (counter != default_counter) {
                prev_counter = counter;
            }
            heartbeat = true;
        }
        else{
            t++;
            if(t>= max_counter_drop){
                heartbeat = false;
            }
        }
        if (joy_emergency_stop){
            message_joy.data = true;
        }
        if (!heartbeat){
            message_hb.data = true;
        }
        publisher_joy_emergency->publish(message_joy);
        publisher_hb_emergency->publish(message_hb);

        auto coord_stop_msg = std_msgs::msg::Bool();
        coord_stop_msg.data = false;
        if(cte_threshold_passed){
            coord_stop_msg.data = true;
        }
        coordinated_stop_publisher->publish(coord_stop_msg);
        
        auto joy_hb = std_msgs::msg::Int32();
        joy_hb.data = counter_hb;
        diagnostic_hb_publisher->publish(joy_hb);
        counter_hb++;
        if(counter_hb == 8){
            counter_hb = 0;
        } 
        
    }

    void cte_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (msg->data >= max_lateral_error){
            cte_threshold_passed = true;
        }

    }
    void trajectory_callback(const autoware_auto_msgs::msg::Trajectory::SharedPtr msg)
    {
        if(mode == 8 && msg->points.size() < 10){
            min_trajectory_indicator = true;
        }
    }
    void counter_callback(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg)
    {   
        // parse counter
        counter = msg->counter;
        // parse emergency 
        if(msg->emergency_stop == 1)
        {
            joy_emergency_stop = true; 
        }
        
    }
    void ct_status_callback(const deep_orange_msgs::msg::CtReport::SharedPtr msg)
    {
        mode = msg->ct_state;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_joy_emergency;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_hb_emergency;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr coordinated_stop_publisher;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr diagnostic_hb_publisher;
    size_t count_;
    unsigned int mode = 255;
    const int max_counter_drop = 20;
    const float max_lateral_error = 3.0; //(in meters)
    int t = 0;
    int counter_hb = 0;
    
    const int default_counter = 502;
    int prev_counter = default_counter-1 ;
    int counter = default_counter; 
    bool oil_pressure_threshold = false;
    bool cte_threshold_passed = false;
    bool joy_emergency_stop = false;
    bool min_trajectory_indicator = false;
    bool heartbeat = true;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_cte;
    rclcpp::Subscription<autoware_auto_msgs::msg::Trajectory>::SharedPtr subscriber_trajectory;
    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr subscriber_joystick;
    rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr subscriber_ct_status;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Diagnostics>());
  rclcpp::shutdown();
  return 0;
}
