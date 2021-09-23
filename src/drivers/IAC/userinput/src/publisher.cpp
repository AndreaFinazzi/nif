#include <memory>
#include <deep_orange_msgs/msg/rc_to_ct.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("publisher")
    {
      subscription_ = this->create_subscription<std_msgs::msg::Float32>(
      "velocity_input", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<std_msgs::msg::Float32>("velocity", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_callback, this));
      
      //########################################################
      //For RC flag input
      rc_subscriber = this->create_subscription<deep_orange_msgs::msg::RcToCt>(
      "rc_flag", 10, std::bind(&MinimalSubscriber::rc_callback, this, _1));
      rc_publisher_ = this->create_publisher<deep_orange_msgs::msg::RcToCt>("raptor_dbw_interface/rc_to_ct", 10);
      rc_timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::rc_timer_callback, this));

      //#####################################################
      //For PT controller gains
      subscription_p_gain_acc = this->create_subscription<std_msgs::msg::Float32>(
      "p_gain_acc", 10, std::bind(&MinimalSubscriber::topic_callback_p_gain_acc, this, _1));
      
      subscription_i_gain_acc = this->create_subscription<std_msgs::msg::Float32>(
      "i_gain_acc", 10, std::bind(&MinimalSubscriber::topic_callback_i_gain_acc, this, _1));
      
      subscription_p_gain_eg = this->create_subscription<std_msgs::msg::Float32>(
      "p_gain_eg", 10, std::bind(&MinimalSubscriber::topic_callback_p_gain_eg, this, _1));
      
      subscription_acc_u_limit = this->create_subscription<std_msgs::msg::Float32>(
      "acc_u_limit", 10, std::bind(&MinimalSubscriber::topic_callback_acc_u_limit, this, _1));
      
      subscription_acc_l_limit = this->create_subscription<std_msgs::msg::Float32>(
      "acc_l_limit", 10, std::bind(&MinimalSubscriber::topic_callback_acc_l_limit, this, _1));
      
      subscription_v_demand_l_limit = this->create_subscription<std_msgs::msg::Float32>(
      "v_demand_l_limit", 10, std::bind(&MinimalSubscriber::topic_callback_v_demand_l_limit, this, _1));
      
      subscription_v_demand_u_limit = this->create_subscription<std_msgs::msg::Float32>(
      "v_demand_u_limit", 10, std::bind(&MinimalSubscriber::topic_callback_v_demand_u_limit, this, _1));
      
      subscription_velocity_demand_rate_u_limit = this->create_subscription<std_msgs::msg::Float32>(
      "velocity_demand_rate_u_limit", 10, std::bind(&MinimalSubscriber::topic_callback_velocity_demand_rate_u_limit, this, _1));
      
      subscription_velocity_demand_rate_l_limit = this->create_subscription<std_msgs::msg::Float32>(
      "velocity_demand_rate_l_limit", 10, std::bind(&MinimalSubscriber::topic_callback_velocity_demand_rate_l_limit, this, _1));
     
      
      publisher_p_gain_acc = this->create_publisher<std_msgs::msg::Float32>("p_gain_acc_ui", 10);
      publisher_i_gain_acc = this->create_publisher<std_msgs::msg::Float32>("i_gain_acc_ui", 10);
      publisher_p_gain_eg = this->create_publisher<std_msgs::msg::Float32>("p_gain_eg_ui", 10);
      publisher_acc_u_limit = this->create_publisher<std_msgs::msg::Float32>("acc_u_limit_ui", 10);
      publisher_acc_l_limit = this->create_publisher<std_msgs::msg::Float32>("acc_l_limit_ui", 10);
      publisher_v_demand_l_limit = this->create_publisher<std_msgs::msg::Float32>("v_demand_l_limit_ui", 10);
      publisher_v_demand_u_limit = this->create_publisher<std_msgs::msg::Float32>("v_demand_u_limit_ui", 10);
      publisher_velocity_demand_rate_u_limit = this->create_publisher<std_msgs::msg::Float32>("velocity_demand_rate_u_limit_ui", 10);
      publisher_velocity_demand_rate_l_limit = this->create_publisher<std_msgs::msg::Float32>("velocity_demand_rate_l_limit_ui", 10);
      
      
    }

  private:
    float v = 0.00;
    unsigned int rcflag = 1;
    float p_gain_acc = 0.75; 
    float i_gain_acc = 0.20;
    float p_gain_eg = 0.10;
    float acc_u_limit = 7.00;
    float acc_l_limit = -20.00;
    float v_demand_l_limit = 0.00;
    float v_demand_u_limit = 22.00;
    float velocity_demand_rate_u_limit = 1.00;
    float velocity_demand_rate_l_limit = -3.00;

    std::array<bool, 16> black = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    std::array<bool, 16> checkered = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    std::array<bool, 16> purple = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
    
    void topic_callback_p_gain_acc(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      p_gain_acc = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_p_gain_acc;

    void topic_callback_i_gain_acc(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      i_gain_acc = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_i_gain_acc;

    void topic_callback_p_gain_eg(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      p_gain_eg = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_p_gain_eg;

    void topic_callback_acc_u_limit(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      acc_u_limit = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_acc_u_limit;

    void topic_callback_acc_l_limit(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      acc_l_limit = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_acc_l_limit;

    void topic_callback_v_demand_l_limit(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      v_demand_l_limit = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_v_demand_l_limit;

    void topic_callback_v_demand_u_limit(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      v_demand_u_limit = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_v_demand_u_limit;

    void topic_callback_velocity_demand_rate_u_limit(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      velocity_demand_rate_u_limit = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_velocity_demand_rate_u_limit;

    void topic_callback_velocity_demand_rate_l_limit(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      velocity_demand_rate_l_limit = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_velocity_demand_rate_l_limit;

    void topic_callback(const std_msgs::msg::Float32::SharedPtr msg) 
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
      v = msg->data;
    }
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    void timer_callback()
    {
      auto velocity = std_msgs::msg::Float32();
      auto p_gain_acc_1 = std_msgs::msg::Float32();
      auto i_gain_acc_1 = std_msgs::msg::Float32();
      auto p_gain_eg_1 = std_msgs::msg::Float32();
      auto acc_u_limit_1 = std_msgs::msg::Float32();
      auto acc_l_limit_1 = std_msgs::msg::Float32();
      auto v_demand_l_limit_1 = std_msgs::msg::Float32();
      auto v_demand_u_limit_1 = std_msgs::msg::Float32();
      auto velocity_demand_rate_u_limit_1 = std_msgs::msg::Float32();
      auto velocity_demand_rate_l_limit_1 = std_msgs::msg::Float32();

      velocity.data = v;
      p_gain_acc_1.data = p_gain_acc;
      i_gain_acc_1.data = i_gain_acc;
      p_gain_eg_1.data = p_gain_eg;
      acc_u_limit_1.data = acc_u_limit;
      acc_l_limit_1.data = acc_l_limit;
      v_demand_l_limit_1.data = v_demand_l_limit;
      v_demand_u_limit_1.data = v_demand_u_limit;
      velocity_demand_rate_u_limit_1.data = velocity_demand_rate_u_limit;
      velocity_demand_rate_l_limit_1.data = velocity_demand_rate_l_limit;
      
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", v);
      publisher_->publish(velocity);
      publisher_p_gain_acc->publish(p_gain_acc_1);
      publisher_i_gain_acc->publish(i_gain_acc_1);
      publisher_p_gain_eg->publish(p_gain_eg_1);
      publisher_acc_u_limit->publish(acc_u_limit_1);
      publisher_acc_l_limit->publish(acc_l_limit_1);
      publisher_v_demand_l_limit->publish(v_demand_l_limit_1);
      publisher_v_demand_u_limit->publish(v_demand_u_limit_1);
      publisher_velocity_demand_rate_u_limit->publish(velocity_demand_rate_u_limit_1);
      publisher_velocity_demand_rate_l_limit->publish(velocity_demand_rate_l_limit_1);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    // rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_p_gain_acc;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_i_gain_acc;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_p_gain_eg;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_acc_u_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_acc_l_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_v_demand_l_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_v_demand_u_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velocity_demand_rate_u_limit;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_velocity_demand_rate_l_limit;

    //For RC flag
    void rc_callback(const deep_orange_msgs::msg::RcToCt::SharedPtr msg) 
    {
      RCLCPP_INFO(this->get_logger(), "Track Condition: '%u'", msg->track_cond);
      rcflag = msg->track_cond;
      black  = msg->black;
      checkered = msg->checkered;
      purple = msg->purple;
    }
    rclcpp::Subscription<deep_orange_msgs::msg::RcToCt>::SharedPtr rc_subscriber;
    void rc_timer_callback()
    {
      auto RCInfo = deep_orange_msgs::msg::RcToCt();
      RCInfo.track_cond = rcflag;
      RCInfo.black = black;
      RCInfo.checkered = checkered;
      RCInfo.purple = purple;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%u'", rcflag);
      // rc_publisher_->publish(RCInfo);
    }
    rclcpp::TimerBase::SharedPtr rc_timer_;
    rclcpp::Publisher<deep_orange_msgs::msg::RcToCt>::SharedPtr rc_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}