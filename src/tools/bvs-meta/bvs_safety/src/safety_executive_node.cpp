#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <bvs_msgs/msg/safety_status.hpp>
#include <deep_orange_msgs/msg/ct_report.hpp>
#include <deep_orange_msgs/msg/joystick_command.hpp>
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/insstdev.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class SafetyExecutiveNode : public rclcpp::Node {
  public:
    SafetyExecutiveNode()
    : Node("bvs_safety_executive_node"), count_(0) {
        // setup QOS to be best effort
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(RMW_QOS_POLICY_HISTORY_KEEP_LAST, 1));
        qos.best_effort();

        publisher_joy_emergency = this->create_publisher<std_msgs::msg::Bool>("/bvs_safety/emergency_joystick", 10);
        publisher_hb_emergency = this->create_publisher<std_msgs::msg::Bool>("/bvs_safety/emergency_heartbeat", 10);
        diagnostic_hb_publisher = this->create_publisher<std_msgs::msg::Int32>("/bvs_safety/heartbeat", 10);
        safety_status_publisher = this->create_publisher<bvs_msgs::msg::SafetyStatus>("/bvs_safety/safety_status", 10);
        safety_status_telem_publisher = this->create_publisher<bvs_msgs::msg::SafetyStatus>("/telemetry/safety_status", qos);

        safety_msg = bvs_msgs::msg::SafetyStatus();
        safety_msg.gps_healthy = false;
        safety_msg.comms_healthy = false;
        safety_msg.joy_emergency = false;

        timer_ = this->create_wall_timer(
            50ms, std::bind(&SafetyExecutiveNode::timer_callback, this));

        telemetry_timer = this->create_wall_timer(
            250ms, std::bind(&SafetyExecutiveNode::telemetry_timer_callback, this));

        // subcribers
        subscriber_joystick = this->create_subscription<deep_orange_msgs::msg::JoystickCommand>(
            "/joystick/command", qos, std::bind(&SafetyExecutiveNode::counter_callback, this, _1));
        subscriber_ct_status = this->create_subscription<deep_orange_msgs::msg::CtReport>(
            "/raptor_dbw_interface/ct_report", 10, std::bind(&SafetyExecutiveNode::ct_status_callback, this, _1));
        subscriber_bestpos = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
            "/novatel_top/bestpos", 1, std::bind(&SafetyExecutiveNode::receive_bestpos, this, std::placeholders::_1));
        subscriber_insstdev = this->create_subscription<novatel_oem7_msgs::msg::INSSTDEV>(
            "/novatel_top/insstdev", 1, std::bind(&SafetyExecutiveNode::receive_insstdev, this, std::placeholders::_1));
        subscriber_planning = this->create_subscription<nav_msgs::msg::Path>(
                "/planning/path_global", rclcpp::SensorDataQoS(), std::bind(&SafetyExecutiveNode::receive_path, this, std::placeholders::_1));

        // Services
        recovery_service = this->create_service<std_srvs::srv::Trigger>(
            "/bvs_safety/recover",
            std::bind(
                &SafetyExecutiveNode::recovery_service_handler,
                this,
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    };
  private:

    void update_debug_flags() {
        safety_msg.stamp = rclcpp::Clock().now();
        safety_msg.lat_stdev = lat_stdev_;
        safety_msg.long_stdev = long_stdev_;
        safety_msg.best_pos_lat_stdev = best_pos_lat_stdev_;
        safety_msg.best_pos_long_stdev = best_pos_long_stdev_;
        safety_msg.time_since_last_update = time_since_last_update_;
    }

    bool heartbeat_ok() {
        // check for timeouts
        if (prev_counter != counter) {
            // received new message, heartbeat ok
            t--;
            if (t < 0) {
                t = 0;
            }
            if (counter != default_counter) {
                prev_counter = counter;
            }
            return true;
        } else {
            // have not received update; check for timeout
            t++;
            if (t >= max_counter_drop) {
                this->recovery_enabled = false;
                return false;
            } else {
                // timeout not reached yet
                return true;
            }
        }
    }

    bool gps_health_ok() {
        double pose_stdev_thres = 2.0;
        bool std_dev_trigger = (this->lat_stdev_ >  pose_stdev_thres ||
                                this->long_stdev_ > pose_stdev_thres);
        bool time_since_trigger = (this->time_since_last_update_ > 0);
        return (!std_dev_trigger && !time_since_trigger);
    }

    void telemetry_timer_callback() {
        // send telemetry message to base station at 4hz
        safety_status_telem_publisher->publish(safety_msg);
    }

    void timer_callback() {
        auto message_joy = std_msgs::msg::Bool();
        auto message_hb = std_msgs::msg::Bool();
        message_joy.data = false;
        message_hb.data = false;

        // check safety confitions
        bool hb_ok = heartbeat_ok();

        if (!hb_ok || !this->recovery_enabled) {
            hb_ok = false;
        }

        bool gps_ok = gps_health_ok() && this->path_healthy;

        // if operator commands emergency stop
        if (joy_emergency_stop){
            message_joy.data = true;
        }

        // if lost hearbeat over comms
        if (!hb_ok){
            message_hb.data = true;
        }

        // update safety status message
        safety_msg.gps_healthy = gps_ok;
        safety_msg.comms_healthy = hb_ok;
        safety_msg.path_healthy = this->path_healthy;
        safety_msg.joy_emergency = joy_emergency_stop;
        update_debug_flags();

        safety_status_publisher->publish(safety_msg);
        publisher_joy_emergency->publish(message_joy);
        publisher_hb_emergency->publish(message_hb);

        // send diagnostic hb to vehicle interface
        auto joy_hb = std_msgs::msg::Int32();
        joy_hb.data = counter_hb;
        diagnostic_hb_publisher->publish(joy_hb);
        counter_hb++;
        if (counter_hb == 8){
            counter_hb = 0;
        }
    }

    void counter_callback(const deep_orange_msgs::msg::JoystickCommand::SharedPtr msg) {
        // parse counter
        counter = msg->counter;
        // parse emergency
        if (msg->emergency_stop == 1) {
            joy_emergency_stop = true;
        }

    }

    void ct_status_callback(const deep_orange_msgs::msg::CtReport::SharedPtr msg) {
        mode = msg->ct_state;
    }

    void receive_bestpos(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg) {
        this->best_pos_lat_stdev_ = msg->lat_stdev;
        this->best_pos_long_stdev_ = msg->lon_stdev;
    }

    void receive_insstdev(
        const novatel_oem7_msgs::msg::INSSTDEV::SharedPtr msg) {
        this->lat_stdev_ = msg->latitude_stdev;
        this->long_stdev_ = msg->longitude_stdev;
        this->time_since_last_update_ = msg->time_since_last_update;
    }

    void receive_path(
            const nav_msgs::msg::Path::SharedPtr msg) {
        this->path_last_update_ = this->now();
        if (!(msg && msg->poses.size() > this->path_min_size_))
        {
            this->path_healthy = false;
        } else {
            this->path_healthy = true;
        }
    }

    /**
    * Recovers from comms failure.
    * It only allows recovery from error status to ok status, not the other way around (pull-up only).
    */
    void recovery_service_handler(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
            if (!this->recovery_enabled) {

                this->recovery_enabled = true;
                response->success = this->recovery_enabled;
                response->message = "recovery_enabled set to ";
                response->message.append( this->recovery_enabled ? "true" : "false" );
            } else {
                response->success = true;
                response->message = "WARN: service call has been ignored, recovery_enable is already true. (pull-up only)";
            }

    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr telemetry_timer;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_joy_emergency;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_hb_emergency;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr diagnostic_hb_publisher;
    rclcpp::Publisher<bvs_msgs::msg::SafetyStatus>::SharedPtr safety_status_publisher;
    rclcpp::Publisher<bvs_msgs::msg::SafetyStatus>::SharedPtr safety_status_telem_publisher;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recovery_service;


    size_t count_;
    unsigned int mode = 255;
    const int max_counter_drop = 20;
    int t = 0;
    int counter_hb = 0;

    const int default_counter = 502;
    int prev_counter = default_counter-1;
    int counter = default_counter;
    bool joy_emergency_stop = false;
    bool recovery_enabled = true;

    bvs_msgs::msg::SafetyStatus safety_msg;

    unsigned int time_since_last_update_ = 0;
    double lat_stdev_ = 0.0;
    double long_stdev_ = 0.0;
    double best_pos_lat_stdev_ = 0.0;
    double best_pos_long_stdev_ = 0.0;
    bool path_healthy = false;

    rclcpp::Subscription<deep_orange_msgs::msg::JoystickCommand>::SharedPtr subscriber_joystick;
    rclcpp::Subscription<deep_orange_msgs::msg::CtReport>::SharedPtr subscriber_ct_status;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subscriber_bestpos;
    rclcpp::Subscription<novatel_oem7_msgs::msg::INSSTDEV>::SharedPtr subscriber_insstdev;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber_planning;
    rclcpp::Time path_last_update_;
    unsigned int path_min_size_ = 20;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyExecutiveNode>());
  rclcpp::shutdown();
  return 0;
}
