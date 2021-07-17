#include <chrono>
#include <memory>

#include "novatel_gps_msgs/msg/inspva.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

using namespace std::chrono_literals;

class SyncerNode : public rclcpp::Node {
public:
  SyncerNode() : Node("syncer") {
    rclcpp::QoS qos(10);
    auto rmw_qos_profile = qos.get_rmw_qos_profile();

    // publisher_temp1_ =
    //     this->create_publisher<sensor_msgs::msg::Temperature>("temp_1", qos);
    // publisher_temp2_ =
    //     this->create_publisher<sensor_msgs::msg::Temperature>("temp_2", qos);

    // timer_ = this->create_wall_timer(
    //     500ms, std::bind(&SyncerNode::TimerCallback, this));

    subscriber_temp1_.subscribe(
        this, "/novatel_bottom/inspva", rmw_qos_profile);
    subscriber_temp2_.subscribe(this, "/novatel_top/inspva", rmw_qos_profile);

    // // Uncomment this to verify that the messages indeed reach the
    // subscriber_temp1_.registerCallback(
    //     std::bind(&SyncerNode::Tmp1Callback, this, std::placeholders::_1));
    // subscriber_temp2_.registerCallback(
    //     std::bind(&SyncerNode::Tmp2Callback, this, std::placeholders::_1));

    temp_sync_ = std::make_shared<
        message_filters::TimeSynchronizer<novatel_gps_msgs::msg::Inspva,
                                          novatel_gps_msgs::msg::Inspva>>(
        subscriber_temp1_, subscriber_temp2_, 10);
    temp_sync_->registerCallback(std::bind(&SyncerNode::TempSyncCallback,
                                           this,
                                           std::placeholders::_1,
                                           std::placeholders::_2));
  }

private:
  void TimerCallback() {
    rclcpp::Time now = this->get_clock()->now();

    // auto msg_tmp1 = novatel_gps_msgs::msg::Temperature();
    // msg_tmp1.header.stamp = now;
    // msg_tmp1.header.frame_id = "test";
    // // msg_tmp1.temperature = 1.0;

    // auto msg_tmp2 = novatel_gps_msgs::msg::Temperature();
    // msg_tmp2.header.stamp = now;
    // msg_tmp2.header.frame_id = "test";
    // // msg_tmp2.temperature = 2.0;

    // publisher_temp1_->publish(msg_tmp1);
    // publisher_temp2_->publish(msg_tmp2);

    // RCLCPP_INFO(this->get_logger(), "Published two temperatures.");
  }

  // For veryfing the single subscriber instances: Uncomment line 26-28.
  void Tmp1Callback(const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& msg) {
    // RCLCPP_INFO(this->get_logger(),
    //             "Frame '%s', temp %f with ts %u.%u sec ",
    //             msg->header.frame_id.c_str(),
    //             msg->temperature,
    //             msg->header.stamp.sec,
    //             msg->header.stamp.nanosec);
  }

  // For veryfing the single subscriber instances: Uncomment line 29-31.
  void Tmp2Callback(const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& msg) {
    // RCLCPP_INFO(this->get_logger(),
    //             "Frame '%s', temp %f with ts %u.%u sec ",
    //             msg->header.frame_id.c_str(),
    //             msg->temperature,
    //             msg->header.stamp.sec,
    //             msg->header.stamp.nanosec);
  }

  // This callback is never being called.
  void
  TempSyncCallback(const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& msg_1,
                   const novatel_gps_msgs::msg::Inspva::ConstSharedPtr& msg_2) {
    RCLCPP_INFO(this->get_logger(),
                "I heard and synchronized the following timestamps: %u, %u",
                msg_1->header.stamp.sec,
                msg_2->header.stamp.sec);
  }

  rclcpp::Publisher<novatel_gps_msgs::msg::Inspva>::SharedPtr publisher_temp1_;
  rclcpp::Publisher<novatel_gps_msgs::msg::Inspva>::SharedPtr publisher_temp2_;
  message_filters::Subscriber<novatel_gps_msgs::msg::Inspva> subscriber_temp1_;
  message_filters::Subscriber<novatel_gps_msgs::msg::Inspva> subscriber_temp2_;
  std::shared_ptr<
      message_filters::TimeSynchronizer<novatel_gps_msgs::msg::Inspva,
                                        novatel_gps_msgs::msg::Inspva>>
      temp_sync_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncerNode>());
  rclcpp::shutdown();
  return 0;
}