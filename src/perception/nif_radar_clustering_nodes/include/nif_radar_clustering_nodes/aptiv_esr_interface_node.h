//
// Created by usrg on 9/9/21.
//

#ifndef APTIVE_ESR_INTERFACE_H
#define APTIVE_ESR_INTERFACE_H

#include "rclcpp/rclcpp.hpp"

#include "delphi_esr_msgs/msg/esr_track.hpp"
#include "nif_common/types.h"
#include <chrono>
#include <mutex>

using nif::common::constants::DEG2RAD;
using namespace std::chrono_literals; 

namespace nif {
namespace control {

auto TIMER_PERIOD = 40ms;
std::mutex mutex;

class AptiveESRInterface : public rclcpp::Node {
public:
  AptiveESRInterface(
      const std::string & node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{})
      : Node(node_name, options) {

    this->radar_msgs_tracks_pub = this->create_publisher<nif::common::msgs::RadarTrackList>(
        "out_track_list", nif::common::constants::QOS_SENSOR_DATA);
    
    this->perception_list_pub = this->create_publisher<nif::common::msgs::PerceptionResultList>(
        "out_perception_list", nif::common::constants::QOS_SENSOR_DATA);
    
    std::weak_ptr<std::remove_pointer<decltype(this->radar_msgs_tracks_pub.get())>::type> 
        radar_captured_pub = this->radar_msgs_tracks_pub;
    std::weak_ptr<std::remove_pointer<decltype(this->perception_list_pub.get())>::type>
        perception_captured_pub = this->perception_list_pub;
    

    this->override_msg_sub = this->create_subscription<nif::common::msgs::RadarTrackList>(
                    "in_radar_track", 
                    nif::common::constants::QOS_SENSOR_DATA,
                    std::bind(&AptiveESRInterface::esrTrackCallback, this, std::placeholders::_1));


    this->dataInit();

    // Publish a converted message
    auto callback = [this, radar_captured_pub, perception_captured_pub]() -> void {
        auto radar_pub_ptr = radar_captured_pub.lock();
        auto perception_pub_ptr = perception_captured_pub.lock();
        if (!radar_pub_ptr || perception_pub_ptr) {
            return;
        }
        // Publish the two detection arrays properly resized.
        mutex.lock();

        radar_pub_ptr->publish(std::move(this->radar_track_list_msg_ptr));
        perception_pub_ptr->publish(std::move(this->perception_list_msg_ptr));

        this->dataInit();

        mutex.unlock();
    };
    this->timer = this->create_wall_timer(TIMER_PERIOD, callback);
  }

private:
  nif::common::msgs::RadarTrackList::UniquePtr radar_track_list_msg_ptr;
  nif::common::msgs::PerceptionResultList::UniquePtr perception_list_msg_ptr;

  rclcpp::Subscription<delphi_esr_msgs::msg::EsrTrack>::SharedPtr override_msg_sub;
  rclcpp::Publisher<nif::common::msgs::RadarTrackList>::SharedPtr radar_msgs_tracks_pub;
  rclcpp::Publisher<nif::common::msgs::PerceptionResultList>::SharedPtr perception_list_pub;

  rclcpp::TimerBase::SharedPtr timer;

    unsigned int next_index = 0;

void esrTrackCallback(
          const delphi_esr_msgs::msg::EsrTrack::SharedPtr msg) 
{
    unsigned int i = next_index++;

    auto theta_rad = DEG2RAD * msg->track_angle;
    auto& r = msg->track_range;
    auto& r_dot = msg->track_range_rate;

    mutex.lock(); // Prevent referencing nullptr (btw move and make_unique)

    if (i >= this->radar_track_list_msg_ptr->detections.size()) {
        this->radar_track_list_msg_ptr->detections.resize(
            this->radar_track_list_msg_ptr->detections.size() + i
        );
    }

    if (i >= this->radar_track_list_msg_ptr->detections.size()) {
        this->radar_track_list_msg_ptr->detections.resize(
            this->radar_track_list_msg_ptr->detections.size() + i
        );
    }

    this->radar_track_list_msg_ptr->detections[i].detection_id = msg->track_id; 

    this->radar_track_list_msg_ptr->detections[i].position.x = 
        r * cos(theta_rad);
    this->radar_track_list_msg_ptr->detections[i].position.y = 
        r * sin(theta_rad);

    // equation for velocity are here, but esrTrack does not have angle_range information (?).
    // auto& theta_dot = msg->track_angle_rate;

    // this->radar_track_list_msg_ptr->detections[i].velocity.x = r_dot * cos(theta_rad) - r * theta_dot * sin(theta_rad); 
    // this->radar_track_list_msg_ptr->detections[i].velocity.y = r_dot * sin(theta_rad) + r * theta_dot * cos(theta_rad); 

    this->perception_list_msg_ptr->perception_list[i].id = msg->track_id;
    this->perception_list_msg_ptr->perception_list[i].detection_result_3d.center.position.x = 
        r * cos(theta_rad);
    this->perception_list_msg_ptr->perception_list[i].detection_result_3d.center.position.y = 
        r * sin(theta_rad);

    mutex.unlock();
}

void dataInit() {
    
    this->radar_track_list_msg_ptr = std::make_unique<nif::common::msgs::RadarTrackList>();
    this->perception_list_msg_ptr = std::make_unique<nif::common::msgs::PerceptionResultList>();

    // Count to store at most 100 tracks 
    this->radar_track_list_msg_ptr->detections.resize(100);
    this->perception_list_msg_ptr->perception_list.resize(100);
} 

};
} // namespace control
} // namespace nif
#endif // APTIVE_ESR_INTERFACE_H
