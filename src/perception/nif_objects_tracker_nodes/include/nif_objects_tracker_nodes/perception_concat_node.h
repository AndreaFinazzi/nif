//
// Created by usrg on 9/9/21.
//

#ifndef APTIVE_ESR_INTERFACE_H
#define APTIVE_ESR_INTERFACE_H

#include "rclcpp/rclcpp.hpp"

#include "nif_common/types.h"
#include <chrono>
#include <mutex>

using nif::common::constants::DEG2RAD;
using namespace std::chrono_literals; 

namespace nif {
namespace perception {

auto TIMER_PERIOD = 25ms;
unsigned int PERCEPTION_CONTAINER_DIM = 10;

std::mutex __MUTEX__;

class PerceptionConcatNode : public rclcpp::Node {
public:
  PerceptionConcatNode(
      const std::string & node_name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions().use_intra_process_comms(true))
      : Node(node_name, options) {

    this->perception_list_pub = this->create_publisher<nif::common::msgs::PerceptionResultList>(
                    "concat_out_perception_list", nif::common::constants::QOS_SENSOR_DATA);

    this->radar_perception_sub = this->create_subscription<nif::common::msgs::PerceptionResultList>(
                    "concat_in_radar_list", 
                    nif::common::constants::QOS_SENSOR_DATA,
                    std::bind(&PerceptionConcatNode::radarCallback, this, std::placeholders::_1));

    this->lidar_perception_sub = this->create_subscription<nif::common::msgs::PerceptionResultList>(
                    "concat_in_lidar_list", 
                    nif::common::constants::QOS_SENSOR_DATA,
                    std::bind(&PerceptionConcatNode::lidarCallback, this, std::placeholders::_1));

    this->dataInit();

    this->timer = this->create_wall_timer(TIMER_PERIOD, std::bind(&PerceptionConcatNode::timer_callback, this));
  }

private:

  nif::common::msgs::PerceptionResultList::UniquePtr perception_list_msg_ptr;

  rclcpp::Subscription<nif::common::msgs::PerceptionResultList>::SharedPtr radar_perception_sub;
  rclcpp::Subscription<nif::common::msgs::PerceptionResultList>::SharedPtr lidar_perception_sub;
  rclcpp::Publisher<nif::common::msgs::PerceptionResultList>::SharedPtr perception_list_pub;

  rclcpp::TimerBase::SharedPtr timer;

    unsigned int next_index = 0;


    // Publish a converted message
void timer_callback() {
        // Publish the two detection arrays properly resized.
        __MUTEX__.lock();

        this->perception_list_msg_ptr->perception_list.resize(next_index);
        
        if (!this->perception_list_msg_ptr->perception_list.empty())
            this->perception_list_pub->publish(std::move(this->perception_list_msg_ptr));

        this->dataInit();

        __MUTEX__.unlock();
}

void radarCallback(
          const nif::common::msgs::PerceptionResultList::SharedPtr msg) 
{

    __MUTEX__.lock(); // Prevent referencing nullptr (btw move and make_unique)
        for (auto& obj : msg->perception_list) {

            unsigned int i = next_index++;
            
            if (i >= this->perception_list_msg_ptr->perception_list.size()) {
                this->perception_list_msg_ptr->perception_list.resize(
                    this->perception_list_msg_ptr->perception_list.size() + i
                );
            }

            this->perception_list_msg_ptr->perception_list[i] = std::move(obj);
        }

    __MUTEX__.unlock();
}

void lidarCallback(
          const nif::common::msgs::PerceptionResultList::SharedPtr msg) 
{
    __MUTEX__.lock(); // Prevent referencing nullptr (btw move and make_unique)
    // TODO individually stamp perception results in perception_list
    for (auto& obj : msg->perception_list) {

        unsigned int i = next_index++;
        
        if (i >= this->perception_list_msg_ptr->perception_list.size()) {
            this->perception_list_msg_ptr->perception_list.resize(
                this->perception_list_msg_ptr->perception_list.size() + i
            );
        }

        this->perception_list_msg_ptr->perception_list[i] = std::move(obj);
    }
    __MUTEX__.unlock();
}

void dataInit() {
    this->next_index = 0; // reset counter
    
    this->perception_list_msg_ptr = std::make_unique<nif::common::msgs::PerceptionResultList>();

    // Count to store less than PERCEPTION_CONTAINER_DIM perception points (not constraining)
    this->perception_list_msg_ptr->perception_list.resize(PERCEPTION_CONTAINER_DIM);
} 

};
} // namespace perception
} // namespace nif
#endif // APTIVE_ESR_INTERFACE_H