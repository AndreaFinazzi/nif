#pragma once

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <string>
#include <array>

#include <delphi_mrr_msgs/msg/detection.hpp>
#include <delphi_mrr_msgs/msg/system_status.hpp>
#include <delphi_mrr_msgs/msg/mrr_header_information_detections.hpp>
#include <delphi_mrr_msgs/msg/vehicle_state_msg2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "tkCommon/CanData.h"
#include "tkCommon/CanInterface.h"
#include "aptiv_mrr_driver/can_serialization.hpp"
#include "./visibility_control.hpp"

namespace aptiv_mrr_driver {

class DriverNode : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit DriverNode(const rclcpp::NodeOptions& options);
  ~DriverNode();

private:
  // Publishers
  rclcpp::Publisher<delphi_mrr_msgs::msg::Detection>::SharedPtr
  pub_detection_;

  rclcpp::Publisher<delphi_mrr_msgs::msg::MrrHeaderInformationDetections>::SharedPtr
  pub_header_information_detections_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
  pub_markers_;

  // Subscribers  
  rclcpp::Subscription<delphi_mrr_msgs::msg::VehicleStateMsg2>::SharedPtr
  sub_vehicle_state_;

  tk::communication::CanInterface can_write_interface_;

  void on_vehicle_state(const delphi_mrr_msgs::msg::VehicleStateMsg2::SharedPtr msg);
  
  // Can
  std::thread can_read_thread_;

  void can_read();
  void can_on_detected(const tk::data::CanData& data);
  void can_on_header_information_detections(const tk::data::CanData& data);
  void publish_markers(const delphi_mrr_msgs::msg::Detection& detection);

  void print_all_parameters() const;
};
    
} // namespace aptive_mrr_driver
