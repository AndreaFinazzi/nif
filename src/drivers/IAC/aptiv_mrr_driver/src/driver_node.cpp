#include <cmath>
#include <string>
#include <chrono>
#include <cstdlib>

#include "aptiv_mrr_driver/driver_node.hpp"
#include "tkCommon/CanInterface.h"
#include "aptiv_mrr_driver/can_serialization.hpp"
#include <geometry_msgs/msg/point.hpp>

#include <rclcpp/duration.hpp>

using delphi_mrr_msgs::msg::VehicleStateMsg2;
using namespace std::chrono_literals;

namespace {

std_msgs::msg::ColorRGBA str_to_ros_color(const std::string& color_str) {
  std_msgs::msg::ColorRGBA color;
  if (color_str == "red") {
    color.r = 1.0;
  } else if (color_str == "blue") {
    color.b = 1.0;
  } else {
    color.g = 1.0;
  }
  color.a = 1.0;
  return color;
}

}  // namespace

namespace aptiv_mrr_driver {

DriverNode::DriverNode(const rclcpp::NodeOptions& options) 
    : Node("aptiv_mrr_driver", options) {
  
  // ROS2 Params
  this->declare_parameter<std::string>("can_interface", "can0");
  this->declare_parameter<std::string>("frame_id", "left_radar");
  this->declare_parameter<std::string>("marker_color", "blue");
  print_all_parameters();

  std::string can_name = this->get_parameter("can_interface").as_string();
  if (!can_write_interface_.initSocket(can_name)) {
    throw std::runtime_error("error while init CanInterface");
  }

  // Quality of Service
  rclcpp::QoS qos(1);
  qos.reliable();

  // Publishers
  using namespace delphi_mrr_msgs::msg;
  pub_detection_ = this->create_publisher<Detection>("detection", qos);
  pub_header_information_detections_ = this->create_publisher<MrrHeaderInformationDetections>(
      "header_information_detections", qos);
  pub_markers_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "marker", qos);

  // Subscribers
  using std::placeholders::_1;
  sub_vehicle_state_ = this->create_subscription<VehicleStateMsg2>(
      "vehicle_state", qos,
      std::bind(&DriverNode::on_vehicle_state, this, _1));

  // Can reader thread
  can_read_thread_ = std::thread(&DriverNode::can_read, this);
}

DriverNode::~DriverNode() {
  can_read_thread_.join();
}

void DriverNode::on_vehicle_state(const VehicleStateMsg2::SharedPtr msg) {
  using namespace can;
  system_status::type system_status_msg = system_status::from_ros(msg);
  ifv_status_compensated::type ifv_status_msg = ifv_status_compensated::from_ros(msg);

  tk::data::CanData data;
  data.init();
  data.frame.can_dlc = system_status::length;
  data.frame.can_id = system_status::frame_id;
  int rv = system_status::pack(data.frame.data,
                               &system_status_msg,
                               system_status::length);
  if (rv < 0)
    throw std::runtime_error("error while packing system_status msg");
  if (!can_write_interface_.write(&data))
    throw std::runtime_error("error while writing to can");

  data.frame.can_dlc = ifv_status_compensated::length;
  data.frame.can_id = ifv_status_compensated::frame_id;
  rv = ifv_status_compensated::pack(data.frame.data,
                                    &ifv_status_msg,
                                    ifv_status_compensated::length);
  if (rv < 0)
    throw std::runtime_error("error while packing ifv_status_componsated msg");
  if (can_write_interface_.write(&data) == false)
    throw std::runtime_error("error while writing to can");
}

void DriverNode::can_read() {
  std::string can_interface_name;
  this->get_parameter("can_interface", can_interface_name);

  tk::communication::CanInterface can_interface;
  if (!can_interface.initSocket(can_interface_name)) {
    throw std::runtime_error("error while init CanInterface");
  }

  tk::data::CanData data;
  data.init();

  while (rclcpp::ok()) {
    if (!can_interface.read(&data))
      throw std::runtime_error("error while reading the can");
    
    if (data.id() >= can::detection::frame_id_first &&
        data.id() <= can::detection::frame_id_last) {
      can_on_detected(data);
    } else if (data.id() == can::header_info_detections::frame_id) {
      can_on_header_information_detections(data);
    }
  }

  can_interface.close();
}

void DriverNode::can_on_detected(const tk::data::CanData& data) {
  can::detection::type can_msg;
  can::detection::unpack(&can_msg, data.frame.data, data.frame.can_dlc);

  if (can_msg.can_det_range_01 == 0)
    return;
  
  using delphi_mrr_msgs::msg::Detection;
  Detection ros_msg = can::detection::to_ros(can_msg, data.header.stamp,
                                                data.frame.can_id);
  pub_detection_->publish(ros_msg);
  publish_markers(ros_msg);
}

void DriverNode::can_on_header_information_detections(
    const tk::data::CanData& data) {
  can::header_info_detections::type can_msg;
  can::header_info_detections::unpack(
      &can_msg, data.frame.data, data.frame.can_dlc);
  
  using delphi_mrr_msgs::msg::MrrHeaderInformationDetections;
  MrrHeaderInformationDetections ros_msg = can::header_info_detections::to_ros(
      can_msg, data.header.stamp);
  pub_header_information_detections_->publish(ros_msg);
}

void DriverNode::publish_markers(const delphi_mrr_msgs::msg::Detection& detection) {
  std::string frame_id, color_str;
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("marker_color", color_str);

  double range = can::detection::det_range_decode(detection.range);
  double x = range * std::cos(detection.azimuth);
  double y = range * std::sin(detection.azimuth);

  visualization_msgs::msg::Marker marker;
  marker.header.stamp = detection.header.stamp;
  marker.header.frame_id = frame_id;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.ns = frame_id;
  marker.id = detection.detection_id;
  marker.lifetime = rclcpp::Duration(1, 0);
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.color = str_to_ros_color(color_str);

  pub_markers_->publish(marker);
}

void DriverNode::print_all_parameters() const {
  std::string can_interface, frame_id, marker_color;
  this->get_parameter("can_interface", can_interface);
  this->get_parameter("frame_id", frame_id);
  this->get_parameter("marker_color", marker_color);
  RCLCPP_INFO_STREAM(this->get_logger(), "Can Interface: " << can_interface);
  RCLCPP_INFO_STREAM(this->get_logger(), "Frame ID:      " << frame_id);
  RCLCPP_INFO_STREAM(this->get_logger(), "Marker color:  " << marker_color);
}

}  // namespace aptiv_mrr_driver

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(aptiv_mrr_driver::DriverNode)
