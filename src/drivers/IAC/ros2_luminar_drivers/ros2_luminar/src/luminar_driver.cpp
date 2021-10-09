// Copyright 2020, Steve Macenski
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/qos.hpp"
#include "ros2_luminar/luminar_driver.hpp"
#include "ros2_luminar/exception.hpp"
#include "ros2_luminar/interfaces/lifecycle_interface.hpp"
#include "ros2_luminar/interfaces/sensor_interface.hpp"
#include "ros2_luminar/H3/processor_factories.hpp"


namespace ros2_luminar
{
#define PCLOUD_BYTES_PER_POINT   (16)

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std::chrono_literals;

LuminarDriver::LuminarDriver(
  std::unique_ptr<SensorInterface> sensor,
  const rclcpp::NodeOptions & options)
: LifecycleInterface("LuminarDriver", options), _sensor{std::move(sensor)}
{
  // declare params with a few default values
  this->declare_parameter("lidar_fingerprint");
  this->declare_parameter("data_source_is_pcap_app", false);
  this->declare_parameter("pcap_port", 5118);
  this->declare_parameter("scan_rate_hz");
  this->declare_parameter("scan_fov");
  this->declare_parameter("scan_pattern");
  this->declare_parameter("lidar_topic_name", std::string("_lidar_"));
  this->declare_parameter("lidar_sensor_pose");
  this->declare_parameter("coordinate_type");
  this->declare_parameter("lidar_frame_id", std::string("map_test"));
  this->declare_parameter("use_system_default_qos", false);
  this->declare_parameter("h3_proc_mask", std::string("PCL"));
  this->declare_parameter("rviz_friendly", false);
  this->declare_parameter("buffer_size", 12582912);

  _packet_count_ref = 0;
  _use_ros_time = true;   // use system time in ROS2 headers

}

LuminarDriver::~LuminarDriver() = default;

// from lifecycle_interface, this is the callback for on_configure
void LuminarDriver::onConfigure()
{
  uint16_t my_lidar_fingerprint = 0;
  try {
    my_lidar_fingerprint = (uint16_t)get_parameter("lidar_fingerprint").as_int();
  } catch (...) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to get lidar ID (fingerprint) from parameters (required)");
    exit(-1);
  }

  // set which data processors are active(PCL only), other settings
  _h3_proc_mask = ros2_luminar::toProcMask(get_parameter("h3_proc_mask").as_string());
  _lidar_frame_id = get_parameter("lidar_frame_id").as_string();
  _use_system_default_qos = get_parameter("use_system_default_qos").as_bool();
  _sensor->setRVizFriendly(get_parameter("rviz_friendly").as_bool());
  _sensor->setAdditionalData((_h3_proc_mask & ros2_luminar::H3_PROC_PCL_EXTENDED) == ros2_luminar::H3_PROC_PCL_EXTENDED);
  bool sourceIsPcap = get_parameter("data_source_is_pcap_app").as_bool();
  if(sourceIsPcap) {
    RCLCPP_INFO(
      this->get_logger(), "USING PCAP PLAYER AS LIDAR DATA SOURCE, ID: %u -------",
      my_lidar_fingerprint
    );
  }
  _sensor->setDataSourceIsPcapApp(sourceIsPcap);

  // callback for parameter setting
  auto param_change_callback = [this, sourceIsPcap](std::vector<rclcpp::Parameter> parameters)
  {
    auto result = rcl_interfaces::msg::SetParametersResult();
    result.successful = true;
    for (auto parameter : parameters) {
      rclcpp::ParameterType parameter_type = parameter.get_type();
      if (rclcpp::ParameterType::PARAMETER_NOT_SET == parameter_type) {
        RCLCPP_INFO(
          this->get_logger(), "parameter '%s' deleted successfully",
          parameter.get_name().c_str()
        );
        result.successful &= true;
      }

      // pointcloud data coordinate type: Cartesian or Spherical
      else if (parameter.get_name() == "coordinate_type" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
      {
        if(parameter.as_string() == "Cartesian") {
          _sensor->setPtcFormat(1);
          result.successful &= true;
        }
        else if(parameter.as_string() == "Spherical") {
          _sensor->setPtcFormat(0);
          result.successful &= true;
        }
        else {
          result.successful = false;
        }
      }
      // Sensor pose (NOTE: setting this to nonzero results in higher latency)
      else if (parameter.get_name() == "lidar_sensor_pose" &&
        parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
      {
        _sensor->setSensorPose(
          parameter.as_double_array().at(0), 
          parameter.as_double_array().at(1), 
          parameter.as_double_array().at(2), 
          parameter.as_double_array().at(3), 
          parameter.as_double_array().at(4), 
          parameter.as_double_array().at(5)); 
        result.successful &= true;
      }

      // if PCapPlayer is used, only print the parm info.
      if(sourceIsPcap) {
        RCLCPP_INFO(
          this->get_logger(), "Received setting for parameter '%s'", 
          parameter.get_name().c_str());
        result.successful &= true;
      }
      else {
        // Scan Pattern: Gaussian, Exponential, Horizon, Trapezoidal, or Uniform
        if (parameter.get_name() == "scan_pattern" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY)
        {
          if(parameter.as_string_array().at(0) == "Gaussian")
          {
              // 3 parms: mean offset from center, sigma (spread), max velocity
            _sensor->setScanPattern( 1, 
              std::stof(parameter.as_string_array().at(1)),
              std::stof(parameter.as_string_array().at(2)),
              std::stof(parameter.as_string_array().at(3)),
              0.0, 0.0);
            result.successful &= true;
          }
          else if (parameter.as_string_array().at(0) == "Uniform")
          {
            // no parms
            _sensor->setScanPattern(0, 0.0, 0.0, 0.0, 0.0, 0.0);
            result.successful &= true;
          }
          else if (parameter.as_string_array().at(0) == "Horizon")
          {
            // 3 parms: location offset from center, width, density
            _sensor->setScanPattern( 2, 
              std::stof(parameter.as_string_array().at(1)),
              std::stof(parameter.as_string_array().at(2)),
              std::stof(parameter.as_string_array().at(3)),
              0.0, 0.0);
  
            result.successful &= true;
          }
          else if (parameter.as_string_array().at(0) == "Trapezoidal")
          {
            // 5 parms: base min, max offset from center, top min, max offset from center, density
            _sensor->setScanPattern( 3, 
              std::stof(parameter.as_string_array().at(1)),
              std::stof(parameter.as_string_array().at(2)),
              std::stof(parameter.as_string_array().at(3)),
              std::stof(parameter.as_string_array().at(4)),
              std::stof(parameter.as_string_array().at(5)));
            result.successful &= true;
          }
          else if (parameter.as_string_array().at(0) == "Exponential")
          {
            // 4 parms: mean offset from center, sigma (spread), max velocity, exponent
            _sensor->setScanPattern( 4, 
              std::stof(parameter.as_string_array().at(1)),
              std::stof(parameter.as_string_array().at(2)),
              std::stof(parameter.as_string_array().at(3)),
              std::stof(parameter.as_string_array().at(4)),
              0.0);
            result.successful &= true;
          }
          else {
            result.successful = false;
          }
        }
  
        // Sensor scan vertical field-of-view by range(0 to 30) and center (-15 to 15) 
        else if (parameter.get_name() == "scan_fov" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY)
        {
          _sensor->setScanFOV(
            parameter.as_double_array().at(0), 
            parameter.as_double_array().at(1)); 
          result.successful &= true;
        }
  
        // Sensor scan rate, in Hz (1.0 to 30.0)
        else if (parameter.get_name() == "scan_rate_hz" &&
          parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          if(_sensor->setScanFrequency(parameter.as_double())) {
            result.successful &= true;
          }
          else {
            result.successful = false;
          }
        }
  
        // unsupported parms
        else {
          RCLCPP_INFO(
            this->get_logger(), "trouble with parameter '%s' of type '%s'",
            parameter.get_name().c_str(), parameter.get_type_name().c_str()
          );
          result.reason = "parameter not supported in driver";
          result.successful = false;
        }
      }
    }
    return result;
  };
  callback_handler = this->add_on_set_parameters_callback(param_change_callback);

  _reset_srv = this->create_service<std_srvs::srv::Empty>(
    "~/reset", std::bind(&LuminarDriver::resetService, this, _1, _2, _3));

  try {
    uint16_t my_pcap_port=(uint16_t)get_parameter("pcap_port").as_int();
    _sensor->configure(my_lidar_fingerprint,my_pcap_port);
  } catch (const LuminarDriverException & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception thrown: (%s)", e.what());
    exit(-1);
  }

  // parms loaded from yaml file --> send to sensor via callback trigger
  std::vector<std::string> param_names = {
    "scan_pattern", "scan_fov", "scan_rate_hz", "coordinate_type", "lidar_sensor_pose"
  };
  std::vector<rclcpp::Parameter> myParms = get_parameters(param_names);
  // trigger the callback (param_change_callback())
  set_parameters(myParms);


  // [neil-rti] use a pre-allocated buffer to get the lidar data
  // NOTE: at a 1Hz update rate, the unit can produce ~702K points, needing 11.2M bytes.
  // Round it up to 12MB.
  const size_t buffer_size = static_cast<size_t>(get_parameter("buffer_size").as_int());
  _ptcDataOut = (uint8_t *)malloc(buffer_size);
  _sensor->setPtcDataPtr(_ptcDataOut);

  createProcessors();
}

void LuminarDriver::createProcessors() {
  auto system_qos = rclcpp::SystemDefaultsQoS();
  auto sensor_qos = rclcpp::SensorDataQoS();
  rclcpp::QoS & proc_qos = sensor_qos;
  if (_use_system_default_qos) {
    proc_qos = system_qos;
  }
  _data_processors = ros2_luminar::createProcessors(
    shared_from_this(), _lidar_frame_id, proc_qos, _h3_proc_mask);
}

void LuminarDriver::onActivate()
{
  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onActivate();
  }

  // this sets the polling rate to check for new lidar data arrival
  // Sensor works at up to 30Hz (0.033 seconds), polling rate should be 
  // (significantly) less than this; it will directly affect latency
  _process_timer = this->create_wall_timer(
    333333ns, std::bind(&LuminarDriver::processData, this));
}

void LuminarDriver::onError()
{ }

void LuminarDriver::onDeactivate()
{
  _process_timer->cancel();
  _process_timer.reset();
  //FIXME: do we need to set mHSubscriptionPaused ?

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    it->second->onDeactivate();
  }
}

void LuminarDriver::onCleanup()
{
  _data_processors.clear();
  _reset_srv.reset();
}

void LuminarDriver::onShutdown()
{
  _process_timer->cancel();
  _process_timer.reset();

  DataProcessorMapIt it;
  for (it = _data_processors.begin(); it != _data_processors.end(); ++it) {
    delete it->second;
  }
  _data_processors.clear();
  free(_ptcDataOut);
}


// [neil-rti] timer callback to poll for received data
void LuminarDriver::processData()
{  
  int32_t packet_count_now = _sensor->getPacketCounter();
  if(_packet_count_ref != packet_count_now) {
    _packet_count_ref = packet_count_now;
    int32_t point_count_now = _sensor->getCountOfPointsInPacket();
    if(point_count_now) {
      try {
        ClientState state = ros2_luminar::ClientState::LIDAR_DATA;
        uint8_t * packet_data = _ptcDataOut;

        if (packet_data) {
          std::pair<DataProcessorMapIt, DataProcessorMapIt> key_its;
          key_its = _data_processors.equal_range(state);
          uint64_t override_ts = this->_use_ros_time ? this->now().nanoseconds() : 0;

          for (DataProcessorMapIt it = key_its.first; it != key_its.second; it++) {
            it->second->process(packet_data, point_count_now, override_ts);
          }
        }
      } catch (const LuminarDriverException & e) {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed to process packet with exception %s.", e.what());
      }
    }
  }
}

// sets up a ROS2 service to reconfigure; untested.
void LuminarDriver::resetService(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  if (!this->isActive()) { return; }
  _sensor->reset((uint16_t)get_parameter("lidar_fingerprint").as_int());
}


}  // namespace ros2_luminar

