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

#ifndef ROS2_LUMINAR__LUMINAR_DRIVER_HPP_
#define ROS2_LUMINAR__LUMINAR_DRIVER_HPP_

#include <memory>
#include <map>
#include <string>

#include "ros2_luminar/interfaces/lifecycle_interface.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"
#include "ros2_luminar/interfaces/data_processor_interface.hpp"


namespace ros2_luminar
{

class SensorInterface;

/**
 * @class ros2_luminar::LuminarDriver
 * @brief A lifecycle interface implementation of a Luminar H3 Lidar
 * driver in ROS2.
 */
class LuminarDriver : public lifecycle_interface::LifecycleInterface
{
public:
  using DataProcessorMap = std::multimap<ClientState, DataProcessorInterface *>;
  using DataProcessorMapIt = DataProcessorMap::iterator;

  /**
   * @brief A constructor for ros2_luminar::LuminarDriver
   * @param options Node options for lifecycle node interfaces
   */
  LuminarDriver(
    std::unique_ptr<SensorInterface> sensor,
    const rclcpp::NodeOptions & options);

  /**
   * @brief A destructor for ros2_luminar::LuminarDriver
   */
  ~LuminarDriver();

  /**
   * @brief lifecycle node's implementation of configure step
   * which will configure ROS interfaces and allocate resources
   */
  void onConfigure() override;

  /**
   * @brief lifecycle node's implementation of activate step
   * which will activate ROS interfaces and start processing information
   */
  void onActivate() override;

  /**
   * @brief lifecycle node's implementation of deactivate step
   * which will deactivate ROS interfaces and stop processing information
   */
  void onDeactivate() override;

  /**
   * @brief lifecycle node's implementation of error step
   * which will handle errors in the lifecycle node system
   */
  void onError() override;

  /**
   * @brief lifecycle node's implementation of shutdown step
   * which will shut down the lifecycle node
   */
  void onShutdown() override;

  /**
   * @brief lifecycle node's implementation of cleanup step
   * which will deallocate resources
   */
  void onCleanup() override;

  // neil: callbacks!
  //rcl_interfaces::msg::SetParametersResult parametersCallback (
  //const std::vector<rclcpp::Parameter> &parameters);

protected:
  /**
  * @brief Timer callback to process the UDP socket
  */
  void processData();

  /**
  * @brief service callback to reset the lidar
  * @param request_header Header of rmw request
  * @param request Shared ptr of the Empty request
  * @param response Shared ptr of the Empty response
  */
  void resetService(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);


  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr _reset_srv;

  std::unique_ptr<SensorInterface> _sensor;
  std::multimap<ClientState, DataProcessorInterface *> _data_processors;
  rclcpp::TimerBase::SharedPtr _process_timer;

  std::string _lidar_frame_id;    // frame ID for lidar data
  bool _use_system_default_qos;
  bool _use_ros_time;
  std::uint32_t _h3_proc_mask;    // bitfield of active data output processors (PTC only now)
  uint32_t _packet_count_ref;
  uint8_t *_ptcDataOut;           // allocated buffer for pointcloud data from sensor

  // callback for 'ros2 param set'
  OnSetParametersCallbackHandle::SharedPtr callback_handler;

  void createProcessors();
  
};

}  // namespace ros2_luminar

#endif  // ROS2_LUMINAR__LUMINAR_DRIVER_HPP_
