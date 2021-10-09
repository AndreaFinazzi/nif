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

#ifndef ROS2_LUMINAR__INTERFACES__DATA_PROCESSOR_INTERFACE_HPP_
#define ROS2_LUMINAR__INTERFACES__DATA_PROCESSOR_INTERFACE_HPP_

#include <memory>
#include <string>
#include "ros2_luminar/interfaces/metadata.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_luminar
{
/**
 * @class ros2_luminar::DataProcessorInterface
 * @brief An interface for data processors using a
 * lidar-specific API to create structured information
 * like pointclouds, images, or directly publishing
 * packets.
 */
class DataProcessorInterface
{
public:
  /**
   * @brief Constructor of the data processor interface
   */
  DataProcessorInterface() {}

  /**
   * @brief Destructor of the data processor interface
   */
  virtual ~DataProcessorInterface() = default;

  /**
   * @brief Process a packet with the lidar-specific APIs
   * @param data packet input
   * @param pointcount total number of points in sample
   * @param override_ts Timestamp in nanos to use to override the ts in the
   *                    packet data. To use the packet data, pass as 0.
   */
  virtual bool process(uint8_t * data, int32_t pointcount, uint64_t override_ts = 0) = 0;

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  virtual void onActivate() = 0;

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  virtual void onDeactivate() = 0;

};

}  // namespace ros2_luminar

#endif  // ROS2_LUMINAR__INTERFACES__DATA_PROCESSOR_INTERFACE_HPP_
