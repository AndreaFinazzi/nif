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

#ifndef ROS2_LUMINAR__H3__PROCESSOR_FACTORIES_HPP_
#define ROS2_LUMINAR__H3__PROCESSOR_FACTORIES_HPP_

#include <cstdint>
#include <string>
#include <map>
#include <utility>

#include "rclcpp/qos.hpp"
#include "ros2_luminar/string_utils.hpp"
#include "ros2_luminar/H3/processors/pointcloud_processor.hpp"

namespace ros2_luminar
{

constexpr std::uint32_t H3_PROC_IMG = (1 << 0);
constexpr std::uint32_t H3_PROC_PCL = (1 << 1);
constexpr std::uint32_t H3_PROC_SCAN = (1 << 2);
constexpr std::uint32_t H3_PROC_PCL_EXTENDED = (1 << 3);

constexpr std::uint32_t H3_DEFAULT_PROC_MASK = H3_PROC_PCL;

/**
 * Transforms a data processor mask-like-string into a mask value
 *
 * We define a mask-like-string to be a pipe-separated list of
 * data processor suffixes. For example, all of the following
 * are valid:
 *
 * IMG|PCL|SCAN
 * IMG|PCL
 * PCL
 *
 * @param[in] mask_str The string to convert into a mask
 * @return The mask obtained from the parsed input string.
 */
inline std::uint32_t
toProcMask(const std::string & mask_str)
{
  std::uint32_t mask = 0x0;
  auto tokens = ros2_luminar::split(mask_str, '|');

  for (auto & token : tokens) {
    if (token == "IMG") {
      mask |= ros2_luminar::H3_PROC_IMG;
    } else if (token == "PCL") {
      mask |= ros2_luminar::H3_PROC_PCL;
    } else if (token == "SCAN") {
      mask |= ros2_luminar::H3_PROC_SCAN;
    } else if (token == "PCL_EXTENDED") {
      mask |= ros2_luminar::H3_PROC_PCL_EXTENDED;
    }     
  }

  return mask;
}

/**
 * @brief Factory method to get a pointer to a processor
 * to create the pointcloud (PointXYZ be default) interface
 * @return Raw pointer to a data processor interface to use
 */
inline ros2_luminar::DataProcessorInterface * createPointcloudProcessor(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & frame,
  const rclcpp::QoS & qos, bool extended)
{
  return new H3::PointcloudProcessor(node, frame, qos, extended);
}

inline std::multimap<ClientState, DataProcessorInterface *> createProcessors(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  const std::string & laser_frame,
  const rclcpp::QoS & qos,
  std::uint32_t mask = ros2_luminar::H3_DEFAULT_PROC_MASK)
{
  std::multimap<ClientState, DataProcessorInterface *> data_processors;
  if ((mask & ros2_luminar::H3_PROC_PCL) == ros2_luminar::H3_PROC_PCL) {
    data_processors.insert(
      std::pair<ClientState, DataProcessorInterface *>(
        ClientState::LIDAR_DATA, createPointcloudProcessor(
          node, laser_frame, qos, false)));
  } else {
    if ((mask & ros2_luminar::H3_PROC_PCL_EXTENDED) == ros2_luminar::H3_PROC_PCL_EXTENDED) {
      data_processors.insert(
        std::pair<ClientState, DataProcessorInterface *>(
          ClientState::LIDAR_DATA, createPointcloudProcessor(
            node, laser_frame, qos, true)));
    }
  }
  return data_processors;
}

}  // namespace ros2_luminar

#endif  // ROS2_LUMINAR__H3__PROCESSOR_FACTORIES_HPP_
