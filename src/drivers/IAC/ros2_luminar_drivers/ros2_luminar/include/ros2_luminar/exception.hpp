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

#ifndef ROS2_LUMINAR__EXCEPTION_HPP_
#define ROS2_LUMINAR__EXCEPTION_HPP_

#include <stdexcept>
#include <string>
#include <memory>

namespace ros2_luminar
{

/**
 * @class LuminarDriverException
 * @brief Thrown when Luminar lidar driver encounters a fatal error
 */
class LuminarDriverException : public std::runtime_error
{
public:
  /**
   * @brief A constructor for ros2_luminar::LuminarDriverException
   * @param description string to display the exception message
   */
  explicit LuminarDriverException(const std::string description)
  : std::runtime_error(description) {}
};

}  // namespace ros2_luminar

#endif  // ROS2_LUMINAR__EXCEPTION_HPP_
