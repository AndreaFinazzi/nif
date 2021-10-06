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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "ros2_luminar/driver_types.hpp"
#include "ros2_luminar/H3/H3_sensor.hpp"

namespace ros2_luminar
{
ros2_luminar::H3Driver::H3Driver(rclcpp::NodeOptions options)
: LuminarDriver{std::make_unique<H3::H3Sensor>(), options} {}
}

RCLCPP_COMPONENTS_REGISTER_NODE(ros2_luminar::H3Driver)
