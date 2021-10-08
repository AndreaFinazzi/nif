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

#ifndef ROS2_LUMINAR__H3__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
#define ROS2_LUMINAR__H3__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_

#include <vector>
#include <memory>
#include <string>
#include <utility>

#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ros2_luminar/interfaces/data_processor_interface.hpp"

namespace H3
{
/**
 * @class H3::PointcloudProcessor
 * @brief A data processor interface implementation of a processor
 * for creating Pointclouds in the
 * driver in ROS2.
 */
class PointcloudProcessor : public ros2_luminar::DataProcessorInterface
{
public:
  /**
   * @brief A constructor for H3::PointcloudProcessor
   * @param node Node for creating interfaces
   * @param mdata metadata about the sensor
   * @param frame frame_id to use for messages
   */
  PointcloudProcessor(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    const std::string & frame,
    const rclcpp::QoS & qos, bool extended)
  : DataProcessorInterface(), _node(node), _frame_id(frame)
  {
    mIsExtended = extended;

    // init the to-be-published pointcloud sample
    _ros_pointcloud.is_bigendian = false;
    _ros_pointcloud.is_dense = true;          // true=no invalid datapoints
    _ros_pointcloud.point_step = addPointField(_ros_pointcloud, "x", sensor_msgs::msg::PointField::FLOAT32);
    _ros_pointcloud.point_step = addPointField(_ros_pointcloud, "y", sensor_msgs::msg::PointField::FLOAT32);
    _ros_pointcloud.point_step = addPointField(_ros_pointcloud, "z", sensor_msgs::msg::PointField::FLOAT32);
    _ros_pointcloud.point_step = addPointField(_ros_pointcloud, "intensity", sensor_msgs::msg::PointField::FLOAT32);
    if(mIsExtended) {
      _ros_pointcloud.point_step = addPointField(_ros_pointcloud, "ring", sensor_msgs::msg::PointField::FLOAT32);
      _ros_pointcloud.point_step = addPointField(_ros_pointcloud, "time", sensor_msgs::msg::PointField::FLOAT32);
    }

    _ros_pointcloud.header.frame_id = _frame_id;
    // for the moment, make this be 1 row x (n) columns(determined at publication)
    _ros_pointcloud.height = 1;
    _ros_pointcloud.width = 0; // will match the number of point
    _ros_pointcloud.row_step = _ros_pointcloud.width*_ros_pointcloud.point_step; 
    _pub = _node->create_publisher<sensor_msgs::msg::PointCloud2>(_node->get_parameter("lidar_topic_name").as_string(), qos);
  }


  uint32_t addPointField(sensor_msgs::msg::PointCloud2 &msg, std::string name, uint8_t datatype) {
    uint32_t size = 0;
    if ((datatype == sensor_msgs::msg::PointField::INT8) || (datatype == sensor_msgs::msg::PointField::UINT8))
        size = 1;
    else if ((datatype == sensor_msgs::msg::PointField::INT16) || (datatype == sensor_msgs::msg::PointField::UINT16))
        size = 2;
    else if ((datatype == sensor_msgs::msg::PointField::INT32) || (datatype == sensor_msgs::msg::PointField::UINT32) ||
        (datatype == sensor_msgs::msg::PointField::FLOAT32))
        size = 4;
    else if (datatype == sensor_msgs::msg::PointField::FLOAT64)
        size = 8;
    else
        throw std::string("Pointcloud datatype not valid");
    
    sensor_msgs::msg::PointField pf;
    pf.name = name;
    pf.offset = msg.fields.empty() ? 0 : msg.fields.back().offset + size;
    pf.datatype = datatype;
    pf.count = 1;
    msg.fields.push_back(pf);
    return pf.offset + size;
  } 


  /**
   * @brief A destructor clearing memory allocated
   */
  ~PointcloudProcessor()
  {
    _pub.reset();
  }

  /**
   * @brief Process method to create pointcloud
   * @param data the packet data
   */
  bool process(uint8_t * data, int32_t pointcount, uint64_t override_ts) override
  {
    _ros_pointcloud.header.stamp.sec    =  override_ts / 1e9;
    _ros_pointcloud.header.stamp.nanosec  = (override_ts - _ros_pointcloud.header.stamp.sec*1e9);
    _ros_pointcloud.width = pointcount;
    _ros_pointcloud.row_step = (_ros_pointcloud.width * _ros_pointcloud.point_step);
    _ros_pointcloud.data.resize(_ros_pointcloud.height * _ros_pointcloud.row_step);
    memcpy(_ros_pointcloud.data.data(), data, _ros_pointcloud.height * _ros_pointcloud.row_step);
    _pub->publish(_ros_pointcloud);
    return true;
  }

  /**
   * @brief Activating processor from lifecycle state transitions
   */
  void onActivate() override
  {
    _pub->on_activate();
  }

  /**
   * @brief Deactivating processor from lifecycle state transitions
   */
  void onDeactivate() override
  {
    _pub->on_deactivate();
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;
  rclcpp_lifecycle::LifecycleNode::SharedPtr _node;
  std::string _frame_id;
  sensor_msgs::msg::PointCloud2 _ros_pointcloud;
  bool mIsExtended = false;
};

}  // namespace H3

#endif  // ROS2_LUMINAR__H3__PROCESSORS__POINTCLOUD_PROCESSOR_HPP_
