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

#ifndef ROS2_LUMINAR__INTERFACES__SENSOR_INTERFACE_HPP_
#define ROS2_LUMINAR__INTERFACES__SENSOR_INTERFACE_HPP_

#include <memory>

#include "ros2_luminar/interfaces/metadata.hpp"
#include "ros2_luminar/interfaces/data_processor_interface.hpp"

// [neil-rti] added
#include "LidarReturn.h"


namespace ros2_luminar
{
/**
 * @class ros2_luminar::SensorInterface
 * @brief An interface for lidars units
 */
class SensorInterface
{
public:
  using SharedPtr = std::shared_ptr<SensorInterface>;
  using Ptr = std::unique_ptr<SensorInterface>;

  /**
   * @brief A sensor interface constructor
   */
  SensorInterface() {}

  /**
   * @brief A sensor interface destructor
   */
  virtual ~SensorInterface() = default;

  // copy
  SensorInterface(const SensorInterface &) = delete;
  SensorInterface & operator=(const SensorInterface &) = delete;

  // move
  SensorInterface(SensorInterface &&) = default;
  SensorInterface & operator=(SensorInterface &&) = default;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   */
  virtual void reset(const uint16_t my_fingerprint) = 0;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   */
  virtual void configure(const uint16_t my_fingerprint,const uint16_t my_pcap_port = 5188) = 0;

  // [neil-rti] added functions ---------------------------------------------------------------------
  virtual void shutdown() = 0;                            // shut down the sensor
  virtual size_t getDiscoveredSensorCount() = 0;          // return count of discovered sensors 
  virtual int32_t getCountOfPointsInPacket() = 0;         // points in returned packet
  virtual uint32_t getPacketCounter() = 0;                // count of frames/packets from sensor
  virtual void setPtcDataPtr(uint8_t *ptcBuffer) = 0;     // set pointer to allocated data buffer

  // parameter setters
  virtual bool setScanFrequency(float freq) = 0;
  virtual void setScanFOV(float range, float center) = 0;
  virtual bool setScanPattern(int pattern, float a, float b, float c, float d, float e) = 0;
  virtual void setPtcFormat(uint32_t ptcFormat) = 0;
  virtual void setRVizFriendly(bool yesRViz) = 0;
  virtual void setDataSourceIsPcapApp(bool isPcap) = 0;
  virtual void setSensorPose(float x, float y, float z, float pitch, float roll, float yaw) = 0;
  virtual void setAdditionalData(bool yesAddData) = 0;  
};

}  // namespace ros2_luminar

#endif  // ROS2_LUMINAR__INTERFACES__SENSOR_INTERFACE_HPP_
