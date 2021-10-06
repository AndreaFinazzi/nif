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

#ifndef ROS2_LUMINAR__H3__H3_SENSOR_HPP_
#define ROS2_LUMINAR__H3__H3_SENSOR_HPP_

#include <memory>
#include <vector>

#include "ros2_luminar/H3/processor_factories.hpp"

#include "ros2_luminar/interfaces/data_processor_interface.hpp"
#include "ros2_luminar/interfaces/sensor_interface.hpp"

//np added
#include "ros2_luminar/H3/ControlClientHelper.h"
#include "ros2_luminar/H3/DataClientHelper.h"

namespace H3
{

class H3Sensor : public ros2_luminar::SensorInterface
{
public:
  H3Sensor();

  ~H3Sensor() override;

  /**
   * @brief Reset lidar sensor
   * @param configuration file to use
   */
  void reset(const uint16_t my_lidar_fingerprint) override;

  /**
   * @brief Configure lidar sensor
   * @param configuration file to use
   */
  void configure(const uint16_t my_fingerprint,const uint16_t my_pcap_port = 5188) override;

  //np new additions -------------------------------------------------------------
  /**
   * @brief shutdown the lidar sensor
   */
  void shutdown(void) override;

  size_t getDiscoveredSensorCount(void) override;

  // returns the count of points in the most-recent packet from sensor
  int32_t getCountOfPointsInPacket(void) override { return mDataHelper.GetPointsCount(); }
  uint32_t getPacketCounter(void) override { return mDataHelper.GetFrameCount(); }
  void setPtcDataPtr(uint8_t *ptcBuffer) override;

  // parameter setters
  bool setScanFrequency(float freqHz) override;
  void setScanFOV(float range, float center) override;
  bool setScanPattern(int pattern, float a, float b, float c, float d, float e) override;
  void setPtcFormat(uint32_t ptcFormat) override;
  void setRVizFriendly(bool yesRViz) override { mDataHelper.SetRVizFriendly(yesRViz); }
  void setDataSourceIsPcapApp(bool isPcap) override { mDataSourceIsPcapApp = isPcap; }
  void setSensorPose(float x, float y, float z, float pitch, float roll, float yaw) override;

  void setAdditionalData(bool yesAddData) override { mDataHelper.setAdditionalData(yesAddData); }
  
private:
  lum::sample::ControlClientHelper mControlHelper;
  lum::sample::DataClientHelper mDataHelper;
  bool mDataSourceIsPcapApp = false;
};

}  // namespace H3

#endif  // ROS2_LUMINAR__H3__H3_SENSOR_HPP_
