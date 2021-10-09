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

#include <string>

#include "ros2_luminar/H3/H3_sensor.hpp"
#include "ros2_luminar/exception.hpp"
#include "ros2_luminar/interfaces/metadata.hpp"

namespace H3
{

H3Sensor::H3Sensor()
: SensorInterface()
{}

H3Sensor::~H3Sensor()
{}

void H3Sensor::reset(const uint16_t my_lidar_fingerprint)
{
  configure(my_lidar_fingerprint);
}

void H3Sensor::configure(const uint16_t my_lidar_fingerprint,const uint16_t my_pcap_port)
{
    // Init the luminar support code (uses UDP port 11000 for discovery)
    mControlHelper.InitializeModelH();

    if( mDataSourceIsPcapApp ) {
      // use if sensor input comes from from pcap application,
      mControlHelper.AddPcapDataListener(my_pcap_port);
      lum::SensorUID tmpID(my_lidar_fingerprint, 0x7f000001);
      mDataHelper.SetSensorID( tmpID );
    }
    else {
      // use data from the sensor unit
      // this discovers the sensors, building a list
      mControlHelper.DiscoverSensors();
      if(0 == mControlHelper.GetDiscoveredSensorCount()) {
        throw ros2_luminar::LuminarDriverException(
              std::string("No LiDAR sensor detected"));
        exit(-1);
      }

      // find the index of lidar with my_lidar_fingerprint
      uint8_t myIdx = mControlHelper.FindIndexOfSensorFingerprint(my_lidar_fingerprint);
      mControlHelper.SetSensorIndex( myIdx );

      lum::SensorUID tmpID = mControlHelper.GetSensorID();
      // info print
      fprintf(stderr, "Idx: %u, ID: %u at IP: %u.%u.%u.%u\n", 
        myIdx,
        tmpID.mFingerprint,
        (tmpID.mHeadIPAddress >> 24) & 0xff,
        (tmpID.mHeadIPAddress >> 16) & 0xff,
        (tmpID.mHeadIPAddress >>  8) & 0xff,
        (tmpID.mHeadIPAddress >>  0) & 0xff);

      if(my_lidar_fingerprint != tmpID.mFingerprint) {
        throw ros2_luminar::LuminarDriverException(
              std::string("My LiDAR is not detected (yet?)"));
        exit(-1);
      }
      mDataHelper.SetSensorID( tmpID );
    }
}

//np added functions ---------------------------------------------------------------
void H3Sensor::shutdown()
{
  mDataHelper.Shutdown();
  mControlHelper.StopModelH();
}

size_t H3Sensor::getDiscoveredSensorCount()
{
  return mControlHelper.GetDiscoveredSensorCount();
}

void H3Sensor::setPtcDataPtr(uint8_t *ptcBuffer)
{
   mDataHelper.SetPtcDataPtr(ptcBuffer); 
}

// parameter setters
bool H3Sensor::setScanFrequency(float freqHz)
{
  return mControlHelper.SetFrequency(freqHz);
}

void H3Sensor::setScanFOV(float range, float center) 
{
  mControlHelper.SetFOV(range, center);
  return;
}

bool H3Sensor::setScanPattern(int type, float a, float b, float c, float d, float e) 
{
  return mControlHelper.SetScanPattern((LumNet_ScanProfileTypes)type, a, b, c, d, e);
}

void H3Sensor::setPtcFormat(uint32_t ptcFormat)
{
  mDataHelper.SetCoordinateType(ptcFormat);
}

void H3Sensor::setSensorPose(float x, float y, float z, float pitch, float roll, float yaw)
{
  lum::SensorPose myPose = {x, y, z, pitch, roll, yaw};
  mDataHelper.SetSensorPose(myPose);
}
}  // namespace H3
