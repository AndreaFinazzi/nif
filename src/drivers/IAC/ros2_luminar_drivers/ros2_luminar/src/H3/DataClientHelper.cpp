/*
* DataClientHelper.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ros2_luminar/H3/DataClientHelper.h"
#include <boost/asio.hpp>
#include <iostream>
#include "ModelHDistributor.h"

using namespace lum;
using namespace lum::sample;

// helper class for lidar data.  Defaults to use PCAP player data source, or this 
// can be overwritten with SetSensorID()
DataClientHelper::DataClientHelper()
    : mSensorID( SensorUID( 5118, boost::asio::ip::address::from_string( "127.0.0.1" ).to_v4().to_ulong() ) )
{

    auto& hDistributor = ModelHDistributor::get();
    hDistributor.InitializeModelHClient();
    hDistributor.StartTransmitting();       // converts and processes
    hDistributor.AddModelHDataSubscriber( this );

    // mHSubscriptionPaused is true by default and will block points from being distributed
    mHSubscriptionPaused = false;

    // [neil-rti] added
    mFrameCounter = 0;
    mPointsInPacket = 0;
    mSwapYZforRViz = false;
}

DataClientHelper::~DataClientHelper()
{
}

// this is the callback for LiDAR data received.
void DataClientHelper::ReceiveModelHPoints( SensorUID sensor,
                                            const std::vector<LidarReturn>& points,
                                            ReceiveContext receiveContext )
{
    if ( sensor != mSensorID ) { return; }

    std::unique_lock<std::mutex> lock( mPointCloudMutex );
    mPointsInPacket = points.size();
    if(points.size() > 5) {
        float *fbuf = reinterpret_cast<float*>(mPtcDataOut);
        if(mSwapYZforRViz || mAdditionalData) {
            // get timestamp of first point, other points are expressed as seconds elapsed from the first
            uint64_t startTs = points.at(0).unix_timestamp*1000000 + points.at(0).microsecond_timestamp;
            int stride = mAdditionalData? 6 : 4;
            // this swaps Y&Z for better RViz viewing, but takes longer
            for (int i=0 ; i<mPointsInPacket ; i++) {
                fbuf[(i*stride)+0] = points.at(i).z;
                // swap y with z for RViz-normal view
                fbuf[(i*stride)+2] = points.at(i).y;
                fbuf[(i*stride)+1] = -points.at(i).x;
                
                // convert reflectivity (0..1 float) to 24-bit RGB
                //uint32_t rcolor = static_cast<uint32_t>(points.at(i).r * 16777215.0f);
                //fbuf[(i*stride)+3] = *reinterpret_cast<float*>(&rcolor);
                fbuf[(i*stride)+3] = points.at(i).r;
                
                if(mAdditionalData) {
                    fbuf[(i*stride)+4] = points.at(i).scan_segment_index;
                    uint64_t curTs = points.at(i).unix_timestamp*1000000 + points.at(i).microsecond_timestamp;
                    fbuf[(i*stride)+5] = (float)(curTs - startTs)/1000000.0;
                }
            }
        }
        else {
            // This does a flat copy (faster, but it's not great for rviz viewing)
            for (int i=0 ; i<mPointsInPacket ; i++) {
                memcpy(fbuf + (i*4), &points.at(i), 16);
            }
        }
        mFrameCounter++;
    }
}

void DataClientHelper::SetSensorID( SensorUID sensor )
{
    std::unique_lock<std::mutex> lock( mPointCloudMutex );
    mSensorID = sensor;
}


void DataClientHelper::SetSensorPose(SensorPose myPose) {
    auto& hDistributor = ModelHDistributor::get();
    hDistributor.SetSensorPose(mSensorID, myPose);
    hDistributor.AddModelHDataSubscriber( this );
}

// set output coordinate type: Cartesian or Spherical
void DataClientHelper::SetCoordinateType(uint32_t coordType)
{
    auto& hDistributor = ModelHDistributor::get();
    hDistributor.Cartesian(coordType & 0x1);
}

void DataClientHelper::Shutdown()
{
    auto& hDistributor = ModelHDistributor::get();
    hDistributor.StopTransmitting();
    hDistributor.RemoveModelHDataSubscriber( this );
}
