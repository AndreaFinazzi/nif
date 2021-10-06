/*
* DataClientHelper.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "DataClientHelper.h"
#include <boost/asio.hpp>
#include <iostream>

#include "ModelHDistributor.h"

using namespace lum;
using namespace lum::sample;

DataClientHelper::DataClientHelper()
    : mSensorID( SensorUID( 5118, boost::asio::ip::address::from_string( "127.0.0.1" ).to_v4().to_ulong() ) )
{
    auto& hDistributor = ModelHDistributor::get();
    hDistributor.InitializeModelHClient();
    hDistributor.StartTransmitting();

    hDistributor.AddModelHDataSubscriber( this );

    // mHSubscriptionPaused is true by default and will block points from being distributed
    mHSubscriptionPaused = false;
}

DataClientHelper::~DataClientHelper()
{
}

void DataClientHelper::ReceiveModelHPoints( SensorUID sensor,
                                            const std::vector<LidarReturn>& points,
                                            ReceiveContext receiveContext )
{
    if ( sensor != mSensorID )
    {
        return;
    }

    std::unique_lock<std::mutex> lock( mPointCloudMutex );
    mReturns = points;
}

void DataClientHelper::SetSensorID( SensorUID sensor )
{
    std::unique_lock<std::mutex> lock( mPointCloudMutex );
    mSensorID = sensor;
}

#define DISPLAY_RETURN_COUNT 10

void DataClientHelper::DisplayReturns()
{
    std::unique_lock<std::mutex> lock( mPointCloudMutex );

    if ( mReturns.size() < DISPLAY_RETURN_COUNT )
    {
        std::cout << "No returns to display, is there data being streamed in?" << std::endl;
        return;
    }

    //Print off some of the points to show the values being returned
    std::cout << "Display Live Returns:" << std::endl;
    for ( auto i = 0; i < DISPLAY_RETURN_COUNT; i++ )
    {
        printf( "\nTimestamp us:\t%d\n", mReturns[i].microsecond_timestamp );
        printf( "Azimuth:\t%f\n", mReturns[i].azimuth );
        printf( "Elevation:\t%f\n", mReturns[i].elevation );
        printf( "SSI:\t\t%d\n", mReturns[i].scan_segment_index );
        printf( "EyeIndex:\t%d\n", ExtractEyeIndex( mReturns[i] ) );
        printf( "ReturnIndex:\t%d\n", mReturns[i].return_index );
    }
}

void DataClientHelper::Shutdown()
{
    auto& hDistributor = ModelHDistributor::get();
    hDistributor.StopTransmitting();
    hDistributor.RemoveModelHDataSubscriber( this );
}
