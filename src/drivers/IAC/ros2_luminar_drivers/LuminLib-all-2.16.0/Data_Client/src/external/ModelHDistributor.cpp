/*
* ModelHDistributor.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ModelHDistributor.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <utility>

#include "Conversions.h"

#include <boost/asio.hpp>

using namespace lum;
using namespace std;
using namespace boost;

// Create a memory pool with 8 * 8MB pages, for up to 64 MB of data in-flight
ModelHDistributor::ModelHDistributor()
    : mPacketPool( 8192, 8 )
{
}

ModelHDistributor& ModelHDistributor::get()
{
    static ModelHDistributor instance;
    return instance;
}

void ModelHDistributor::InitializeModelHClient( bool initializeDefaultClient )
{
    if ( initializeDefaultClient )
    {
        AddModelHListener( mDefaultPort );
    }
}

void ModelHDistributor::AddModelHListener( uint16_t port )
{
    mModelHDataClients.insert( std::make_pair( port, std::make_unique<ModelHClient>( port ) ) )->second->Start();
}

void ModelHDistributor::RemoveModelHListener( uint16_t port )
{
    const auto search = mModelHDataClients.find( port );
    if ( search != mModelHDataClients.end() )
    {
        search->second->Stop();
        mModelHDataClients.erase( search );
    }
    else
    {
        std::cout << "Model H Listener not found for requested port " << port << "\n";
    }
}

bool ModelHDistributor::AddModelHDataSubscriber( IModelHSubscriber* subscriber )
{
    return mModelHDataSubscriptions.AddSubscriber( subscriber );
}

bool ModelHDistributor::RemoveModelHDataSubscriber( IModelHSubscriber* subscriber )
{
    return mModelHDataSubscriptions.RemoveSubscriber( subscriber );
}

void ModelHDistributor::FilterReturns( bool returnsFilter[4] )
{
    for ( int i = 0; i < 4; ++i )
    {
        mReturnFilters[i] = returnsFilter[i];
    }
}

void ModelHDistributor::ApplyEyeFilter( bool eyeFilter[2] )
{
    for ( int i = 0; i < 2; ++i )
    {
        mEyeFilters[i] = eyeFilter[i];
    }
}

void ModelHDistributor::SetSensorPose( SensorUID sensorID, SensorPose pose )
{
    const std::unique_lock<std::mutex> lock( mPoseMutex );
    mSensorPoses[sensorID] = pose;
}

bool ModelHDistributor::GetSensorPose( SensorUID sensorID, SensorPose& pose )
{
    const std::unique_lock<std::mutex> lock( mPoseMutex );
    const auto poseIt = mSensorPoses.find( sensorID );
    if ( poseIt != mSensorPoses.end() )
    {
        pose = poseIt->second;
        return true;
    }

    return false;
}

void ModelHDistributor::ResetSensorPoses()
{
    const std::unique_lock<std::mutex> lock( mPoseMutex );
    mSensorPoses.clear();
}

void ModelHDistributor::Cartesian( bool cartesian )
{
    mConvertToCartesian = cartesian;
}

void ModelHDistributor::Degrees( bool returnDegrees )
{
    mConvertToDegrees = returnDegrees;
}

void ModelHDistributor::PointTransmissionFrequency( TransmissionFrequency frequency )
{
    mTransmissionFq = frequency;
}

void ModelHDistributor::PacketReceived( uint8_t* pPacketData, size_t packetSize, std::string address )
{
    const auto packet = mPacketPool.alloc( pPacketData, packetSize );
    // If we've maxed out our pool of packets,
    if ( packet )
    {
        const std::unique_lock<std::mutex> lock( mQueueMutex );
        mPacketQueue.push( {address, packet} );
    }
}

void lum::ModelHDistributor::SetEmptyRaysRange( float emptyRaysRange )
{
    mEmptyRaysRange = emptyRaysRange;
}

float lum::ModelHDistributor::GetEmptyRaysRange()
{
    return mEmptyRaysRange;
}

void ModelHDistributor::StartTransmitting()
{
    mModelHDataSubscriptions.StartData();
    for ( const auto& e : mModelHDataClients )
    {
        e.second->Start();
    }
}

void ModelHDistributor::StopTransmitting()
{
    for ( const auto& e : mModelHDataClients )
    {
        e.second->Stop();
    }
    mModelHDataSubscriptions.StopData();
}

void ModelHDistributor::ShowSnapback( bool show )
{
    mShowSnapback = show;
}

bool ModelHDistributor::ShowSnapback()
{
    return mShowSnapback;
}

void ModelHDistributor::ShowFrame( bool show )
{
    mShowFrame = show;
}

bool ModelHDistributor::ShowFrame()
{
    return mShowFrame;
}

bool ModelHDistributor::ShowCheckpoint( const LumNet_ScanProfileType scanProfile, const uint8_t checkpoint )
{
    if ( scanProfile == LUM_NET_SCAN_PROFILE_TYPE_HORIZON_FOCUS )
    {
        if ( checkpoint == 3 )
        {
            return false;  // snapback has checkpoint 3
        }
    }
    else if ( scanProfile == LUM_NET_SCAN_PROFILE_TYPE_TRAPEZOID )
    {
        if ( checkpoint == 5 )
        {
            return false;  // snapback has checkpoint 5
        }
    }
    else
    {
        if ( checkpoint == 1 )
        {
            return false;  // snapback has checkpoint 1
        }
    }
    // Otherwise, show it
    return true;
}

bool ModelHDistributor::ProcessPacketData()
{
    ModelHDataPacketParser* packet = nullptr;
    boost::asio::ip::address address;
    {
        const std::unique_lock<std::mutex> lock( mQueueMutex );
        if ( !mPacketQueue.empty() )
        {
            const auto nextItem = mPacketQueue.front();
            address = boost::asio::ip::address::from_string( nextItem.first );
            packet = nextItem.second;
            mPacketQueue.pop();
        }
    }

    if ( packet )
    {
        const uint16_t fingerprint = packet->GetFingerPrint();
        const auto scanCount = packet->GetLegacyScanCount();
        const auto sensor = SensorUID( fingerprint, static_cast<uint32_t>( address.to_v4().to_ulong() ) );
        bool frameComplete = false;

        if ( mPointClouds.find( sensor ) == mPointClouds.end() )
        {
            mPointClouds[sensor].reserve( MAX_NUM_RETURNS_PER_SENSOR );
        }

        std::vector<LidarReturn>& perFrameCloud = mPointClouds[sensor];
        std::map<uint8_t, std::vector<LidarReturn>>& perSSICloud = mSSIPoints[sensor];
        std::map<uint32_t, std::vector<LidarReturn>>& perLineCloud = mLinePoints[sensor];

        if ( scanCount != mSensorScanCounts[sensor] )
        {
            frameComplete = true;
            mSensorScanCounts[sensor] = scanCount;
        }
        if ( perFrameCloud.size() > MAX_NUM_RETURNS_PER_SENSOR )
        {
            frameComplete = true;
        }

        if ( frameComplete )
        {
            ReceiveContext receiveContext;
            receiveContext.includesMissingData = !mReturnFilters[EMPTY_RAYS_FILTER_INDEX];
            receiveContext.frameComplete = true;
            if ( mTransmissionFq & TransmissionFrequency::Frame )
            {
                // TODO: need to enforce state is constant across a frame / demarcator boundary
                receiveContext.transmissionFq = TransmissionFrequency::Frame;
                mModelHDataSubscriptions.PointsReady( sensor, perFrameCloud, receiveContext );
                perFrameCloud.clear();
            }
            // If we're transmitting per line, if any line contains elements, broadcast them
            if ( mTransmissionFq & TransmissionFrequency::Line )
            {
                receiveContext.transmissionFq = TransmissionFrequency::Line;
                for ( const auto& lineCloud : perLineCloud )
                {
                    mModelHDataSubscriptions.PointsReady( sensor, lineCloud.second, receiveContext );
                }

                perLineCloud.clear();
            }
            if ( mTransmissionFq & TransmissionFrequency::SSI )
            {
                receiveContext.transmissionFq = TransmissionFrequency::SSI;
                for ( int i = 0; i < ( 1 << NUM_EYE_BITS ); i++ )
                {
                    if ( perSSICloud[i].size() > 0 )
                    {
                        mModelHDataSubscriptions.PointsReady( sensor, perSSICloud[i], receiveContext );
                    }
                    perSSICloud[i].clear();
                }
            }
        }

        const uint8_t checkpoint = packet->GetMostRecentCheckpointLegacy();
        const bool isSnapback = !ShowCheckpoint( packet->GetScanProfile(), checkpoint );
        const auto showingFrame = !isSnapback && ShowFrame();
        const auto showingSnapback = isSnapback && ShowSnapback();

        /*
        * The packet either contains valid frame data or snapback,
        * if the user chooses to filter that type of data, drop the packet and return
        */
        if ( !showingFrame && !showingSnapback )
        {
            mPacketPool.free( packet );  // always cleanup packet
            return true;
        }

        const uint8_t numRays = packet->GetNumRays();
        ModelHReturnIterator& returnIterator = packet->GetReturnIterator();

        // variables for applying sensor pose if one is set
        auto poseIter = std::map<SensorUID, SensorPose>::const_iterator();
        bool applyPoseOffset = false;
        {
            const std::unique_lock<std::mutex> lock( mPoseMutex );
            poseIter = mSensorPoses.find( sensor );
            applyPoseOffset = poseIter != mSensorPoses.end();
        }

        while ( !returnIterator.End() && returnIterator.RayIndex() < numRays )
        {
            const bool returnValid = returnIterator.ReturnValid();
            const auto eye = returnIterator.Eye();
            int returnIndex = 0;

            /*
            * If the return is valid, or we're permitting artifacts from returns that we're not filtering out
            * create a lidar point and add it to the point cloud
            */
            const bool showingEmptyRays = !returnValid && !mReturnFilters[EMPTY_RAYS_FILTER_INDEX] && returnIndex == 0;
            const bool showingValidReturn = returnValid && !mReturnFilters[returnIndex] && returnIndex != EMPTY_RAYS_FILTER_INDEX;
            const bool showingLeftEye = eye == LEFT_EYE_INDEX && !mEyeFilters[LEFT_EYE_INDEX];
            const bool showingRightEye = eye == RIGHT_EYE_INDEX && !mEyeFilters[RIGHT_EYE_INDEX];

            if ( ( showingValidReturn || showingEmptyRays ) && ( showingLeftEye || showingRightEye ) )
            {
                const float range = returnValid ? returnIterator.Range() : mEmptyRaysRange;
                const float azimuth = returnIterator.Azimuth();
                const float elevation = returnIterator.Elevation();
                const float reflectance = returnIterator.Reflectance();
                const uint32_t microsecondTimestamp = returnIterator.MicrosecondTimestamp();
                const uint8_t scanSegmentIndex = returnIterator.ScanSegmentIndex();
                const uint8_t packetSequenceNumber = packet->GetSequenceNumber();
                const uint8_t scanProfile = packet->GetScanProfile();
                const uint8_t scanCount = packet->GetLegacyScanCount();
                const uint32_t unixTimestamp = packet->GetUnixTimeStamp();
                const uint8_t packedReturnIndex = returnValid ? returnIndex : EMPTY_RAY_INDEX;

                LidarReturn newReturn = LidarReturn(
                    azimuth,
                    elevation,
                    range,
                    reflectance,
                    unixTimestamp,
                    microsecondTimestamp,
                    fingerprint,
                    packedReturnIndex,
                    packetSequenceNumber,
                    scanProfile,
                    scanCount,
                    checkpoint,
                    scanSegmentIndex );

                if ( mConvertToCartesian )
                {
                    SphericalToCartesian( newReturn );
                    ApplyParallaxCorrection( newReturn );

                    if ( applyPoseOffset )
                    {
                        ApplyPose( newReturn, poseIter->second );
                    }
                }
                else if ( mConvertToDegrees )  // only valid if not Cartesian
                {
                    RadiansToDegrees( newReturn );
                }

                //Handle per-SSI and per-Line at the same time
                if ( ( mTransmissionFq & TransmissionFrequency::Line ) || ( mTransmissionFq & TransmissionFrequency::SSI ) )
                {
                    const auto eye = returnIterator.Eye();
                    //Compare the line number of this return vs. the last return for this eye.
                    //If we've moved on to a new eye, push this one out and move on.
                    if ( ( perSSICloud[eye].size() > 0 ) && ( perSSICloud[eye][0].scan_segment_index != newReturn.scan_segment_index ) )
                    {
                        ReceiveContext receiveContext;
                        receiveContext.includesMissingData = !mReturnFilters[EMPTY_RAYS_FILTER_INDEX];

                        if ( mTransmissionFq & TransmissionFrequency::Line )
                        {
                            const auto scanLine = ExtractScanSegmentIndex( newReturn );

                            //There are two eyes per line updating each...if data exists already, publish everything and clear it.
                            const bool publishLine = perLineCloud[scanLine].size() > 0;

                            //We're about to blow away the half-line (SSI), so store it into a vector indexed with the line number.
                            std::copy( perSSICloud[eye].begin(), perSSICloud[eye].end(), back_inserter( perLineCloud[scanLine] ) );
                            if ( publishLine )
                            {
                                receiveContext.transmissionFq = TransmissionFrequency::Line;
                                const auto linePoints = perLineCloud.find( scanLine );
                                mModelHDataSubscriptions.PointsReady( sensor, linePoints->second, receiveContext );
                                perLineCloud.erase( linePoints );
                            }
                        }
                        if ( mTransmissionFq & TransmissionFrequency::SSI )
                        {
                            receiveContext.transmissionFq = TransmissionFrequency::SSI;
                            mModelHDataSubscriptions.PointsReady( sensor, perSSICloud[eye], receiveContext );
                        }

                        perSSICloud[eye].clear();
                    }

                    perSSICloud[eye].emplace_back( std::move( newReturn ) );
                }
                if ( mTransmissionFq & TransmissionFrequency::Frame )
                {
                    perFrameCloud.emplace_back( std::move( newReturn ) );
                }

                returnIndex = returnValid ? ++returnIndex : 0;
            }
            else
            {
                //If we're filtering out that return, but it was still valid, don't move on to the next point
                if ( returnValid && ( returnIndex >= 3 || mReturnFilters[returnIndex] ) )
                {
                    ++returnIndex;

                    // assert that we never actually reach our empty return sentinel
                    assert( returnIndex != EMPTY_RAY_INDEX );
                }
                else
                {
                    returnIndex = 0;
                }
            }
            returnIterator.NextReturn();
        }

        static bool warningPrinted = false;
        if ( !warningPrinted && returnIterator.RayIndex() == numRays - 1 && !returnIterator.End() )
        {
            cout << "Reached the last ray but there is still data in the packet!";
            warningPrinted = true;
        }

        mPacketPool.free( packet );
        return true;
    }
    return false;
}
