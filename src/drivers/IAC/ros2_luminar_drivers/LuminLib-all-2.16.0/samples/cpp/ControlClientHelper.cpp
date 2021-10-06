/*
* ControlClientHelper.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ControlClientHelper.h"

#include <algorithm>

using namespace lum;
using namespace lum::sample;

void ControlClientHelper::InitializeModelH()
{
    auto& modelHController = lum::ModelHController::get();
    modelHController.Initialize();
    modelHController.StartResponseServer();
    modelHController.AddSensorConnectionSubscriber( this );

    mCheckResponses = true;

    //kick off thread discover heads
    mResponseThread = std::thread( &ControlClientHelper::CheckForResponses, this );

    mScanProfile.commonScanProfileParams.frequencyHz = 10.0f;
    mScanProfile.commonScanProfileParams.fieldOfViewDegrees = 30.0f;
    mScanProfile.commonScanProfileParams.centerDegrees = 0.0f;

    // This is only needed to enable PCAP
    lum::ModelHDistributor::get().AddModelHListener( 5118 );
}

void ControlClientHelper::StopModelH()
{
    mCheckResponses = false;

    auto& modelHController = lum::ModelHController::get();
    modelHController.StopResponseServer();
    mResponseThread.join();

    modelHController.RemoveSensorConnectionSubscriber( this );
}

void ControlClientHelper::DiscoverSensors()
{
    lum::ModelHController::get().DiscoverSensorHeads();
    std::this_thread::sleep_for( std::chrono::seconds( 1 ) );

    std::cout << "Displaying Discovered Sensors" << std::endl;
    auto index = 0;

    for ( auto sensor : mDiscoveredSensors )
    {
        std::cout << "Sensor # " << index++ << " Fingerprint: " << sensor.mFingerprint << std::endl;
    }
}

size_t ControlClientHelper::GetDiscoveredSensorCount()
{
    return mDiscoveredSensors.size();
}

SensorUID ControlClientHelper::GetSensorID()
{
    if ( mSensorHeadIndex < GetDiscoveredSensorCount() )
    {
        return SensorUID( mDiscoveredSensors[mSensorHeadIndex].mFingerprint, mDiscoveredSensors[mSensorHeadIndex].mHeadIPAddress );
    }

    return SensorUID();
}

// Set the index of the discovered sensor we're communicating with
void ControlClientHelper::SetSensorIndex( uint8_t sensorIndex )
{
    if ( mSensorHeadIndex >= mDiscoveredSensors.size() )
    {
        std::cout << "Selected # " << mSensorHeadIndex << "  Index out of range, unable to set power" << std::endl;
        return;
    }

    mSensorHeadIndex = sensorIndex;

    // Add a listener for the sensor's port
    lum::ModelHDistributor::get().AddModelHListener( mDiscoveredSensors[mSensorHeadIndex].mDestPort );
    lum::ModelHController::get().ConnectToSensor( mDiscoveredSensors[mSensorHeadIndex] );

    std::cout << "Selected Sensor # " << static_cast<int>( mSensorHeadIndex ) << "  Fingerprint " << mDiscoveredSensors[mSensorHeadIndex].mFingerprint << std::endl;
}

void ControlClientHelper::SetDuration( const SettingDuration& duration )
{
    mDuration = duration;

    std::cout << "Subsequent scan commands will " << ( duration == VOLATILE ? "not " : "" ) << "persist on restart." << std::endl;
}

void ControlClientHelper::SetFOV( float fov, float fovCenter )
{
    mScanProfile.commonScanProfileParams.fieldOfViewDegrees = fov;
    mScanProfile.commonScanProfileParams.centerDegrees = fovCenter;

    std::cout << "Setting the FOV on sensor " << mDiscoveredSensors[mSensorHeadIndex].mFingerprint << " to " << fov << std::endl;
    std::cout << "And the center of the FOV to " << fovCenter << std::endl;

    SendRequest();
}

bool ControlClientHelper::SetFrequency( float frequency )
{
    auto tempProfile = mScanProfile;
    tempProfile.commonScanProfileParams.frequencyHz = frequency;
    auto success = ValidScanProfile( tempProfile );

    if ( success )
    {
        mScanProfile.commonScanProfileParams.frequencyHz = frequency;

        std::cout << "Setting the scan frequency on sensor " << mDiscoveredSensors[mSensorHeadIndex].mFingerprint << " to " << frequency << std::endl;

        SendRequest();
        return true;
    }
    return false;
}

bool ControlClientHelper::ValidScanProfile( const LumNet_GenericScanProfile profile )
{
    return Command_ValidateScanProfile( profile ) == LUM_NET_CMD_RETURN_RESULT_SUCCESS;
}

void ControlClientHelper::SendRequest()
{
    if ( mSensorHeadIndex >= mDiscoveredSensors.size() )
    {
        std::cout << "Unable to send the scan profile, the selected sensor index is out of range." << std::endl;
        DiscoverSensors();
        return;
    }

    // Need to determine if we're sending volatilely or persistently and send the appropriate call
    if ( mDuration == VOLATILE )
    {
        lum::ModelHController::get().SetVolatileGenericScanProfile( mDiscoveredSensors[mSensorHeadIndex], mScanProfile );
    }
    else if ( mDuration == PERSISTENT )
    {
        lum::ModelHController::get().SetPersistentGenericScanProfile( mDiscoveredSensors[mSensorHeadIndex], mScanProfile );
    }
}

void ControlClientHelper::CheckForResponses()
{
    while ( mCheckResponses )
    {
        std::vector<DiscoveryResponse> list = lum::ModelHController::get().GetDiscoveryList();
        /** Traverse current sensor list removing any sensor which is not rediscovered */
        // clang-format off
        mDiscoveredSensors.erase( std::remove_if( mDiscoveredSensors.begin(), mDiscoveredSensors.end(),
            [&]( DiscoveryResponse& sensor )
            {
                auto reDiscovered = false;
                /** Check all discovered sensors and determine if updates are required */
                list.erase(std::remove_if(list.begin(), list.end(),
                    [&](DiscoveryResponse& element)
                    {
                        if (element.mSerialNumber == sensor.mSerialNumber)
                        {
                            reDiscovered = true;
                            return true;
                        }
                        return false;
                    }), list.end());
                if ( !reDiscovered )
                {
                    lum::ModelHController::get().RemoveSensorConnectDestination( sensor );
                    lum::ModelHDistributor::get().RemoveModelHListener( sensor.mDestPort );
                    return true;
                }
                return false;
            }), mDiscoveredSensors.end() );
        // clang-format on

        for ( auto newRes : list )
        {
            mDiscoveredSensors.push_back( newRes );
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
    }
}

void ControlClientHelper::SensorConnected( uint16_t fingerprint )
{
    std::cout << "Sensor Connected: Fingerprint " << fingerprint << std::endl;
}
