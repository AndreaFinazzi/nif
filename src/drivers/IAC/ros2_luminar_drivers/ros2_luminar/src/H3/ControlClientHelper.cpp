/*
* ControlClientHelper.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ros2_luminar/H3/ControlClientHelper.h"

#include <algorithm>

using namespace lum;
using namespace lum::sample;

#if 1   // neil: try to get commands working
ControlClientHelper::~ControlClientHelper() { ; }
#endif



void ControlClientHelper::InitializeModelH()
{
    auto& modelHController = lum::ModelHController::get();
    modelHController.Initialize();
    modelHController.StartResponseServer();
    modelHController.AddSensorConnectionSubscriber( this );

    mCheckResponses = true;

    //kick off thread discover heads
    mResponseThread = std::thread( &ControlClientHelper::CheckForResponses, this );

    // initialize the primary scan profile var
    mScanProfile.commonScanProfileParams.frequencyHz = 10.0;
    mScanProfile.commonScanProfileParams.fieldOfViewDegrees = 30.0;
    mScanProfile.commonScanProfileParams.centerDegrees = 0.0;
    mScanProfile.commonScanProfileParams.scanProfileType = LUM_NET_SCAN_PROFILE_TYPE_GAUSSIAN;
    mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MEAN_OFFSET_FROM_CENTER] = 0.0;
    mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_SIGMA] = 3.0;
    mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MAX_VEL] = 900.0;

    mDuration = VOLATILE;

    // This is only needed to enable PCAP (see below:AddPcapDataListener())
    // lum::ModelHDistributor::get().AddModelHListener( 5118 );
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

// update the scan pattern
bool ControlClientHelper::SetScanPattern(LumNet_ScanProfileTypes pattern, float a, float b, float c, float d, float e)
{
    bool rtnVal = true;
    if(pattern < LUM_NET_SCAN_PROFILE_TYPE_NUM_SCAN_PROFILES) {
        mScanProfile.commonScanProfileParams.scanProfileType = pattern;
        switch(pattern) {
        case LUM_NET_SCAN_PROFILE_TYPE_UNIFORM:         // 0 extra data fields
            break;
        case LUM_NET_SCAN_PROFILE_TYPE_GAUSSIAN:        // 3 extra data 
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MEAN_OFFSET_FROM_CENTER] = a;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_SIGMA] = b;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MAX_VEL] = c;
            break;
        case LUM_NET_SCAN_PROFILE_TYPE_HORIZON_FOCUS:   // 3 extra data
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_LOCATION_OFFSET_FROM_CENTER] = a;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_WIDTH] = b;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_DENSITY] = c;
            break;
        case LUM_NET_SCAN_PROFILE_TYPE_TRAPEZOID:       // 5 extra data
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_BASE_MIN_OFFSET_FROM_CENTER] = a;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_BASE_MAX_OFFSET_FROM_CENTER] = b;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_MIN_OFFSET_FROM_CENTER] = c;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_MAX_OFFSET_FROM_CENTER] = d;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_DENSITY] = e;
            break;
        case LUM_NET_SCAN_PROFILE_TYPE_EXPONENTIAL:     // 4 extra data
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_MEAN_OFFSET_FROM_CENTER] = a;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_SIGMA] = b;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_MAX_VEL] = c;
            mScanProfile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_EXPONENT] = d;
            break;
        }
        if(ValidScanProfile(mScanProfile)) {
            SendScanPattern(mScanProfile);
        }
        else {
            std::cout << "-- ScanProfile fails ValidScanProfile check --" << std::endl;
            rtnVal = false;
        }
    }
    else {
        rtnVal = false;
    }
    return rtnVal;
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

void ControlClientHelper::TransactionStatus( uint16_t fingerprint,
                                       uint16_t transactionID,
                                       CSCTransactionStatus transactionStatus,
                                       const uint8_t* response )
{
    //std::lock_guard<std::mutex> lock( this->mControlMutex );

    // TODO: respond failures if a generic scan file failed

    if ( transactionStatus != VALIDATED && transactionStatus != COMPLETED )
    {
        std::cout << "ERROR: Cmd failed, status: " << int(transactionStatus) << std::endl;
    }
}

void ControlClientHelper::GenericScanProfileUpdated( uint16_t fingerprint,
                                               uint16_t transactionID,
                                               struct LumNet_GenericScanProfile scan_profile )
{
    if ( fingerprint != this->mDiscoveredSensors[mSelectIdx]
                            .mFingerprint )  // fingerprints are mismatched, so ignore the data
    {
        std::cout << "Mismatch from sensor info and selected sensor, ignoring data" << std::endl;
        return;
    }

    if ( mLastCenter != scan_profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MEAN_OFFSET_FROM_CENTER] )
    {
        mLastCenter = scan_profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MEAN_OFFSET_FROM_CENTER];
    }

    std::cout << "\nCHANGED to: "
              << "\t[Frame Rate]: " << scan_profile.commonScanProfileParams.frequencyHz
              << "\t[Vertical FOV]: " << scan_profile.commonScanProfileParams.fieldOfViewDegrees
              << "\t[Center Offset]: " << scan_profile.commonScanProfileParams.centerDegrees
              //<< "\t[Mean Offset]: " << scan_profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MEAN_OFFSET_FROM_CENTER] << std::endl;
              << "\t[Mean Offset]: " << mLastCenter << std::endl;

    return;
}

void ControlClientHelper::InterlockStatesUpdated( uint16_t fingerprint,
                                            uint16_t transactionID,
                                            struct LumNet_Interlocks interlock_states )
{
    std::cout << "Interlock State:\t[Laser State]: "
              << std::to_string( interlock_states.laserArmed )
              << "\t[System Ok]: " << std::to_string( interlock_states.systemOk ) << std::endl;
}

void ControlClientHelper::ReadInterlockState()
{
    auto& modelHCtrl = ModelHController::get();

    modelHCtrl.ReadVolatileInterlockStates( this->mDiscoveredSensors[mSelectIdx] );

    return;
}

void ControlClientHelper::SendScanPattern( LumNet_GenericScanProfile scanProfile )
{
    mScanProfile = scanProfile;
    
    lum::ModelHController::get().SetVolatileGenericScanProfile( mDiscoveredSensors[mSensorHeadIndex], mScanProfile );
    
}

// [neil-rti] helper to find index of my fingerprint
uint8_t ControlClientHelper::FindIndexOfSensorFingerprint(uint16_t my_fingerprint)
{
    uint8_t rtnIdx = 0;
    uint8_t index = 0;
    for ( auto sensor : mDiscoveredSensors )
    {
        if (sensor.mFingerprint == my_fingerprint) {
            rtnIdx = index;
            break;
        }
        index++;
    }

    return rtnIdx;
}

// [neil-rti] adds a listener for PCAP application playback - port 5118 or 5121
void ControlClientHelper::AddPcapDataListener(uint16_t my_pcap_port)
{
    lum::ModelHDistributor::get().AddModelHListener(my_pcap_port);
}
