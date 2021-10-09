/*
* DiscoveryResponse.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief DiscoveryResponse
 *         The `DiscoveryResponse` class stores information returned from a `Discovery` request.

 */

#pragma once

#include <stdint.h>
#include <string>
#include "SensorUID.h"

#include <LumNet/Common.h>
#include <LumNet/Discovery.h>

#include <LegacyLumNet/Common.h>
#include <LegacyLumNet/Discovery.h>

namespace lum
{
enum class DiscoveryVersion
{
    LegacyV1 = 0,
    LegacyV2,
    LegacyV4,
    Latest,
    NumTypes
};

class DiscoveryResponse : public lum::SensorUID
{
public:
    std::string mSerialNumber, mHeadDescription, mMacAddress;
    float mBootCompletionPercent = 0.f;
    uint32_t mDestIPAddress = 0;
    uint32_t mSubNetMask = 0;
    uint32_t mGateway = 0;
    uint16_t mDestPort = 0;
    uint16_t mDscUDPPort = 0;
    uint16_t mCmdTCPPort = 0;
    uint16_t mMTU = 0;
    uint16_t mNetworkType = 0;
    uint8_t mDiscoveryMajor = 0;
    uint8_t mDiscoveryMinor = 0;
    uint8_t mDiscoveryPatch = 0;
    uint8_t mPayloadID = 0;
    uint8_t mFirmwareMajor = 0;
    uint8_t mFirmwareMinor = 0;
    uint8_t mFirmwarePatch = 0;
    uint8_t mLidarDataEnabled = 0;
    uint8_t mBootState = 0;
    uint8_t mSystemOk = 0;
    uint8_t mLaserReady = 0;
    uint8_t mLaserArmed = 0;
    uint8_t mScanning = 0;
    uint8_t mAzimuthScanning = 0;
    uint8_t mElevationScanning = 0;
    uint8_t mDataStreaming = 0;

    DiscoveryVersion mDiscoveryVersion = DiscoveryVersion::Latest;

    // Default constructor
    DiscoveryResponse() {}

    // Robust constructor
    DiscoveryResponse( LegacyLumNet_DiscoveryResponse* response );
    DiscoveryResponse( LegacyLumNet_V2_DiscoveryResponse* response );
    DiscoveryResponse( LegacyLumNet_V4_DiscoveryResponse* response );
    DiscoveryResponse( LumNet_DiscoveryResponse* response );

    ~DiscoveryResponse() = default;
    DiscoveryResponse( const DiscoveryResponse& ) = default;
    DiscoveryResponse( DiscoveryResponse&& ) = default;
    DiscoveryResponse& operator=( const DiscoveryResponse& ) = default;
    DiscoveryResponse& operator=( DiscoveryResponse&& ) = default;

    //Returns a formatted string displaying the firmware semantic version
    std::string GetFirmwareString() const;

    //Returns a formatted string displaying the discovery semantic version
    std::string GetDiscoveryString() const;

    //Returns whether the response represented by other is from the same sensor
    bool SameSensor( const DiscoveryResponse& other ) const;

    bool SameSensor( const std::string& serialNumber ) const;

    //Returns true if sensor is running a LegacyV1 based firmware
    bool IsLegacyV1Sensor() const;

    /*
        Checks if system boot has completed. Available on discovery versions 0.5.0 or greater
        /returns True if boot has completed, false if sensor is still booting or discovery version is unsupported
    */
    bool BootComplete() const;

    /*
        Verify if boot was successful, only available on discovery versions 0.5.0 or greater
        /returns True if boot has completed and was successful. If sensor is still booting,
                 has reported a failure, or discovery version is unsupported will return false
    */
    bool BootSuccess() const;

    bool operator==( const DiscoveryResponse& other ) const;

    bool operator!=( const DiscoveryResponse& other ) const;

    bool operator<( const DiscoveryResponse& other ) const;

    bool operator>( const DiscoveryResponse& other ) const;

    bool operator>=( const DiscoveryResponse& other ) const;
};
}  // namespace lum
