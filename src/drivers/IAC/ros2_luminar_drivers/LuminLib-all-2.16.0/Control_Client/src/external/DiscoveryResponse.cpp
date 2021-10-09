/*
* DiscoveryResponse.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include <DiscoveryResponse.h>

#include <boost/asio.hpp>

using namespace lum;

DiscoveryResponse::DiscoveryResponse( LegacyLumNet_DiscoveryResponse* legacyRequest )
    : SensorUID( ( uint16_t )( legacyRequest->nickName.value ), ntohl( legacyRequest->networkConfig.networkConnection.ipAddress ) )  // Cast to 16 bit until FW is changed
{
    mDiscoveryVersion = DiscoveryVersion::LegacyV1;

    mDiscoveryMajor = legacyRequest->commonHeader.semVersionedHeader.semver.major;
    mDiscoveryMinor = legacyRequest->commonHeader.semVersionedHeader.semver.minor;
    mDiscoveryPatch = legacyRequest->commonHeader.semVersionedHeader.semver.patch;

    mFirmwareMajor = legacyRequest->buildVersion.major;
    mFirmwareMinor = legacyRequest->buildVersion.minor;
    mFirmwarePatch = legacyRequest->buildVersion.patch;

    mSerialNumber.assign( legacyRequest->serialNumber.value,
                          std::end( legacyRequest->serialNumber.value ) );
    mMacAddress.assign( legacyRequest->networkConfig.macAddress.value,
                        std::end( legacyRequest->networkConfig.macAddress.value ) );

    mLidarDataEnabled = legacyRequest->networkConfig.lidarDataEnable;
    mDestIPAddress = ntohl( legacyRequest->networkConfig.lidarDataEndPoint.ipAddress );
    mDestPort = legacyRequest->networkConfig.lidarDataEndPoint.port;
    mDscUDPPort = legacyRequest->networkConfig.dscUdpPort;
    mCmdTCPPort = legacyRequest->networkConfig.cmdTcpPort;
    mSubNetMask = ntohl( legacyRequest->networkConfig.networkConnection.subNetMask );
    mGateway = ntohl( legacyRequest->networkConfig.networkConnection.gateway );
    mMTU = legacyRequest->networkConfig.mtu;
    mNetworkType = legacyRequest->networkConfig.networkType;
    mSystemOk = legacyRequest->state.interLocks.systemOk;
    mLaserReady = legacyRequest->state.interLocks.laserReady;
    mScanning = legacyRequest->state.interLocks.scanning;
    mDataStreaming = legacyRequest->state.interLocks.dataStreaming;
    mLaserArmed = legacyRequest->state.interLocks.laserArmed;
}

DiscoveryResponse::DiscoveryResponse( LegacyLumNet_V2_DiscoveryResponse* legacyRequestV2 )
    : SensorUID( ( uint16_t )( legacyRequestV2->nickName.value ), ntohl( legacyRequestV2->networkConfig.networkConnection.ipAddress ) )
{
    mDiscoveryVersion = DiscoveryVersion::LegacyV2;

    mDiscoveryMajor = legacyRequestV2->commonHeader.semVersionedHeader.semver.major;
    mDiscoveryMinor = legacyRequestV2->commonHeader.semVersionedHeader.semver.minor;
    mDiscoveryPatch = legacyRequestV2->commonHeader.semVersionedHeader.semver.patch;

    mFirmwareMajor = legacyRequestV2->buildVersion.major;
    mFirmwareMinor = legacyRequestV2->buildVersion.minor;
    mFirmwarePatch = legacyRequestV2->buildVersion.patch;

    mSerialNumber.assign( legacyRequestV2->serialNumber.value,
                          std::end( legacyRequestV2->serialNumber.value ) );
    mMacAddress.assign( legacyRequestV2->networkConfig.macAddress.value,
                        std::end( legacyRequestV2->networkConfig.macAddress.value ) );

    mLidarDataEnabled = legacyRequestV2->networkConfig.lidarDataEnable;
    mDestIPAddress = ntohl( legacyRequestV2->networkConfig.lidarDataEndPoint.ipAddress );
    mDestPort = legacyRequestV2->networkConfig.lidarDataEndPoint.port;
    mDscUDPPort = legacyRequestV2->networkConfig.dscUdpPort;
    mCmdTCPPort = legacyRequestV2->networkConfig.cmdTcpPort;
    mSubNetMask = ntohl( legacyRequestV2->networkConfig.networkConnection.subNetMask );
    mGateway = ntohl( legacyRequestV2->networkConfig.networkConnection.gateway );
    mMTU = legacyRequestV2->networkConfig.mtu;
    mNetworkType = legacyRequestV2->networkConfig.networkType;
    mSystemOk = legacyRequestV2->state.interLocks.systemOk;
    mLaserReady = legacyRequestV2->state.interLocks.laserReady;
    mScanning = legacyRequestV2->state.interLocks.scanning;
    mDataStreaming = legacyRequestV2->state.interLocks.dataStreaming;
    mLaserArmed = legacyRequestV2->state.interLocks.laserArmed;
}

DiscoveryResponse::DiscoveryResponse( LegacyLumNet_V4_DiscoveryResponse* legacyRequestV4 )
    : SensorUID( legacyRequestV4->nickName, ntohl( legacyRequestV4->networkConfig.networkConnection.ipAddress ) )
{
    mDiscoveryVersion = DiscoveryVersion::LegacyV4;

    mDiscoveryMajor = legacyRequestV4->commonHeader.semVersionedHeader.semver.major;
    mDiscoveryMinor = legacyRequestV4->commonHeader.semVersionedHeader.semver.minor;
    mDiscoveryPatch = legacyRequestV4->commonHeader.semVersionedHeader.semver.patch;

    mFirmwareMajor = legacyRequestV4->buildVersion.major;
    mFirmwareMinor = legacyRequestV4->buildVersion.minor;
    mFirmwarePatch = legacyRequestV4->buildVersion.patch;

    mSerialNumber.assign( legacyRequestV4->serialNumber.value,
                          std::end( legacyRequestV4->serialNumber.value ) );
    mMacAddress.assign( legacyRequestV4->networkConfig.macAddress.value,
                        std::end( legacyRequestV4->networkConfig.macAddress.value ) );
    mHeadDescription.assign( legacyRequestV4->description.value,
                             std::end( legacyRequestV4->description.value ) );

    mLidarDataEnabled = legacyRequestV4->networkConfig.lidarDataEnable;
    mDestIPAddress = ntohl( legacyRequestV4->networkConfig.lidarDataEndPoint.ipAddress );
    mDestPort = legacyRequestV4->networkConfig.lidarDataEndPoint.port;
    mDscUDPPort = legacyRequestV4->networkConfig.dscUdpPort;
    mCmdTCPPort = legacyRequestV4->networkConfig.cmdTcpPort;
    mSubNetMask = ntohl( legacyRequestV4->networkConfig.networkConnection.subNetMask );
    mGateway = ntohl( legacyRequestV4->networkConfig.networkConnection.gateway );
    mMTU = legacyRequestV4->networkConfig.mtu;
    mNetworkType = legacyRequestV4->networkConfig.networkType;
    mSystemOk = legacyRequestV4->state.interLocks.systemOk;
    mLaserReady = legacyRequestV4->state.interLocks.laserReady;
    mScanning = legacyRequestV4->state.interLocks.scanning;
    mDataStreaming = legacyRequestV4->state.interLocks.dataStreaming;
    mLaserArmed = legacyRequestV4->state.interLocks.laserArmed;
}

DiscoveryResponse::DiscoveryResponse( LumNet_DiscoveryResponse* discoveryPacket )
    : SensorUID( discoveryPacket->nickName, ntohl( discoveryPacket->networkConfig.networkConnection.ipAddress ) )
{
    mDiscoveryVersion = DiscoveryVersion::Latest;

    mDiscoveryMajor = discoveryPacket->commonHeader.semVersionedHeader.semver.major;
    mDiscoveryMinor = discoveryPacket->commonHeader.semVersionedHeader.semver.minor;
    mDiscoveryPatch = discoveryPacket->commonHeader.semVersionedHeader.semver.patch;

    mFirmwareMajor = discoveryPacket->buildVersion.major;
    mFirmwareMinor = discoveryPacket->buildVersion.minor;
    mFirmwarePatch = discoveryPacket->buildVersion.patch;

    mSerialNumber.assign( discoveryPacket->serialNumber.value,
                          std::end( discoveryPacket->serialNumber.value ) );
    mMacAddress.assign( discoveryPacket->networkConfig.macAddress.value,
                        std::end( discoveryPacket->networkConfig.macAddress.value ) );
    mHeadDescription.assign( discoveryPacket->description.value,
                             std::end( discoveryPacket->description.value ) );

    mLidarDataEnabled = discoveryPacket->networkConfig.lidarDataEnable;
    mDestIPAddress = ntohl( discoveryPacket->networkConfig.lidarDataEndPoint.ipAddress );
    mDestPort = discoveryPacket->networkConfig.lidarDataEndPoint.port;
    mDscUDPPort = discoveryPacket->networkConfig.dscUdpPort;
    mCmdTCPPort = discoveryPacket->networkConfig.cmdTcpPort;
    mSubNetMask = ntohl( discoveryPacket->networkConfig.networkConnection.subNetMask );
    mGateway = ntohl( discoveryPacket->networkConfig.networkConnection.gateway );
    mMTU = discoveryPacket->networkConfig.mtu;
    mNetworkType = discoveryPacket->networkConfig.networkType;
    mBootState = discoveryPacket->state.bootState;
    mSystemOk = discoveryPacket->state.interLocks.systemOk;
    mLaserReady = discoveryPacket->state.interLocks.laserReady;
    mScanning = discoveryPacket->state.interLocks.azimuthScanning & discoveryPacket->state.interLocks.elevationScanning;
    mAzimuthScanning = discoveryPacket->state.interLocks.azimuthScanning;
    mElevationScanning = discoveryPacket->state.interLocks.elevationScanning;
    mDataStreaming = discoveryPacket->state.interLocks.dataStreaming;
    mLaserArmed = discoveryPacket->state.interLocks.laserArmed;
    mBootCompletionPercent = discoveryPacket->state.bootCompletionPercentage;
}

std::string DiscoveryResponse::GetFirmwareString() const
{
    return std::string( std::to_string( mFirmwareMajor ) + "." + std::to_string( mFirmwareMinor ) + "." + std::to_string( mFirmwarePatch ) );
}

std::string DiscoveryResponse::GetDiscoveryString() const
{
    return std::to_string( mDiscoveryMajor ) + "." + std::to_string( mDiscoveryMinor ) + "." + std::to_string( mDiscoveryPatch );
}

bool DiscoveryResponse::SameSensor( const DiscoveryResponse& other ) const
{
    return mMacAddress == other.mMacAddress;
}

bool DiscoveryResponse::SameSensor( const std::string& serialNumber ) const
{
    return mSerialNumber == serialNumber;
}

bool DiscoveryResponse::IsLegacyV1Sensor() const
{
    return mDiscoveryVersion == DiscoveryVersion::LegacyV1;
}

bool DiscoveryResponse::BootComplete() const
{
    return ( mDiscoveryVersion > DiscoveryVersion::LegacyV4 && mBootCompletionPercent == 100.0f ) ? true : false;
}

bool DiscoveryResponse::BootSuccess() const
{
    return ( mDiscoveryVersion > DiscoveryVersion::LegacyV4 && mBootCompletionPercent == 100.0f && mBootState == LUM_NET_BOOT_STATE_SUCCESS ) ? true : false;
}

bool DiscoveryResponse::operator==( const DiscoveryResponse& other ) const
{
    return SensorUID::operator==( other ) &&
           mSerialNumber == other.mSerialNumber &&
           mHeadDescription == other.mHeadDescription &&
           mMacAddress == other.mMacAddress &&
           mBootCompletionPercent == other.mBootCompletionPercent &&
           mDestIPAddress == other.mDestIPAddress &&
           mSubNetMask == other.mSubNetMask &&
           mGateway == other.mGateway &&
           mDestPort == other.mDestPort &&
           mDscUDPPort == other.mDscUDPPort &&
           mCmdTCPPort == other.mCmdTCPPort &&
           mMTU == other.mMTU &&
           mNetworkType == other.mNetworkType &&
           mDiscoveryMajor == other.mDiscoveryMajor &&
           mDiscoveryMinor == other.mDiscoveryMinor &&
           mDiscoveryPatch == other.mDiscoveryPatch &&
           mPayloadID == other.mPayloadID &&
           mFirmwareMajor == other.mFirmwareMajor &&
           mFirmwareMinor == other.mFirmwareMinor &&
           mFirmwarePatch == other.mFirmwarePatch &&
           mLidarDataEnabled == other.mLidarDataEnabled &&
           mBootState == other.mBootState &&
           mSystemOk == other.mSystemOk &&
           mLaserReady == other.mLaserReady &&
           mLaserArmed == other.mLaserArmed &&
           mScanning == other.mScanning &&
           mAzimuthScanning == other.mAzimuthScanning &&
           mElevationScanning == other.mElevationScanning &&
           mDataStreaming == other.mDataStreaming &&
           mDiscoveryVersion == other.mDiscoveryVersion;
}

bool DiscoveryResponse::operator!=( const DiscoveryResponse& other ) const
{
    return !( *this == other );
}

bool DiscoveryResponse::operator<( const DiscoveryResponse& other ) const
{
    return mDiscoveryVersion < other.mDiscoveryVersion;
}

bool DiscoveryResponse::operator>( const DiscoveryResponse& other ) const
{
    return mDiscoveryVersion > other.mDiscoveryVersion;
}

bool DiscoveryResponse::operator>=( const DiscoveryResponse& other ) const
{
    return mDiscoveryVersion >= other.mDiscoveryVersion;
}
