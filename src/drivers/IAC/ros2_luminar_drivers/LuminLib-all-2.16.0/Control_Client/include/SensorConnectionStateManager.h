/*
* SensorConnectionStateManager.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief SensorConnectionStateManager
 *     Manages the TCP connections with the Sensors to maintain TCP framing and connection with multiple senors at once.
 */

#pragma once
#include <LumNet/Common/PayloadTypeID.h>
#include <stdint.h>

namespace lum
{
enum CommandRequestListConnectionState
{
    WaitingForSentinel,
    WaitingForPayloadType,
    WaitingForPayloadSize,
    WaitingForPayload
};

class SensorConnectionStateManager
{
public:
    SensorConnectionStateManager();
    void ProcessBytes( const uint8_t* const pBytesToProcess, const uint32_t numBytesToProcess, const uint16_t sensorFingerprint );

private:
    bool FindSentinel();
    bool FindRemainingPayloadBytes( const bool setPayloadType = false );
    bool FindPayloadType();
    bool FindPayloadSize();
    bool FindPayload();
    void HandleWaitingForPayloadState();
    void HandleWaitingForPayloadSizeState();
    void HandleWaitingForPayloadTypeState();
    void HandleWaitingForSentinelState();

    static constexpr auto MAX_PAYLOAD_SIZE = 8192;
    static constexpr auto PAYLOAD_TYPE_OFFSET = 1;

    lum::CommandRequestListConnectionState mConnectionState = lum::CommandRequestListConnectionState::WaitingForSentinel;
    LumNet_PayloadTypeIDs mCurrentPayloadType = LumNet_PayloadTypeIDs::LUM_NET_PAYLOAD_TYPE_ID_DSC_NETWORK_CONNECTION_ASSIGNMENT;  //Invalid
    uint32_t mCurrentPayloadSize = 0;
    uint32_t mCurrentPayloadBytesAccumulated = 0;
    uint32_t mBufferIndex = 0;
    uint8_t mCommandRequestBuffer[MAX_PAYLOAD_SIZE];
    uint16_t mSensorFingerprint = 0;
    const uint8_t* mpCurrentPacket = nullptr;
    uint16_t mNumBytesRemainingInCurrentPacket = 0;
};
}  // namespace lum