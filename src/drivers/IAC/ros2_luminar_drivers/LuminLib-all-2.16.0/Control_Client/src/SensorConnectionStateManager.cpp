#include "SensorConnectionStateManager.h"
#include <LumNet/Command.h>
#include <cstring>
#include <iostream>
#include "ModelHController.h"

using namespace lum;

SensorConnectionStateManager::SensorConnectionStateManager()
{
}

void SensorConnectionStateManager::ProcessBytes( const uint8_t* const pBytesToProcess, const uint32_t numBytesToProcess, const uint16_t sensorFingerprint )
{
    mSensorFingerprint = sensorFingerprint;
    mpCurrentPacket = pBytesToProcess;
    mNumBytesRemainingInCurrentPacket = numBytesToProcess;
    while ( mNumBytesRemainingInCurrentPacket > 0 )
    {
        switch ( mConnectionState )
        {
        case lum::CommandRequestListConnectionState::WaitingForSentinel:
        {
            HandleWaitingForSentinelState();
            break;
        }
        case lum::CommandRequestListConnectionState::WaitingForPayloadType:
        {
            HandleWaitingForPayloadTypeState();
            break;
        }
        case lum::CommandRequestListConnectionState::WaitingForPayloadSize:
        {
            HandleWaitingForPayloadSizeState();
            break;
        }
        case lum::CommandRequestListConnectionState::WaitingForPayload:
        {
            HandleWaitingForPayloadState();
            break;
        }
        default:
        {
            std::cout << "Error: Invalid Sensor Command Request List State on sensor with fingerprint: " << mSensorFingerprint << "\n";
            break;
        }
        }
    }
}
bool SensorConnectionStateManager::FindSentinel()
{
    if ( mNumBytesRemainingInCurrentPacket < sizeof( LumNet_Sentinel ) )
    {
        return false;
    }

    while ( mNumBytesRemainingInCurrentPacket >= sizeof( LumNet_Sentinel ) )
    {
        const LumNet_Sentinel& possibleSentinel = *( reinterpret_cast<const LumNet_Sentinel*>( mpCurrentPacket ) );

        if ( possibleSentinel == LUM_NET_SENTINEL )
        {
            return true;
        }
        mpCurrentPacket++;
        mNumBytesRemainingInCurrentPacket--;
    }

    return false;
}
bool SensorConnectionStateManager::FindRemainingPayloadBytes( const bool setPayloadType )
{
    if ( mNumBytesRemainingInCurrentPacket < mCurrentPayloadSize - mCurrentPayloadBytesAccumulated )
    {
        memcpy( mCommandRequestBuffer + mBufferIndex, mpCurrentPacket, mNumBytesRemainingInCurrentPacket );
        mBufferIndex += mNumBytesRemainingInCurrentPacket;
        mCurrentPayloadBytesAccumulated += mNumBytesRemainingInCurrentPacket;
        mpCurrentPacket += mNumBytesRemainingInCurrentPacket;
        return false;
    }
    else
    {
        uint32_t bytesUntilEndOfPayload = mCurrentPayloadSize - mCurrentPayloadBytesAccumulated;
        memcpy( mCommandRequestBuffer + mBufferIndex, mpCurrentPacket, bytesUntilEndOfPayload );
        mBufferIndex += bytesUntilEndOfPayload;
        if ( setPayloadType )
        {
            mCurrentPayloadType = static_cast<LumNet_PayloadTypeIDs>( mCommandRequestBuffer[mBufferIndex - 1] );
        }
        mCurrentPayloadBytesAccumulated += bytesUntilEndOfPayload;
        mpCurrentPacket += bytesUntilEndOfPayload;
        return true;
    }
}
bool SensorConnectionStateManager::FindPayloadType()
{
    return FindRemainingPayloadBytes( true );
}
bool SensorConnectionStateManager::FindPayloadSize()
{
    return FindRemainingPayloadBytes();
}
bool SensorConnectionStateManager::FindPayload()
{
    return FindRemainingPayloadBytes();
}
void SensorConnectionStateManager::HandleWaitingForPayloadState()
{
    const uint8_t* const pPreviousPacketPosition = mpCurrentPacket;
    if ( FindPayload() )
    {
        ModelHController::get().DispatchCommandResponse( mSensorFingerprint, mCommandRequestBuffer );
        mBufferIndex = 0;
        mConnectionState = lum::CommandRequestListConnectionState::WaitingForSentinel;
        mCurrentPayloadBytesAccumulated = 0;
        mCurrentPayloadSize = 0;
    }
    mNumBytesRemainingInCurrentPacket -= static_cast<uint16_t>( ( mpCurrentPacket - pPreviousPacketPosition ) );
    ;
}

void SensorConnectionStateManager::HandleWaitingForPayloadSizeState()
{
    const uint8_t* const pPreviousPacketPosition = mpCurrentPacket;
    if ( FindPayloadSize() )
    {
        mCurrentPayloadBytesAccumulated = 0;
        mConnectionState = lum::CommandRequestListConnectionState::WaitingForPayload;
        LumNet_CommandResponseListPayload& commandResponseListPayload = *( reinterpret_cast<LumNet_CommandResponseListPayload*>( mCommandRequestBuffer ) );
        mCurrentPayloadSize = commandResponseListPayload.responseListHeader.listPayloadLength;
    }
    mNumBytesRemainingInCurrentPacket -= static_cast<uint16_t>( ( mpCurrentPacket - pPreviousPacketPosition ) );
    ;
}

void SensorConnectionStateManager::HandleWaitingForPayloadTypeState()
{
    const uint8_t* const pPreviousPacketPosition = mpCurrentPacket;
    if ( FindPayloadType() )
    {
        if ( mCurrentPayloadType == LumNet_PayloadTypeIDs::LUM_NET_PAYLOAD_TYPE_ID_CMD_REQUEST_LIST_RETURN_RESULT )
        {
            mConnectionState = lum::CommandRequestListConnectionState::WaitingForPayload;
            mCurrentPayloadBytesAccumulated = 0;
            mCurrentPayloadSize = sizeof( LumNet_CommandRequestListValidationPayload ) - mBufferIndex;
        }
        else if ( mCurrentPayloadType == LumNet_PayloadTypeIDs::LUM_NET_PAYLOAD_TYPE_ID_CMD_RESPONSE_LIST )
        {
            mConnectionState = lum::CommandRequestListConnectionState::WaitingForPayloadSize;
            mCurrentPayloadBytesAccumulated = 0;

            mCurrentPayloadSize = offsetof( LumNet_CommandResponseListPayload, commands ) - mBufferIndex;
        }
        else
        {
            std::cout << "Error: Invalid paylaod type for TCP command from sensor with fingerprint: " << mSensorFingerprint << "\n";
            mBufferIndex = 0;
            mConnectionState = lum::CommandRequestListConnectionState::WaitingForSentinel;
            mCurrentPayloadBytesAccumulated = 0;
            mCurrentPayloadSize = 0;
        }
    }
    mNumBytesRemainingInCurrentPacket -= static_cast<uint16_t>( ( mpCurrentPacket - pPreviousPacketPosition ) );
    ;
}

void SensorConnectionStateManager::HandleWaitingForSentinelState()
{
    const uint8_t* const pPreviousPacketPosition = mpCurrentPacket;
    if ( FindSentinel() )
    {
        memcpy( mCommandRequestBuffer, mpCurrentPacket, sizeof( LumNet_Sentinel ) );
        mConnectionState = lum::CommandRequestListConnectionState::WaitingForPayloadType;
        mCurrentPayloadBytesAccumulated = 0;
        mCurrentPayloadSize = PAYLOAD_TYPE_OFFSET;
        mBufferIndex = sizeof( LumNet_Sentinel );
        auto bytesProcessed = static_cast<uint16_t>( sizeof( LumNet_Sentinel ) );
        mNumBytesRemainingInCurrentPacket -= bytesProcessed;
        mpCurrentPacket += sizeof( LumNet_Sentinel );
    }
    else
    {
        mNumBytesRemainingInCurrentPacket -= mNumBytesRemainingInCurrentPacket;
        mpCurrentPacket = pPreviousPacketPosition + mNumBytesRemainingInCurrentPacket;
    }
}
