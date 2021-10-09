/*
* ModelHDataPacketParser.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ModelHDataPacketParser.h"

#include <assert.h>

ModelHDataPacketParser::ModelHDataPacketParser( const void* pData, size_t dataSize )
    : mDataLength( static_cast<int>( dataSize ) )
    , mReturnIterator( mpPacketData + sizeof( LumNet_LidarDataPreamble ), dataSize - sizeof( LumNet_LidarDataPreamble ) )
{
    assert( dataSize <= LUM_NET_MAX_LIDAR_DATA_PAYLOAD_SIZE );
    std::memcpy( mpPacketData, pData, dataSize );
}

ModelHDataPacketParser::~ModelHDataPacketParser()
{
}

uint16_t ModelHDataPacketParser::GetPacketLength() const
{
    return mDataLength;
}

uint8_t ModelHDataPacketParser::GetPacketType() const
{
    return GetPacket()->preamble.semVersionedHeader.payloadTypeID;
}

uint8_t ModelHDataPacketParser::GetNumRays() const
{
    return GetPacket()->preamble.num_rays;
}

uint8_t ModelHDataPacketParser::GetSequenceNumber() const
{
    return GetPacket()->preamble.sequence_number;
}

uint16_t ModelHDataPacketParser::GetFingerPrint() const
{
    return GetPacket()->preamble.fingerprint;
}

uint32_t ModelHDataPacketParser::GetUnixTimeStamp() const
{
    return GetPacket()->preamble.timestamp_seconds;
}

uint8_t ModelHDataPacketParser::GetScanProfile() const
{
    return GetPacket()->preamble.scan_profile;
}

uint8_t ModelHDataPacketParser::GetMostRecentCheckpointLegacy() const
{
    return GetPacket()->preamble.checkpoint;
}

uint8_t ModelHDataPacketParser::GetTrackNumber() const
{
    return LumNet_RayHeader_GetTrackNumber( GetPacket()->preamble.checkpoint );
}

uint8_t ModelHDataPacketParser::GetMostRecentCheckpoint() const
{
    return LumNet_RayHeader_GetTrackCheckpoint( GetPacket()->preamble.checkpoint );
}

uint8_t ModelHDataPacketParser::GetLegacyScanCount() const
{
    return GetPacket()->preamble.scan_counters;
}

uint8_t ModelHDataPacketParser::GetScanCount() const
{
    return LumNet_RayHeader_GetScanCount( GetPacket()->preamble.scan_counters );
}

uint8_t ModelHDataPacketParser::GetInterlaceIndex() const
{
    return LumNet_RayHeader_GetInterlaceIndex( GetPacket()->preamble.scan_counters );
}

ModelHReturnIterator& ModelHDataPacketParser::GetReturnIterator()
{
    return mReturnIterator;
}

const ModelHDataPacketParser::LidarDataPacket* ModelHDataPacketParser::GetPacket() const
{
    return reinterpret_cast<const LidarDataPacket*>( mpPacketData );
}