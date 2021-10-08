/*
* ModelHDataPacketParser.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHDataPacketParser
 *         Parses LiDAR packets received from the sensor
 *
 *  The ModelHDataParser is constructed with a pointer to the packet data.
 *  It performs a memcpy of the data from the source buffer, and then provides accessor
 *  functions for parsed packet data.
 *  The object is fixed size for direct array allocation or pooling.
 */

#pragma once

#include <cstring>

#include "LumNet/LidarData.h"
#include "ModelHReturnIterator.h"

class ModelHDataPacketParser
{
public:
    ModelHDataPacketParser( const void* pData, size_t dataSize );
    virtual ~ModelHDataPacketParser();
    /*
    * Get the byte length of packet data
    */
    uint16_t GetPacketLength() const;

    /*
    * Data packets should always be of type LUM_NET_PAYLOAD_TYPE_ID_LIDAR_DATA
    */
    uint8_t GetPacketType() const;

    /*
    * Get the number of rays found in the packet
    */
    uint8_t GetNumRays() const;

    /*
    * Get the ID of the packet from the laser head
    */
    uint8_t GetSequenceNumber() const;

    /*
    * Get the fingerprint of the head delivering this packet
    * Note: this will change from the head's port to an actual unique value in future releases
    */
    uint16_t GetFingerPrint() const;

    /*
    * Get the timestamp of the packet recorded from the head
    */
    uint32_t GetUnixTimeStamp() const;

    /*
    * Get the id of the current scan profile
    */
    uint8_t GetScanProfile() const;

    /*
    * Get the most recently seen scan profile checkpoint
    * Note : the meaning of checkpoints varies by scan profile
    */
    uint8_t GetMostRecentCheckpointLegacy() const;

    /*
    * Get the current track number
    * Track number is the index of the currently playing Track
    */
    uint8_t GetTrackNumber() const;

    /*
    * Get the most recent checkpoint
    * `Track Checkpoint` is the checkpoint for the current region from the current "track"
    */
    uint8_t GetMostRecentCheckpoint() const;

    /*
    * Get the current scan count
    * The scan count increments each time the selected scan profile completes
    */
    uint8_t GetLegacyScanCount() const;

    /*
    * Get the current scan count
    * The scan count increments each time the selected scan profile completes
    */
    uint8_t GetScanCount() const;

    /*
    * Get the current Interlace Index
    * InterLace Index is reset to 0 when all interlace scans have completed, then the scan count is updated
    */
    uint8_t GetInterlaceIndex() const;

    /*
    *Get an iterator to walk each return contained in the packet
    */
    ModelHReturnIterator& GetReturnIterator();

private:
    /*
    * Definition to allow for reinterpret overlay
    * This is defined as a private member because the firstRayHeader is a
    * potential landmine.  The array is not valid to index past the first
    * member because rays are variable-size
    */
    struct LidarDataPacket
    {
        LumNet_LidarDataPreamble preamble;
        LumNet_RayHeader firstRayHeader;
    };

    /*
    * Get a reinterpret-cast overlay of the packet data
    */
    const LidarDataPacket* GetPacket() const;

    /*
    * Packet parsers contain a buffer large enough for a single packet
    */
    uint8_t mpPacketData[LUM_NET_MAX_LIDAR_DATA_PAYLOAD_SIZE];

    /*
    * Data length is set during construction when packet data is copied to
    * internal buffer
    */
    int mDataLength;

    /*
    * Packet parsers include return iterator so their use is allocation free
    */
    ModelHReturnIterator mReturnIterator;
};
