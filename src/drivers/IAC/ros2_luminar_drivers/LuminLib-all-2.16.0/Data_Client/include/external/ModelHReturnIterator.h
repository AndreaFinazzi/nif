/*
* ModelHReturnIterator.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHReturnIterator
 *         Provides a wrapper around returned LiDAR data
 *
 *  The ModelHReturnIterator provides a simple interface for traversing all returns in a ModelHDataPacket
 *  To traverse a packet, check if the iterator has reached the end, access the return information, and then advance the iterator
 */

#pragma once

#include <stdint.h>
#include <cstddef>

#include "LumNet/LidarData.h"

class ModelHReturnIterator
{
public:
    ModelHReturnIterator( uint8_t* ptr, size_t pktLen );

    // Check if the iterator has reached the end of the ray data
    bool End() const;

    // Advance the iterator to the next return
    void NextReturn();

    // Reset the iterator to the start of the ray data
    void Reset();

    // Check if the current ray has a valid return
    bool ReturnValid() const;

    // Get the Azimuth of the return in radians
    float Azimuth() const;

    // Get the Elevation of the return in radians
    float Elevation() const;

    // Get the timestamp of the return in microseconds since the internal clock was at the top of the second.
    uint32_t MicrosecondTimestamp() const;

    // Range measurement of the return in meters
    float Range() const;

    // Reflectance measurement
    float Reflectance() const;

    // Get scan segment index of the current return
    uint8_t ScanSegmentIndex() const;

    // Get the eye of the current return
    uint8_t Eye() const;

    // Index of the ray currently referenced
    int RayIndex() const;

private:
    // Check if the ReturnPtr is in fact pointing at the start of the next ray
    bool IsRay() const;

    LumNet_Ray* GetCurrentRay() const;
    LumNet_RayReturn* GetCurrentReturn() const;

    uint8_t* mpRayDataStart;
    uint8_t* mpRayDataEnd;

    uint8_t* mpRayPtr;
    uint8_t* mpReturnPtr;

    int mRayIndex = 0;
};
