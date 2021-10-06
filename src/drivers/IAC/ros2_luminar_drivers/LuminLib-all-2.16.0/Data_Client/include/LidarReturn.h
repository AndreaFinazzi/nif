/*
* LidarReturn.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*
* Lidar Return a configurable representation of a measured return along a sampled ray.
* Each sampled ray can result in 0 or more measured returns, each of which will be represented as a LidarReturn.
* LidarReturns are currently configurable in that they can contain the raw angular measurements taken by the sensor,
* or they can contain the converted Cartesian points.
*/

#pragma once

#include <stdint.h>

#include "luminlib_export.h"

namespace lum
{
static uint8_t EMPTY_RAY_INDEX = 15;
static uint8_t NUM_EYE_BITS = 1;
static uint8_t NUM_RETURN_BITS = 4;

enum EyeIndex
{
    LEFT_EYE_INDEX = 0,
    RIGHT_EYE_INDEX = 1
};

// Dual purpose point which can represent either a polar or a Cartesian point depending on transformations requested
struct LUMINLIB_EXPORT LidarReturn
{
    LidarReturn()
        : x( 0.f ), y( 0.f ), z( 0.f ), r( 0.f ), unix_timestamp( 0 ), microsecond_timestamp( 0 ), device_fingerprint( 0 ), return_index( 0 ), packet_sequence_number( 0 ), scan_profile( 0xFF ), scan_count( 0xFF ), checkpoint( 0xFF ), scan_segment_index( 0xFF )
    {
    }

    LidarReturn(
        float x,
        float y,
        float z,
        float r,
        uint32_t unix_timestamp = 0,
        uint32_t microsecond_timestamp = 0,
        uint16_t device_fingerprint = 0,
        uint8_t return_index = 0,
        uint8_t packet_sequence_number = 0,
        uint8_t scan_profile = 0xFF,
        uint8_t scan_count = 0xFF,
        uint8_t checkpoint = 0xFF,
        uint8_t scan_segment_index = 0xFF )
        : x( x ), y( y ), z( z ), r( r ), unix_timestamp( unix_timestamp ), microsecond_timestamp( microsecond_timestamp ), device_fingerprint( device_fingerprint ), return_index( return_index ), packet_sequence_number( packet_sequence_number ), scan_profile( scan_profile ), scan_count( scan_count ), checkpoint( checkpoint ), scan_segment_index( scan_segment_index )
    {
    }

    union
    {
        struct  // Cartesian
        {
            float x;
            float y;
            float z;
            float r;
        };
        struct  // Polar
        {
            float azimuth;
            float elevation;
            float range;
            float reflectance;
        };
    };

    uint32_t unix_timestamp;
    uint32_t microsecond_timestamp;

    uint16_t device_fingerprint;
    uint8_t return_index;  // Set to EMPTY_RAY_INDEX if ray is empty (no range returns)
    uint8_t packet_sequence_number;

    uint8_t scan_profile;
    uint8_t scan_count;  // represents the scan_counters field which is composed of two sub-fields (scan_count and interlace_index)
    uint8_t checkpoint;  // represents the checkpoint field which is composed of two sub-fields (track_number and track_checkpoint)
    uint8_t scan_segment_index;
};

/*
    * Extract the Scan Segment Index of a LidarReturn without the Eye Index
    */
static inline const uint8_t LUMINLIB_EXPORT ExtractScanSegmentIndex( const LidarReturn& lidarReturn )
{
    return lidarReturn.scan_segment_index >> NUM_EYE_BITS;
}

/*
    * Extract the Eye Index of a LidarReturn from the Scan Segment Index
    *
    * Cast from a bigger int to avoid undefined behavior if NUM_EYE_BITS
    * is equal to the number of bits in the word
    */
static inline const uint8_t LUMINLIB_EXPORT ExtractEyeIndex( const LidarReturn& lidarReturn )
{
    uint8_t eye_mask = ~( ( ~(uint16_t)0 ) << NUM_EYE_BITS );
    return lidarReturn.scan_segment_index & eye_mask;
}

/*
    * Extract the Return Index Bits of a LidarReturn from the Return Index
    *
    * Cast from a bigger int to avoid undefined behavior if NUM_RETURN_BITS
    * is equal to the number of bits in the word
    */
static inline const uint8_t LUMINLIB_EXPORT ExtractReturnIndex( const LidarReturn& lidarReturn )
{
    uint8_t return_mask = ~( ( ~(uint16_t)0 ) << NUM_RETURN_BITS );
    return lidarReturn.return_index & return_mask;
}

/*
    * Extract the Debug Bits of a LidarReturn from the Return Index
    */
static inline const uint8_t LUMINLIB_EXPORT ExtractDebugBits( const LidarReturn& lidarReturn )
{
    return lidarReturn.return_index >> NUM_RETURN_BITS;
}
}  // namespace lum
