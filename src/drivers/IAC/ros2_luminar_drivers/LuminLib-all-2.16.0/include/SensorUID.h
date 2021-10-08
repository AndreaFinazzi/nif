/*
* SensorUID.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#pragma once

#include <stdint.h>
#include <string>

#include "luminlib_export.h"

namespace lum
{
// Properties of a sensor unique identifier
class LUMINLIB_EXPORT SensorUID
{
public:
    SensorUID();
    SensorUID( uint16_t fingerprint, uint32_t headIPAddress );

    /*
     * Generate a string identifier for the SensorUID
     */
    const std::string StringIdentifier() const;

    bool operator<( const SensorUID& sensor ) const;

    bool operator==( const SensorUID& other ) const;

    bool operator!=( const SensorUID& other ) const;

    uint16_t mFingerprint = 0;
    uint32_t mHeadIPAddress = 0;
};
}  // namespace lum
