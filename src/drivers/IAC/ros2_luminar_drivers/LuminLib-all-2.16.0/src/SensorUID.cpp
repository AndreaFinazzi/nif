/*
* SensorUID.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "SensorUID.h"

#include <iomanip>
#include <iostream>
#include <sstream>

#include <boost/asio.hpp>

namespace lum
{
SensorUID::SensorUID()
    : mFingerprint( 0 )
    , mHeadIPAddress( 0 )
{
}

SensorUID::SensorUID( uint16_t fingerprint, uint32_t headIPAddress )
    : mFingerprint( fingerprint )
    , mHeadIPAddress( headIPAddress )
{
}

/*
    * Generate a unique string identifier for the LumSensor
    */
const std::string SensorUID::StringIdentifier() const
{
    return std::to_string( mFingerprint ) + "-" + boost::asio::ip::address_v4( mHeadIPAddress ).to_string();
}

bool SensorUID::operator<( const SensorUID& sensor ) const
{
    return std::tie( mFingerprint, mHeadIPAddress ) < std::tie( sensor.mFingerprint, sensor.mHeadIPAddress );
}

bool SensorUID::operator==( const SensorUID& other ) const
{
    return mFingerprint == other.mFingerprint &&
           mHeadIPAddress == other.mHeadIPAddress;
}

bool SensorUID::operator!=( const SensorUID& other ) const
{
    return !( *this == other );
}
}  // namespace lum
