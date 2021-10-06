/*
 * SensorConnectionSubscriber.h
 *
 * Copyright (c) 2019, Luminar Technologies, Inc.
 *
 * This material contains confidential and trade secret information of Luminar Technologies.
 * Reproduction, adaptation, and distribution are prohibited, except to the extent expressly
 * permitted in writing by Luminar Technologies.
 */

/*! \brief SensorSocketSubscriber
 *         The sensor connection subscriber is the interface that can be used to get status updates
 * on the TCP socket connections to discovered LiDAR sensors.
 */

#pragma once

#include "DiscoveryResponse.h"
#include "luminlib_export.h"

namespace lum
{
class LUMINLIB_EXPORT SensorConnectionSubscriber
{
public:
    /*
     * Handler that is called when a TCP socket connection is established with a sensor.
     */
    virtual void SensorConnected( uint16_t fingerprint ) {}
};
}  // namespace lum
