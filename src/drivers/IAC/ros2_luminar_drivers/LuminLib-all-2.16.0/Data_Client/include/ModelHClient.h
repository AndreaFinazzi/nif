/*
* ModelHClient.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHClient
 *         Manages receiving LiDAR data from the sensor
 *
 */

#pragma once

#include <stdint.h>
#include <memory>

#include "luminlib_export.h"

namespace lum
{
class LUMINLIB_EXPORT ModelHClient
{
public:
    /*
     * The ModelHClient class owns the thread that is receiving the network traffic from each device.
     * Right now the clients are configured based off each device's port, in the future they'll each
     * have their own unique fingerprint.
     */
    ModelHClient( uint16_t port );
    ~ModelHClient();

    // Sets up the async receiver and starts the points thread
    void Start();

    // Stops and closes all the open networking code. Stops all threads started by this class.
    void Stop();

private:
    class SocketImpl;
    std::unique_ptr<SocketImpl> mpSocketImpl;
};
}  // namespace lum
