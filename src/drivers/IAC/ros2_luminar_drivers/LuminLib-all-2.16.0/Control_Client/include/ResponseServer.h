/*
* ResponseServer.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ResponseServer
 *         The `ResponseServer` manages UDP connections to the sensor, especially as it relates to `Discovery`.
 *
 */

#pragma once

#include <stdint.h>
#include <memory>

namespace lum
{
class ResponseServer
{
public:
    ResponseServer( uint16_t port );
    virtual ~ResponseServer();

    // Sets up the async receiver and starts the points thread
    void Start();

    // Stops and closes all the open networking code. Stops all threads started by this class.
    void Stop();

private:
    class SocketImpl;
    std::unique_ptr<SocketImpl> mpSocketImpl;
};
}  // namespace lum
