/*
* SensorSocketService.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief SensorSocketService
 *         The sensor socket service manages TCP connections to the LiDAR sensor.
 *
 *  To initiate a connection a sensor's fingerprint, IP address, and port must be provided.
 *  This information is obtained from the `Discovery` response, see `DiscoveryResponse.h` and the
 *  `ModelHController`.
 *
 *  After a connection is established, command buffers can be sent via `SendCommand`. See `CommandConstructors.h`
 *  for information on how to create command buffers and `ModelHController` for usage of this service.
 */

#pragma once

#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include <stdint.h>
#include "SensorConnectionStateManager.h"

namespace lum
{
class SensorConnectionSubscriber;

class SensorSocketService
{
public:
    SensorSocketService();
    ~SensorSocketService();

    void AddSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber );
    bool RemoveSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber );

    void ConnectToDevice( uint16_t fingerprint, uint32_t ipAddress, uint16_t tcpPort );
    void CloseDeviceConnection( uint16_t fingerprint );

    void SendCommand( uint16_t fingerprint, uint8_t* data, size_t dataSize );
    void UpdateDeviceFingerprint( uint16_t prevFingerprint, uint16_t newFingerprint );

private:
    class SocketImpl;
    std::unique_ptr<SocketImpl> mpSocketImpl;
};
}  // namespace lum
