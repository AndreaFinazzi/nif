/*
* ModelHController.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHController
 *         The `ModelHController` ties together SDK network and command and control functionality.
 *
 *  This class manages UDP and TCP connections to the sensor and the sending of commands.
 *
 */

#pragma once

#include "DiscoveryResponse.h"
#include "ModelHControllerCommands.h"
#include "ResponseServer.h"
#include "luminlib_export.h"

#include <atomic>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "SensorSocketService.h"

#include "CommandLumNetDefinitions.h"

namespace lum
{
class ModelHCommandClientSubscriber;
class SensorConnectionSubscriber;

class LUMINLIB_EXPORT ModelHController : public ModelHControllerCommands
{
public:
    //Returns the current instance of ModelHController
    static ModelHController& get();

    //Starts or resets the Response Server
    void Initialize();

    void AddCommandResponseSubscriber( ModelHCommandClientSubscriber* subscriber );
    bool RemoveCommandResponseSubscriber( ModelHCommandClientSubscriber* subscriber );

    // Part of the `ModelHControllerCommands` interface we need to implement
    TransactionID NextTransactionId();
    bool SendCommand( DiscoveryResponse commandedSensor, uint8_t* command, size_t command_size );
    LegacyCommandProtocolSemVer GetCurrentSemVer();

    // Sets the Command Protocol SemVer
    void SetCurrentSemVer( LegacyCommandProtocolSemVer version );

    void DispatchCommandResponse( FingerprintID fingerprint, uint8_t* response );

    /*
     * Starts the Response Server thread. Response server is responsible for handling all UDP responses
     * from the sensor heads
     */
    void StartResponseServer();

    //Stops the Response Server thread.
    void StopResponseServer();

    /**
        Discovers sensors by broadcasting LumNet_DiscoveryRequests. Note
        this function clears the existing DiscoveryResponseList
        \param  port    (Optional) The port to broadcast LumNet_DiscoveryRequests
        on. Defaults to LUM_NET_DEFAULT_UDP_DISCOVERY_PORT
     */
    void DiscoverSensorHeads( uint16_t port = LUM_NET_DEFAULT_UDP_DISCOVERY_PORT );

    //Broadcasts a recovery packet in an attempt to recover sensors that have been configured incorrectly.
    void RecoverSensorHead( std::string serial_number, bool override, uint16_t port );

    /*
     * Adds a sensor connection subscriber.
     */
    void AddSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber );

    /*
     * Removes a sensor connection subscriber; returns whether the subscriber was successfully
     * removed.
     */
    bool RemoveSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber );

    //Connects to a new specified sensor
    void ConnectToSensor( const DiscoveryResponse& sensor );

    //Closes the device connection
    void RemoveSensorConnectDestination( const DiscoveryResponse& sensor );

    //Update sensor management when discoveryresponse changes
    void UpdateSensorConnection( const DiscoveryResponse& prevResponse, const DiscoveryResponse& newResponse );

    //Responsible for parsing all responses from Response Server
    void ReceivedResponse( char* buffer, size_t bytes_received );

    /**
        Adds the passed DiscoveryResponse to the DiscoveryResponseList based
        on newness (version) of the DiscoveryResponse.
        \param  res     The DiscoveryResponse to add.
     */
    void UpdateDiscoveryResponseList( const DiscoveryResponse& res );

    //Returns the Discovery Response List
    std::vector<DiscoveryResponse> GetDiscoveryList();

    //Sets the Persistent Network Info of the sensor head that matches the @param SensorConnectInfo commandedSensor
    uint16_t SetPersistentNetworkInfo( DiscoveryResponse commandedSensor, uint32_t newDestIp, uint16_t newDestport, int enable, uint32_t newSensorIp, LumNet_Nickname nickname );

    //Sets the Volatile Network Info of the sensor head that matches the @param SensorConnectInfo commandedSensor
    uint16_t SetVolatileNetworkInfo( DiscoveryResponse commandedSensor, uint32_t newDestIp, uint16_t newDestport, int enable, LumNet_Nickname nickname );

    //Returns a list of strings representing network endpoints with the machine hostname
    const std::vector<std::string> GetHostnamesIpv4() const;

    //Broadcasts specified request to all network interfaces on machine
    void BroadcastRequest( void* request, size_t requestSizeBytes, uint16_t port );

private:
    ModelHController();
    ~ModelHController();
    // Prevent accidental copy/assignment of this singleton
    ModelHController( const ModelHController& ) = delete;
    ModelHController& operator=( const ModelHController& ) = delete;

    std::unique_ptr<ResponseServer> mpResponseServer;
    std::thread mResponseThread;
    std::vector<DiscoveryResponse> mDiscoveryResponseList;
    std::mutex mDiscoveryMutex;

    SensorSocketService mSocketService;
    std::atomic<uint16_t> mTransactionCounter;

    std::vector<ModelHCommandClientSubscriber*> mResponseSubscribers;

    LegacyCommandProtocolSemVer mCommandSemVer = LegacyCommandProtocolSemVer::COMMAND_PROTOCOL_SEM_VER_LATEST;

    void* mpCurrentPacket = nullptr;

    class SocketImpl;
    std::unique_ptr<SocketImpl> mpSocketImpl;
};
}  // namespace lum
