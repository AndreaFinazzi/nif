/*
* SensorSocketService.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "SensorSocketService.h"

#include <assert.h>

#include <boost/asio.hpp>
#include <iostream>
#include <mutex>

#include "ModelHController.h"
#include "SensorConnectionSubscriber.h"

using namespace boost::asio;
using namespace boost::asio::ip;
using namespace lum;

class SensorSocketService::SocketImpl
{
public:
    struct SensorConnectionInfo
    {
        static constexpr auto BUFFER_SIZE = 1500;

        boost::asio::io_service* pIOService = nullptr;
        boost::asio::ip::tcp::socket* pSocket = nullptr;
        std::thread* pThread = nullptr;
        uint8_t buffer[BUFFER_SIZE];
        lum::SensorConnectionStateManager mSensorConnectionStateManager;
        std::vector<boost::asio::ip::tcp::endpoint> mEndpoints;
    };

    SocketImpl() {}

    void AddSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber )
    {
        assert( subscriber );

        std::unique_lock<std::mutex> lock( mConnectionSubscribersMutex );
        mConnectionSubscribers.push_back( subscriber );
    }
    bool RemoveSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber )
    {
        std::unique_lock<std::mutex> lock( mConnectionSubscribersMutex );

        auto&& loc =
            std::find( mConnectionSubscribers.begin(), mConnectionSubscribers.end(), subscriber );

        if ( loc != mConnectionSubscribers.end() )
        {
            mConnectionSubscribers.erase( loc );
            return true;
        }

        return false;
    }

    void ReadSocket( uint16_t fingerprint )
    {
        SensorConnectionInfo* connectionInfo = nullptr;
        {
            std::unique_lock<std::mutex> lock( mConnectionsMutex );
            auto found = mActiveConnections.find( fingerprint );
            if ( found != mActiveConnections.end() )
            {
                connectionInfo = ( *found ).second;
            }
        }
        while ( connectionInfo )
        {
            boost::system::error_code ec;
            size_t byteCount = connectionInfo->pSocket->read_some(
                buffer( connectionInfo->buffer, SensorConnectionInfo::BUFFER_SIZE ), ec );
            if ( ec )
            {
#ifdef _DEBUG
                // socket error, shutdown socket
                std::cout << "TCP Connection lost on sensor: " << fingerprint << ". With the following error...\n"
                          << "\"" << ec.message() << "\"\n";
#endif
                return;
            }
        }
    }

    void SetupSocket( uint16_t fingerprint )
    {
        SensorConnectionInfo* pConnectionInfo = nullptr;
        {
            std::unique_lock<std::mutex> lock( mConnectionsMutex );
            auto found = mActiveConnections.find( fingerprint );
            if ( found != mActiveConnections.end() )
            {
                pConnectionInfo = ( *found ).second;
            }
        }
        if ( pConnectionInfo )
        {
            boost::asio::async_connect( *( pConnectionInfo->pSocket ),
                                        pConnectionInfo->mEndpoints.begin(), pConnectionInfo->mEndpoints.end(),
                                        [this, fingerprint]( const auto& ec, const auto iter ) {
                                            this->ConnectHandler( ec, iter, fingerprint );
                                        } );
            pConnectionInfo->pIOService->run();
        }
    }

    void ConnectHandler( const boost::system::error_code& ec,
                         std::vector<boost::asio::ip::tcp::endpoint>::iterator i,
                         uint16_t fingerprint )
    {
        SensorConnectionInfo* pConnectionInfo = nullptr;
        {
            std::unique_lock<std::mutex> lock( mConnectionsMutex );

            auto found = mActiveConnections.find( fingerprint );
            if ( found != mActiveConnections.end() )
            {
                pConnectionInfo = ( *found ).second;
            }
        }
        if ( pConnectionInfo )
        {
            if ( ec )
            {
#ifdef _DEBUG
                std::cout << "Error: " << ec.message() << "\n";
#endif
                boost::asio::async_connect( *( pConnectionInfo->pSocket ),
                                            pConnectionInfo->mEndpoints.begin(), pConnectionInfo->mEndpoints.end(),
                                            [this, fingerprint]( const auto& errorCode, const auto iter ) {
                                                this->ConnectHandler( errorCode, iter, fingerprint );
                                            } );
            }
            else
            {
#ifdef _DEBUG
                std::cout << "Address: " << ( *i ).address().to_string() << "\n";
#endif
                NotifyConnectionSubscribers( fingerprint );

                pConnectionInfo->pSocket->async_read_some(
                    buffer( pConnectionInfo->buffer, SensorConnectionInfo::BUFFER_SIZE ),
                    [this, fingerprint]( const auto& ec, const auto size ) {
                        this->ReadHandler( ec, size, fingerprint );
                    } );
            }
        }
    }

    void ReadHandler( const boost::system::error_code& ec, size_t bytes_transferred, uint16_t fingerprint )
    {
        SensorConnectionInfo* pConnectionInfo = nullptr;
        {
            std::unique_lock<std::mutex> lock( mConnectionsMutex );
            auto found = mActiveConnections.find( fingerprint );
            if ( found != mActiveConnections.end() )
            {
                pConnectionInfo = ( *found ).second;
            }
        }
        if ( pConnectionInfo )
        {
            if ( !ec )
            {
                pConnectionInfo->mSensorConnectionStateManager.ProcessBytes( pConnectionInfo->buffer, static_cast<uint32_t>( bytes_transferred ), fingerprint );

                pConnectionInfo->pSocket->async_read_some(
                    buffer( pConnectionInfo->buffer, SensorConnectionInfo::BUFFER_SIZE ),
                    [this, fingerprint]( const auto& errorCode, const auto size ) {
                        this->ReadHandler( errorCode, size, fingerprint );
                    } );
            }
            else
            {
                //Attempt to reconnect
                //TODO: Check error code, might not always make sense
                boost::asio::async_connect( *( pConnectionInfo->pSocket ),
                                            pConnectionInfo->mEndpoints.begin(), pConnectionInfo->mEndpoints.end(),
                                            [this, fingerprint]( const auto& errorCode, const auto iter ) {
                                                this->ConnectHandler( errorCode, iter, fingerprint );
                                            } );
            }
        }
    }

    void CloseAllDeviceConnections()
    {
        std::unique_lock<std::mutex> lock( mConnectionsMutex );

        for ( auto&& iter = mActiveConnections.begin(); iter != mActiveConnections.end(); )
        {
            auto pConnectionInfo = iter->second;
            iter = mActiveConnections.erase( iter );
            lock.unlock();
            ShutdownConnection( pConnectionInfo );
            lock.lock();
        }
    }
    void ShutdownConnection( SensorConnectionInfo* pConnectionInfo )
    {
        try
        {
            pConnectionInfo->pSocket->cancel();
            pConnectionInfo->pSocket->shutdown( tcp::socket::shutdown_both );
            pConnectionInfo->pSocket->close();
        }
        catch ( std::exception e )
        {
            (void)e;
#ifdef _DEBUG
            std::cout << e.what() << '\n';
#endif
        }
        pConnectionInfo->pIOService->stop();
        if ( pConnectionInfo->pThread->joinable() )
        {
            pConnectionInfo->pThread->join();
        }
        delete pConnectionInfo->pThread;
        pConnectionInfo->pThread = nullptr;
        delete pConnectionInfo->pSocket;
        pConnectionInfo->pSocket = nullptr;
        delete pConnectionInfo->pIOService;
        pConnectionInfo->pIOService = nullptr;
        delete pConnectionInfo;
        pConnectionInfo = nullptr;
    }

    void NotifyConnectionSubscribers( uint16_t fingerprint )
    {
        std::unique_lock<std::mutex> lock( mConnectionSubscribersMutex );
        for ( auto&& subscriber : mConnectionSubscribers )
        {
            subscriber->SensorConnected( fingerprint );
        }
    }

    std::mutex mConnectionsMutex;
    std::map<uint16_t, SocketImpl::SensorConnectionInfo*> mActiveConnections;
    std::mutex mConnectionSubscribersMutex;
    std::vector<SensorConnectionSubscriber*> mConnectionSubscribers;
};

SensorSocketService::SensorSocketService()
    : mpSocketImpl( new SocketImpl() )
{
}

SensorSocketService::~SensorSocketService()
{
    mpSocketImpl->CloseAllDeviceConnections();
}

void SensorSocketService::AddSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber )
{
    mpSocketImpl->AddSensorConnectionSubscriber( subscriber );
}

bool SensorSocketService::RemoveSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber )
{
    return mpSocketImpl->RemoveSensorConnectionSubscriber( subscriber );
}

void SensorSocketService::ConnectToDevice( uint16_t fingerprint, uint32_t ipAddress, uint16_t tcpPort )
{
    CloseDeviceConnection( fingerprint );
    auto connectionInfo = new SocketImpl::SensorConnectionInfo();
    connectionInfo->pIOService = new io_service();
    //convert to dotted decimal
    const int NBYTES = 4;
    uint8_t octet[NBYTES];
    char ipAddressFinal[16];
    for ( int i = 0; i < NBYTES; i++ )
    {
        octet[i] = ( ipAddress >> ( i * 8 ) ) & UINT8_MAX;
    }
    snprintf( ipAddressFinal, sizeof( ipAddressFinal ) / sizeof( ipAddressFinal[0] ),
              "%d.%d.%d.%d", octet[3], octet[2], octet[1], octet[0] );
    tcp::endpoint endpoint( boost::asio::ip::address::from_string( ipAddressFinal ), tcpPort );
    connectionInfo->pSocket = new tcp::socket( *connectionInfo->pIOService );

    try
    {
        connectionInfo->mEndpoints.push_back( endpoint );
        {
            std::unique_lock<std::mutex> lock( mpSocketImpl->mConnectionsMutex );
            mpSocketImpl->mActiveConnections[fingerprint] = connectionInfo;
        }
        connectionInfo->pThread = new std::thread( &SocketImpl::SetupSocket, mpSocketImpl.get(), fingerprint );
    }
    catch ( std::exception& e )
    {
        // Suppress unused variable warning
        (void)e;
#ifdef _DEBUG
        std::cout << e.what() << '\n';
#endif
        if ( connectionInfo->pSocket )
        {
            delete connectionInfo->pSocket;
            connectionInfo->pSocket = nullptr;
        }
        if ( connectionInfo->pIOService )
        {
            delete connectionInfo->pIOService;
            connectionInfo->pIOService = nullptr;
        }
        delete connectionInfo;
        connectionInfo = nullptr;
    }
}

void SensorSocketService::CloseDeviceConnection( uint16_t fingerprint )
{
    std::unique_lock<std::mutex> lock( mpSocketImpl->mConnectionsMutex );
    if ( mpSocketImpl->mActiveConnections.find( fingerprint ) != mpSocketImpl->mActiveConnections.end() )
    {
        auto pConnectionInfo = mpSocketImpl->mActiveConnections[fingerprint];
        mpSocketImpl->mActiveConnections.erase( fingerprint );
        lock.unlock();
        mpSocketImpl->ShutdownConnection( pConnectionInfo );
    }
}

void SensorSocketService::SendCommand( uint16_t fingerprint, uint8_t* data, size_t size )
{
    boost::system::error_code error;
    std::unique_lock<std::mutex> lock( mpSocketImpl->mConnectionsMutex );
    auto it = mpSocketImpl->mActiveConnections.find( fingerprint );

    if ( it != mpSocketImpl->mActiveConnections.end() )
    {
        auto pConnectionInfo = it->second;
        if ( pConnectionInfo != nullptr )
        {
            size_t written = pConnectionInfo->pSocket->write_some( boost::asio::buffer( data, size ), error );
            if ( !written )
            {
#ifdef _DEBUG
                std::cout << error.message() << "\n"
                          << "Error Code: " << error.value() << "\n";
#endif
                auto discoveryList = ModelHController::get().GetDiscoveryList();
                auto iter = std::find_if( discoveryList.begin(), discoveryList.end(),
                                          [&]( const auto& response ) {
                                              return response.mFingerprint == fingerprint;
                                          } );
                if ( iter != discoveryList.end() )
                {
                    mpSocketImpl->mActiveConnections.erase( it );
                    lock.unlock();
                    mpSocketImpl->ShutdownConnection( pConnectionInfo );
                    ConnectToDevice( fingerprint, ( *iter ).mHeadIPAddress, ( *iter ).mCmdTCPPort );
                }
            }
        }
    }
}

void SensorSocketService::UpdateDeviceFingerprint( uint16_t prevFingerprint, uint16_t newFingerprint )
{
    std::unique_lock<std::mutex> lock( mpSocketImpl->mConnectionsMutex );
    auto it = mpSocketImpl->mActiveConnections.find( prevFingerprint );
    if ( it != mpSocketImpl->mActiveConnections.end() )
    {
        auto pConnectionInfo = it->second;
        boost::asio::ip::tcp::endpoint endpoint = pConnectionInfo->pSocket->remote_endpoint();
        mpSocketImpl->mActiveConnections.erase( it );
        lock.unlock();
        mpSocketImpl->ShutdownConnection( pConnectionInfo );
        ConnectToDevice( newFingerprint, endpoint.address().to_v4().to_ulong(), endpoint.port() );
    }
}
