/*
* ModelHClient.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ModelHClient.h"

#include <iostream>

#include "Conversions.h"
#include "LidarReturn.h"
#include "LumNet/LidarData.h"
#include "ModelHDistributor.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>

using namespace lum;
using namespace std;
using namespace boost::asio::ip;

class ModelHClient::SocketImpl
{
public:
    // Set receive buffer to 64MB (matches memory pool size)
    static const auto RECEIVE_BUFFER_SIZE = 64'000'000;

    SocketImpl( uint16_t port )
        : mPort( port )
    {
    }

    void SetupReceive()
    {
        ScheduleReceive();
        mpIoService->run();
    }

    void ScheduleReceive()
    {
        memset( mpCurrentPacket.get(), 0, LUM_NET_MAX_LIDAR_DATA_PAYLOAD_SIZE * sizeof( *mpCurrentPacket.get() ) );
        mpSocket->async_receive_from(
            boost::asio::buffer( mpCurrentPacket.get(), LUM_NET_MAX_LIDAR_DATA_PAYLOAD_SIZE ),
            *mpReceiveEndpoint,
            boost::bind( &SocketImpl::HandleReceiveFromUdpSocket,
                         this, boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred ) );
    }

    void HandleReceiveFromUdpSocket( const boost::system::error_code& error, size_t bytes_recvd )
    {
        if ( error || mUdpThreadRunning == false )
        {
            return;
        }
        if ( bytes_recvd > 0 )
        {
            const auto preamble = reinterpret_cast<LumNet_LidarDataPreamble*>( mpCurrentPacket.get() );

            if ( preamble->semVersionedHeader.payloadTypeID == LUM_NET_PAYLOAD_TYPE_ID_LIDAR_DATA )
            {
                ModelHDistributor::get().PacketReceived( (uint8_t*)preamble, bytes_recvd, mpReceiveEndpoint->address().to_string() );
            }
        }
        ScheduleReceive();
    }

    std::unique_ptr<uint8_t[]> mpCurrentPacket = std::make_unique<uint8_t[]>( LUM_NET_MAX_LIDAR_DATA_PAYLOAD_SIZE );
    boost::asio::io_service* mpIoService = nullptr;
    boost::asio::ip::udp::socket* mpSocket = nullptr;
    boost::asio::ip::udp::endpoint* mpReceiveEndpoint = nullptr;
    std::thread* mpUdpThread = nullptr;
    uint16_t mPort = 0;
    bool mUdpThreadRunning = false;
};

ModelHClient::ModelHClient( uint16_t port )
{
    mpSocketImpl = std::make_unique<SocketImpl>( port );
}

ModelHClient::~ModelHClient()
{
    Stop();
}

void ModelHClient::Start()
{
    Stop();
    if ( mpSocketImpl->mUdpThreadRunning == false )
    {
        try
        {
            mpSocketImpl->mpIoService = new boost::asio::io_service();
            mpSocketImpl->mpReceiveEndpoint = new udp::endpoint( udp::v4(), mpSocketImpl->mPort );
            mpSocketImpl->mpSocket = new udp::socket( *mpSocketImpl->mpIoService );
            mpSocketImpl->mpSocket->open( mpSocketImpl->mpReceiveEndpoint->protocol() );
            mpSocketImpl->mpSocket->set_option( boost::asio::socket_base::reuse_address( true ) );
            mpSocketImpl->mpSocket->set_option( boost::asio::socket_base::receive_buffer_size( SocketImpl::RECEIVE_BUFFER_SIZE ) );
            mpSocketImpl->mpSocket->bind( *mpSocketImpl->mpReceiveEndpoint );
            mpSocketImpl->mUdpThreadRunning = true;
            mpSocketImpl->mpUdpThread = new std::thread( &SocketImpl::SetupReceive, mpSocketImpl.get() );
        }
        catch ( std::exception& e )
        {
            mpSocketImpl->mUdpThreadRunning = false;
            (void)e;
#ifdef _DEBUG
            std::cout << e.what() << '\n';
#endif
            if ( mpSocketImpl->mpSocket )
            {
                delete mpSocketImpl->mpSocket;
                mpSocketImpl->mpSocket = nullptr;
            }
            if ( mpSocketImpl->mpReceiveEndpoint )
            {
                delete mpSocketImpl->mpReceiveEndpoint;
                mpSocketImpl->mpReceiveEndpoint = nullptr;
            }
            if ( mpSocketImpl->mpIoService )
            {
                delete mpSocketImpl->mpIoService;
                mpSocketImpl->mpIoService = nullptr;
            }
        }
    }
}

void ModelHClient::Stop()
{
    if ( mpSocketImpl->mpSocket )
    {
        try
        {
            mpSocketImpl->mpSocket->cancel();
            mpSocketImpl->mpSocket->shutdown( udp::socket::shutdown_both );
            mpSocketImpl->mpSocket->close();
        }
        catch ( std::exception e )
        {
#ifdef _DEBUG
            std::cout << e.what() << '\n';
#endif
        }
        mpSocketImpl->mpIoService->stop();
        mpSocketImpl->mUdpThreadRunning = false;
        if ( mpSocketImpl->mpUdpThread->joinable() )
        {
            mpSocketImpl->mpUdpThread->join();
        }
        delete mpSocketImpl->mpUdpThread;
        mpSocketImpl->mpUdpThread = nullptr;
        delete mpSocketImpl->mpSocket;
        mpSocketImpl->mpSocket = nullptr;
        delete mpSocketImpl->mpIoService;
        mpSocketImpl->mpIoService = nullptr;
        delete mpSocketImpl->mpReceiveEndpoint;
        mpSocketImpl->mpReceiveEndpoint = nullptr;
    }
}
