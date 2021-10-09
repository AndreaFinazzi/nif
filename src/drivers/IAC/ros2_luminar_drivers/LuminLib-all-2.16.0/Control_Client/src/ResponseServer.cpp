/*
* ResponseServer.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ResponseServer.h"
#include "ModelHController.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <iostream>
#include <string>

namespace lum
{
class ResponseServer::SocketImpl
{
public:
    static const auto MAX_PACKET_SIZE = 1500;

    SocketImpl()
        : mSocket( mIoService )
    {
    }

    const std::string ReceiveEndpoint() const
    {
        return mReceiveEndpoint.address().to_string();
    }

    void SendDataLoopback( void* data, size_t bytes, uint16_t port )
    {
        boost::system::error_code error;
        mSocket.send_to(
            boost::asio::buffer( data, bytes ),
            boost::asio::ip::udp::endpoint(
                boost::asio::ip::address_v4::loopback(), port ),
            0, error );
    }

    //Sets up the async receiver from the sensor
    void SetupReceive()
    {
        mSocket.async_receive_from(
            boost::asio::buffer( mpCurrentPacket.get(), MAX_PACKET_SIZE ),
            mReceiveEndpoint,
            boost::bind( &ResponseServer::SocketImpl::HandleReceiveFromUdpSocket, this,
                         boost::asio::placeholders::error,
                         boost::asio::placeholders::bytes_transferred ) );
    }

    /*
    * Callback responsible for parsing the preamble from packet.
    * Responsible for calling HandleRayPacket for each ray contained in the packet
    */
    void HandleReceiveFromUdpSocket( const boost::system::error_code& error,
                                     std::size_t bytes_recvd )
    {
        if ( !error && bytes_recvd > 0 )
        {
            ModelHController::get().ReceivedResponse( mpCurrentPacket.get(), bytes_recvd );
        }
        SetupReceive();
    }
    boost::asio::io_service mIoService;
    boost::asio::ip::udp::socket mSocket;
    boost::asio::ip::udp::endpoint mReceiveEndpoint;
    std::unique_ptr<char[]> mpCurrentPacket = std::make_unique<char[]>( MAX_PACKET_SIZE );
};

ResponseServer::ResponseServer( uint16_t port )
{
    mpSocketImpl = std::make_unique<SocketImpl>();
    mpSocketImpl->mReceiveEndpoint = boost::asio::ip::udp::endpoint( boost::asio::ip::udp::v4(), port );
    boost::system::error_code error;
    mpSocketImpl->mSocket.open( mpSocketImpl->mReceiveEndpoint.protocol(), error );
    assert( !error );
    mpSocketImpl->mSocket.set_option( boost::asio::ip::udp::socket::reuse_address( true ), error );
    mpSocketImpl->mSocket.set_option( boost::asio::socket_base::broadcast( true ), error );
    assert( !error );
    mpSocketImpl->mSocket.bind( mpSocketImpl->mReceiveEndpoint );
}

ResponseServer::~ResponseServer()
{
    Stop();
}

void ResponseServer::Start()
{
    mpSocketImpl->SetupReceive();
    boost::system::error_code error;
    mpSocketImpl->mIoService.run( error );
    assert( !error );
}

//Stops and closes all the open networking code. Stops all threads started by this class
void ResponseServer::Stop()
{
    mpSocketImpl->mIoService.stop();

    if ( mpSocketImpl->mSocket.is_open() )
    {
        boost::system::error_code ec;
        mpSocketImpl->mSocket.shutdown( boost::asio::socket_base::shutdown_both, ec );
        if ( ec )
        {
            std::cout << ec.message() << '\n';
        }
        mpSocketImpl->mSocket.close();
    }
}
}  // namespace lum
