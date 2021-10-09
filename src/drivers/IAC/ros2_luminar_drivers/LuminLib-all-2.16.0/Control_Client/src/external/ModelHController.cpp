/*
* ModelHController.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ModelHController.h"

#include <boost/algorithm/string.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <regex>
#ifdef __unix__
#include <ifaddrs.h>
#endif

#include "CommandConstructors.h"
#include "CommandSizes.h"
#include "LumNet/Common.h"
#include "LumNet/Discovery.h"

using boost::asio::ip::address_v4;

namespace lum
{
class ModelHController::SocketImpl
{
public:
    SocketImpl()
        : mSendSocket( mIoService ) {}
    boost::asio::io_service mIoService;
    boost::asio::ip::udp::socket mSendSocket;
    boost::asio::ip::udp::endpoint mReceiveEndpoint;
};

ModelHController::ModelHController()
    : mpSocketImpl( new SocketImpl() )
{
    mTransactionCounter.store( 0 );
}

ModelHController::~ModelHController()
{
    if ( mpResponseServer )
    {
        StopResponseServer();
    }
}

ModelHController& ModelHController::get()
{
    static ModelHController instance;
    return instance;
}

/*
 * Adds a command response subscriber
 *
 */
void ModelHController::AddCommandResponseSubscriber( ModelHCommandClientSubscriber* subscriber )
{
    assert( subscriber );

    mResponseSubscribers.push_back( subscriber );
}

/*
 * Removes a command response subscriber
 *
 */
bool ModelHController::RemoveCommandResponseSubscriber( ModelHCommandClientSubscriber* subscriber )
{
    auto&& loc = std::find( mResponseSubscribers.begin(), mResponseSubscribers.end(), subscriber );

    if ( loc != mResponseSubscribers.end() )
    {
        mResponseSubscribers.erase( loc );
        return true;
    }

    return false;
}

TransactionID ModelHController::NextTransactionId()
{
    TransactionID transactionId = mTransactionCounter++;

    return transactionId;
}

bool ModelHController::SendCommand( DiscoveryResponse commandedSensor, uint8_t* command, size_t command_size )
{
    mSocketService.SendCommand( commandedSensor.mFingerprint, command, command_size );

    return true;
}

LegacyCommandProtocolSemVer ModelHController::GetCurrentSemVer()
{
    return mCommandSemVer;
}

void ModelHController::SetCurrentSemVer( LegacyCommandProtocolSemVer version )
{
    mCommandSemVer = version;
}

void ModelHController::DispatchCommandResponse( FingerprintID fingerprint, uint8_t* response )
{
    for ( auto&& subscriber : mResponseSubscribers )
    {
        HandleCommandResponse( subscriber, fingerprint, response );
    }
}

void ModelHController::StartResponseServer()
{
    mResponseThread = std::thread( &lum::ResponseServer::Start, mpResponseServer.get() );
}

void ModelHController::StopResponseServer()
{
    if ( mpResponseServer )
    {
        mpResponseServer->Stop();

        if ( mResponseThread.joinable() )
        {
            mResponseThread.join();
        }
    }
}

void ModelHController::Initialize()
{
    mpResponseServer.reset( new ResponseServer( LUM_NET_DEFAULT_UDP_DISCOVERY_PORT ) );
}

void ModelHController::AddSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber )
{
    mSocketService.AddSensorConnectionSubscriber( subscriber );
}

bool ModelHController::RemoveSensorConnectionSubscriber( SensorConnectionSubscriber* subscriber )
{
    return mSocketService.RemoveSensorConnectionSubscriber( subscriber );
}

void ModelHController::ConnectToSensor( const DiscoveryResponse& sensor )
{
    mSocketService.ConnectToDevice( sensor.mFingerprint, sensor.mHeadIPAddress, sensor.mCmdTCPPort );
}

void ModelHController::RemoveSensorConnectDestination( const DiscoveryResponse& sensor )
{
    mSocketService.CloseDeviceConnection( sensor.mFingerprint );
}

void ModelHController::UpdateSensorConnection( const DiscoveryResponse& prevResponse, const DiscoveryResponse& newResponse )
{
    if ( prevResponse.mFingerprint != newResponse.mFingerprint )
    {
        mSocketService.UpdateDeviceFingerprint( prevResponse.mFingerprint, newResponse.mFingerprint );
    }
}

void ModelHController::DiscoverSensorHeads( uint16_t port )
{
    /** Start with fresh discovery response list for every discovery event */
    {
        std::lock_guard<std::mutex> lock( mDiscoveryMutex );
        mDiscoveryResponseList.clear();
    }

    auto request = LumNet_DiscoveryRequest();
    request.commonHeader.sentinel = LUM_NET_SENTINEL;
    request.commonHeader.semVersionedHeader.semver = LumNet_GetDiscoveryProtocolSemVer();
    request.commonHeader.semVersionedHeader.payloadTypeID = LUM_NET_PAYLOAD_TYPE_ID_DSC_REQUEST;
    BroadcastRequest( &request, sizeof( request ), port );

    // insert a delay to minimize network impact (and provide client time to respond)
    std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );

    auto request_v4 = LumNet_DiscoveryRequest();
    request_v4.commonHeader.sentinel = LUM_NET_SENTINEL;
    request_v4.commonHeader.semVersionedHeader.semver = LegacyLumNet_V4_GetDiscoveryProtocolSemVer();
    request_v4.commonHeader.semVersionedHeader.payloadTypeID = LUM_NET_PAYLOAD_TYPE_ID_DSC_REQUEST;
    BroadcastRequest( &request_v4, sizeof( request_v4 ), port );

    // insert a delay to minimize network impact (and provide client time to respond)
    std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );

    auto request_v2 = LumNet_DiscoveryRequest();
    request_v2.commonHeader.sentinel = LUM_NET_SENTINEL;
    request_v2.commonHeader.semVersionedHeader.semver = LegacyLumNet_V2_GetDiscoveryProtocolSemVer();
    request_v2.commonHeader.semVersionedHeader.payloadTypeID = LUM_NET_PAYLOAD_TYPE_ID_DSC_REQUEST;

    BroadcastRequest( &request_v2, sizeof( request_v2 ), port );
}

void ModelHController::RecoverSensorHead( std::string serial_number, bool override, uint16_t port )
{
    LumNet_NetworkConnectionRecovery request;
    request.header.sentinel = LUM_NET_SENTINEL;
    request.header.semVersionedHeader.semver = LumNet_GetDiscoveryProtocolSemVer();
    request.header.semVersionedHeader.payloadTypeID = LUM_NET_PAYLOAD_TYPE_ID_DSC_NETWORK_CONNECTION_RECOVERY;

    request.overrideSerialNumber = override;
    memcpy( request.serialNumber.value, serial_number.c_str(), LUM_NET_SERIAL_NUMBER_BYTE_LEN );

    BroadcastRequest( &request, sizeof( request ), port );
}

void ModelHController::ReceivedResponse( char* buffer, size_t bytes_received )
{
    if ( bytes_received == sizeof( LumNet_DiscoveryResponse ) )
    {
        auto response = reinterpret_cast<LumNet_DiscoveryResponse*>( buffer );
        if ( response &&
             LumNet_HasDiscoveryCommonHeader( &response->commonHeader ) &&
             response->commonHeader.semVersionedHeader.payloadTypeID == LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE )
        {
            auto res = DiscoveryResponse( response );
            UpdateDiscoveryResponseList( res );
        }
    }
    else if ( bytes_received == sizeof( LegacyLumNet_V4_DiscoveryResponse ) )
    {
        auto response_v4 = reinterpret_cast<LegacyLumNet_V4_DiscoveryResponse*>( buffer );
        if ( response_v4 &&
             LegacyLumNet_HasLegacyV4DiscoveryCommonHeader( &response_v4->commonHeader ) &&
             response_v4->commonHeader.semVersionedHeader.payloadTypeID == LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE_DEPRECATED_V4 )
        {
            auto res = DiscoveryResponse( response_v4 );
            UpdateDiscoveryResponseList( res );
        }
    }
    else if ( bytes_received == sizeof( LegacyLumNet_V2_DiscoveryResponse ) )
    {
        auto response_v2 = reinterpret_cast<LegacyLumNet_V2_DiscoveryResponse*>( buffer );
        if ( response_v2 &&
             LegacyLumNet_HasLegacyV2DiscoveryCommonHeader( &response_v2->commonHeader ) &&
             response_v2->commonHeader.semVersionedHeader.payloadTypeID == LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE_DEPRECATED )
        {
            auto res = DiscoveryResponse( response_v2 );
            UpdateDiscoveryResponseList( res );
        }
    }
}

void ModelHController::UpdateDiscoveryResponseList( const DiscoveryResponse& res )
{
    std::lock_guard<std::mutex> lock( mDiscoveryMutex );
    auto iter = std::find_if( mDiscoveryResponseList.begin(), mDiscoveryResponseList.end(),
                              [&]( const auto& response ) {
                                  return response.SameSensor( res );
                              } );
    if ( iter != mDiscoveryResponseList.end() )
    {
        // Check if type is newer than current
        if ( res.mDiscoveryVersion > ( *iter ).mDiscoveryVersion )
        {
            ( *iter ) = res;
        }
        // Type is the same (duplicate) or older than current (ignore older responses)
        else
        {
            return;
        }
    }
    // Not in our history (add it)
    else
    {
        mDiscoveryResponseList.push_back( res );
    }
}

std::vector<DiscoveryResponse> ModelHController::GetDiscoveryList()
{
    std::lock_guard<std::mutex> lock( mDiscoveryMutex );
    // Sort responses newest -> oldest
    std::sort( mDiscoveryResponseList.begin(), mDiscoveryResponseList.end(), std::greater<lum::DiscoveryResponse>() );
    return mDiscoveryResponseList;
}

uint16_t ModelHController::SetPersistentNetworkInfo( DiscoveryResponse commandedSensor, uint32_t newDestIp, uint16_t newDestport, int enable, uint32_t newSensorIp, LumNet_Nickname nickname )
{
    uint8_t* buffer = nullptr;
    uint16_t transactionID = mTransactionCounter++;
    LumNet_Endpoint endPoint;
    endPoint.ipAddress = newDestIp;
    endPoint.port = newDestport;

    Command_EncodeSetPersistentLidarDataEndpointCommand(
        endPoint,
        transactionID,
        &buffer,
        GetCurrentSemVer() );

    size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize( (struct LumNet_CommandRequestListPayload*)buffer );

    mSocketService.SendCommand( commandedSensor.mFingerprint, buffer, buffer_size );

    free( buffer );

    buffer = nullptr;
    transactionID = mTransactionCounter++;

    Command_EncodeSetPersistentSensorLidarDataEnabledCommand(
        enable,
        transactionID,
        &buffer,
        GetCurrentSemVer() );

    buffer_size = Command_CalculateLumNet_CommandListEncodeSize( (struct LumNet_CommandRequestListPayload*)buffer );

    mSocketService.SendCommand( commandedSensor.mFingerprint, buffer, buffer_size );

    free( buffer );

    buffer = nullptr;
    transactionID = mTransactionCounter++;
    endPoint.ipAddress = newSensorIp;
    endPoint.port = 0;

    Command_EncodeSetPersistentSensorIPAddressCommand(
        endPoint.ipAddress,
        transactionID,
        &buffer,
        GetCurrentSemVer() );

    buffer_size = Command_CalculateLumNet_CommandListEncodeSize( (struct LumNet_CommandRequestListPayload*)buffer );

    mSocketService.SendCommand( commandedSensor.mFingerprint, buffer, buffer_size );

    free( buffer );

    buffer = nullptr;
    transactionID = mTransactionCounter++;

    Command_EncodeSetPersistentSensorNicknameCommand(
        nickname,
        transactionID,
        &buffer,
        GetCurrentSemVer() );

    buffer_size = Command_CalculateLumNet_CommandListEncodeSize( (struct LumNet_CommandRequestListPayload*)buffer );

    mSocketService.SendCommand( commandedSensor.mFingerprint, buffer, buffer_size );

    free( buffer );

    return transactionID;
}

uint16_t ModelHController::SetVolatileNetworkInfo( DiscoveryResponse commandedSensor, uint32_t newDestIp, uint16_t newDestport, int enable, LumNet_Nickname nickname )
{
    uint8_t* buffer = nullptr;
    uint16_t transactionID = mTransactionCounter++;
    LumNet_Endpoint endPoint;
    endPoint.ipAddress = newDestIp;
    endPoint.port = newDestport;

    Command_EncodeSetVolatileLidarDataEndpointCommand(
        endPoint,
        transactionID,
        &buffer,
        GetCurrentSemVer() );

    size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize( (struct LumNet_CommandRequestListPayload*)buffer );

    mSocketService.SendCommand( commandedSensor.mFingerprint, buffer, buffer_size );

    free( buffer );

    buffer = nullptr;
    transactionID = mTransactionCounter++;

    Command_EncodeSetVolatileSensorLidarDataEnabledCommand(
        enable,
        transactionID,
        &buffer,
        GetCurrentSemVer() );

    buffer_size = Command_CalculateLumNet_CommandListEncodeSize( (struct LumNet_CommandRequestListPayload*)buffer );

    mSocketService.SendCommand( commandedSensor.mFingerprint, buffer, buffer_size );

    free( buffer );

    buffer = nullptr;
    transactionID = mTransactionCounter++;

    Command_EncodeSetVolatileSensorNicknameCommand(
        nickname,
        transactionID,
        &buffer,
        GetCurrentSemVer() );

    buffer_size = Command_CalculateLumNet_CommandListEncodeSize( (struct LumNet_CommandRequestListPayload*)buffer );

    mSocketService.SendCommand( commandedSensor.mFingerprint, buffer, buffer_size );

    free( buffer );

    return transactionID;
}

const std::vector<std::string> ModelHController::GetHostnamesIpv4() const
{
    std::vector<std::string> ips;
#if WIN32
    // Get my IP address.
    boost::asio::io_service io;
    boost::asio::ip::udp::resolver resolver( io );
    boost::asio::ip::udp::resolver::query query( boost::asio::ip::udp::v4(),
                                                 boost::asio::ip::host_name(), "" );
    boost::asio::ip::udp::resolver::iterator hosts = resolver.resolve( query );

    std::regex ipPattern( "[0-9]{1,3}[.][0-9]{1,3}[.][0-9]{1,3}[.][0-9]{1,3}" );

    while ( hosts != boost::asio::ip::udp::resolver::iterator() )
    {
        boost::asio::ip::udp::endpoint e = ( *hosts );

        // Tokenize the to get IP and num.
        std::vector<std::string> tokens;
        std::string eString = boost::lexical_cast<std::string>( e );
        boost::algorithm::split( tokens,
                                 eString,
                                 boost::is_any_of( ":" ),
                                 boost::algorithm::token_compress_on );
        if ( tokens.size() >= 1 && std::regex_match( tokens[0], ipPattern ) )
        {
            const auto firstOctetPos = tokens[0].find_first_of( '.' );
            auto firstOctet = std::string( tokens[0].substr( 0, firstOctetPos ) );
            if ( firstOctet != "127" )
            {
                ips.push_back( tokens[0] );
            }
        }
        hosts++;
    }
#else
    ifaddrs* interfaces = nullptr;
    getifaddrs( &interfaces );
    for ( ifaddrs* ifa = interfaces; ifa != nullptr; ifa = ifa->ifa_next )
    {
        if ( ifa->ifa_addr == nullptr )
            continue;

        if ( ifa->ifa_addr->sa_family == AF_INET )
        {
            //get interfaces ip address
            char host[NI_MAXHOST];
            getnameinfo( ifa->ifa_addr, sizeof( struct sockaddr_in ), host, NI_MAXHOST, nullptr, 0, NI_NUMERICHOST );
            std::string hostString( host );
            ips.push_back( std::string( host ) );
        }
    }
#endif
    return ips;
}

void ModelHController::BroadcastRequest( void* request, size_t requestSizeBytes, uint16_t port )
{
    for ( const auto& ip : GetHostnamesIpv4() )
    {
        boost::system::error_code error;
        mpSocketImpl->mSendSocket.open( boost::asio::ip::udp::v4(), error );
        mpSocketImpl->mSendSocket.set_option( boost::asio::ip::udp::socket::reuse_address( true ), error );
        mpSocketImpl->mSendSocket.set_option( boost::asio::socket_base::broadcast( true ), error );
        mpSocketImpl->mSendSocket.bind( boost::asio::ip::udp::endpoint( address_v4::from_string( ip ), 0 ), error );
#ifdef _DEBUG
        if ( error )
        {
            std::cout << error.message() << '\n';
        }
#endif
        mpSocketImpl->mSendSocket.send_to( boost::asio::buffer( request, requestSizeBytes ),
                                           boost::asio::ip::udp::endpoint( address_v4::broadcast(), port ), 0, error );
        mpSocketImpl->mSendSocket.close( error );
    }
}
}  // namespace lum
