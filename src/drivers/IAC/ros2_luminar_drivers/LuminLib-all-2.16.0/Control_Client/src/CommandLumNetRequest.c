/*
* CommandLumNetRequest.c
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "CommandLumNetRequest.h"
#include "assert.h"
#include "stddef.h"
#include "stdlib.h"
#include "string.h"

#include "CommandSizes.h"
#include "CommonDefinitions.h"

#include "math.h"

struct LumNet_CommandRequestListPayload* Command_CreateCommandRequestList(
    struct LumNet_CommandRequest commands[],
    size_t num_commands,
    uint16_t listTransactionID,
    enum LegacyCommandProtocolSemVer version )
{
    size_t payload_size = Command_CalculateLumNet_CommandListPayloadSize( commands, num_commands );
    size_t list_size = offsetof( struct LumNet_CommandRequestListPayload, commands ) + payload_size;

    struct LumNet_CommandRequestListPayload* request_list = malloc( list_size );
    assert( request_list != NULL );

    if ( request_list != NULL )
    {
        memset( request_list, 0, list_size );

        request_list->preamble.versionPayloadHeader.sentinel = LUM_NET_SENTINEL;

        request_list->preamble.versionPayloadHeader.semVersionedHeader.semver = GetLegacyCommandProtocolSemVer( version );

        request_list->preamble.versionPayloadHeader.semVersionedHeader.payloadTypeID = LUM_NET_PAYLOAD_TYPE_ID_CMD_REQUEST_LIST;

        request_list->commandRequestListHeader.listTransactionID = listTransactionID;
        request_list->commandRequestListHeader.listPayloadLength = (uint16_t)payload_size;
    }

    memcpy( request_list->commands, commands, payload_size );

    return request_list;
}

enum Common_returnCode Command_CopyCommandRequestList(
    struct LumNet_CommandRequestListPayload* request_list,
    uint8_t** encoded_list,
    size_t* encoded_list_size )
{
    assert( request_list != NULL );

    assert( encoded_list != NULL );
    assert( *encoded_list == NULL );

    *encoded_list_size = Command_CalculateLumNet_CommandListEncodeSize( request_list );

    uint8_t* the_encoded_list = malloc( *encoded_list_size );
    assert( the_encoded_list );

    if ( the_encoded_list != NULL )
    {
        memset( the_encoded_list, 0, *encoded_list_size );
        memcpy( the_encoded_list, request_list, *encoded_list_size );

        *encoded_list = the_encoded_list;
    }

    return COMMAND_RETURN_CODE_SUCCESS;
}

enum Common_returnCode Command_CreateLumNetRequest_preallocated(
    struct LumNet_CommandRequest* request,
    uint16_t address,
    uint8_t operation,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload )
{
    assert( request != NULL );
    return Command_CreateLumNetRequest_preallocated_internal( request,
                                                              address,
                                                              operation,
                                                              LUM_NET_CMD_MAGIC_NUMBER,
                                                              transaction_id,
                                                              payload_length,
                                                              payload );
}

enum Common_returnCode Command_CreateLumNetRequest_preallocated_internal(
    struct LumNet_CommandRequest* request,
    uint16_t address,
    uint8_t operation,
    uint8_t magic_number,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload )
{
    assert( request != NULL );

    request->commandHeader.address = address;
    request->commandHeader.operation = operation;
    request->commandHeader.magicNumber = magic_number;
    request->commandHeader.transactionID = transaction_id;
    request->commandHeader.payloadLength = payload_length;

    memcpy( request->payload, payload, payload_length );

    return COMMAND_RETURN_CODE_SUCCESS;
}

struct LumNet_CommandRequest* Command_CreateLumNetRequestNoPayload(
    uint16_t address,
    uint8_t operation,
    uint16_t transaction_id )
{
    return Command_CreateLumNetRequest_internal( address,
                                                 operation,
                                                 LUM_NET_CMD_MAGIC_NUMBER,
                                                 transaction_id,
                                                 0,
                                                 NULL );
}

struct LumNet_CommandRequest* Command_CreateLumNetRequest(
    uint16_t address,
    uint8_t operation,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload )
{
    return Command_CreateLumNetRequest_internal( address,
                                                 operation,
                                                 LUM_NET_CMD_MAGIC_NUMBER,
                                                 transaction_id,
                                                 payload_length,
                                                 payload );
}

struct LumNet_CommandRequest* Command_CreateLumNetRequest_internal(
    uint16_t address,
    uint8_t operation,
    uint8_t magic_number,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload )
{
    size_t req_size = sizeof( struct LumNet_CommandRequest ) + ( payload_length );
    struct LumNet_CommandRequest* req = malloc( req_size );
    assert( req );

    if ( req != NULL )
    {
        memset( req, 0, req_size );

        req->commandHeader.address = address;
        req->commandHeader.operation = operation;
        req->commandHeader.magicNumber = magic_number;
        req->commandHeader.transactionID = transaction_id;
        req->commandHeader.payloadLength = payload_length;

        if ( payload != NULL && payload_length > 0U )
        {
            memcpy( req->payload, payload, payload_length );
        }
    }

    return req;
}
