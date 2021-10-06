/*
* CommandLumNetResponse.c
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "CommandLumNetResponse.h"

#include "assert.h"
#include "stddef.h"
#include "stdlib.h"
#include "string.h"

size_t Command_CalculateLumNet_CommandResponseEncodeSize(
    struct LumNet_CommandResponse *const response )
{
    size_t encoded_response_size =
        offsetof( struct LumNet_CommandResponse, payload ) +
        response->commandHeader.payloadLength;

    return encoded_response_size;
}

size_t Command_CalculateLumNet_CommandResponseListPayloadSize(
    struct LumNet_CommandResponse *commands,
    size_t num_commands )
{
    assert( commands != NULL );

    size_t payload_size = 0;
    for ( size_t i = 0; i < num_commands; i++ )
    {
        payload_size += Command_CalculateLumNet_CommandResponseEncodeSize( &commands[i] );
    }

    return payload_size;
}

size_t Command_CalculateLumNet_CommandResponseListEncodeSize(
    struct LumNet_CommandResponseListPayload *const response_list )
{
    size_t encoded_list_size =
        offsetof( struct LumNet_CommandResponseListPayload, commands ) +
        response_list->responseListHeader.listPayloadLength;

    return encoded_list_size;
}

enum Common_returnCode Command_GetResponseCount(
    uint8_t const *encoded_response_list,
    int *response_count )
{
    assert( encoded_response_list );

    struct LumNet_CommandResponseListPayload *response_list = (struct LumNet_CommandResponseListPayload *)encoded_response_list;

    struct LumNet_CommandResponse *response = response_list->commands;

    size_t walked_bytes = 0;

    *response_count = 0;
    while ( walked_bytes < response_list->responseListHeader.listPayloadLength )
    {
        assert( response );

        *response_count = ( *response_count ) + 1;

        size_t offset = offsetof( struct LumNet_CommandResponse, payload ) + response->commandHeader.payloadLength;
        walked_bytes += offset;

        response = response_list->commands + walked_bytes;
    }

    return COMMAND_RETURN_CODE_SUCCESS;
}

enum Common_returnCode Command_GetResponseRange(
    int response_number,
    uint8_t const *encoded_response_list,
    size_t *start,
    size_t *end )
{
    assert( encoded_response_list );

    struct LumNet_CommandResponseListPayload *response_list = (struct LumNet_CommandResponseListPayload *)encoded_response_list;
    struct LumNet_CommandResponse *response = response_list->commands;

    size_t walked_bytes = 0;

    int current_number = 0;

    *start = *end = 0;
    while ( walked_bytes < response_list->responseListHeader.listPayloadLength )
    {
        assert( response );

        size_t offset = offsetof( struct LumNet_CommandResponse, payload ) + response->commandHeader.payloadLength;

        if ( current_number == response_number )
        {
            *start = walked_bytes;
            *end = walked_bytes + offset;
        }

        current_number++;

        walked_bytes += offset;
        response = response_list->commands + walked_bytes;
    }

    return COMMAND_RETURN_CODE_SUCCESS;
}

enum Common_returnCode Command_GetResponses(
    uint8_t const *encoded_response_list,
    struct LumNet_CommandResponse **decoded_responses,
    int *num_responses )
{
    assert( encoded_response_list );

    assert( decoded_responses );
    assert( *decoded_responses == NULL );

    Command_GetResponseCount( encoded_response_list,
                              num_responses );

    struct LumNet_CommandResponseListPayload *response_list = (struct LumNet_CommandResponseListPayload *)encoded_response_list;
    struct LumNet_CommandResponse *responses = response_list->commands;

    *decoded_responses = malloc( response_list->responseListHeader.listPayloadLength );
    memcpy( *decoded_responses, responses, response_list->responseListHeader.listPayloadLength );

    return COMMAND_RETURN_CODE_SUCCESS;
}
