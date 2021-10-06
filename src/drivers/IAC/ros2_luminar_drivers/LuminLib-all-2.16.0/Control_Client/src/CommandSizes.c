/*
* CommandSizes.c
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "CommandSizes.h"
#include "LumNet/Command.h"

#include "assert.h"

size_t Command_CalculateLumNet_CommandSize( struct LumNet_CommandRequest *const request )
{
    assert( request );

    size_t encoded_request_size =
        offsetof( struct LumNet_CommandRequest, payload ) +
        request->commandHeader.payloadLength;

    return encoded_request_size;
}

size_t Command_CalculateLumNet_CommandListPayloadSize(
    struct LumNet_CommandRequest *commands,
    size_t num_commands )
{
    assert( commands != NULL );

    size_t payload_size = 0;
    for ( size_t i = 0; i < num_commands; i++ )
    {
        payload_size += Command_CalculateLumNet_CommandSize( &commands[i] );
    }

    return payload_size;
}

size_t Command_CalculateLumNet_CommandListEncodeSize(
    struct LumNet_CommandRequestListPayload *const request_list )
{
    size_t encoded_list_size = offsetof( struct LumNet_CommandRequestListPayload, commands ) + request_list->commandRequestListHeader.listPayloadLength;

    return encoded_list_size;
}

size_t Command_CalculateLumNet_CommandListSize(
    struct LumNet_CommandRequest *commands,
    int num_commands )
{
    return offsetof( struct LumNet_CommandRequestListPayload, commands ) + Command_CalculateLumNet_CommandListPayloadSize( commands, num_commands );
}
