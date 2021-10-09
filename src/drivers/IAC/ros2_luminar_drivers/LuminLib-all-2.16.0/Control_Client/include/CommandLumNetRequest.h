/*
* CommandLumNetRequest.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief CommandLumNetRequest
 *         This file provides functions for creating `LumNet_CommandRequest` and `LumNet_CommandRequestListPayload` types
 *
 */

#ifndef H_CSC_COMMAND_LUMNET_REQUEST_H
#define H_CSC_COMMAND_LUMNET_REQUEST_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include "CommandLumNetDefinitions.h"
#include "CommonDefinitions.h"
#include "LumNet/Command.h"

// +Requests

/*
* Creates a `LumNet_CommandRequestListPayload` from the parameters
* Makes a copy of the input `commands`
*/
struct LumNet_CommandRequestListPayload* Command_CreateCommandRequestList(
    struct LumNet_CommandRequest commands[],
    size_t num_commands,
    uint16_t listTransactionID,
    enum LegacyCommandProtocolSemVer version );

// Copies the input `request_list` into the provided buffer
enum Common_returnCode Command_CopyCommandRequestList(
    struct LumNet_CommandRequestListPayload* request_list,
    uint8_t** encoded_list,
    size_t* encoded_list_size );

// Copies the inputs into the already allocated `request`
enum Common_returnCode Command_CreateLumNetRequest_preallocated(
    struct LumNet_CommandRequest* request,
    uint16_t address,
    uint8_t operation,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload );

// Returns an allocated `LumNet_CommandRequest` initialized with the provided inputs
struct LumNet_CommandRequest* Command_CreateLumNetRequestNoPayload(
    uint16_t address,
    uint8_t operation,
    uint16_t transaction_id );

// Returns an allocated `LumNet_CommandRequest` initialized with the provided inputs
struct LumNet_CommandRequest* Command_CreateLumNetRequest(
    uint16_t address,
    uint8_t operation,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload );

// Copies the inputs into the preallocated `request`, allowing the caller to specify the "magic number"
enum Common_returnCode Command_CreateLumNetRequest_preallocated_internal(
    struct LumNet_CommandRequest* request,
    uint16_t address,
    uint8_t operation,
    uint8_t magic_number,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload );

// Returns an allocated `LumNet_CommandRequest` initialized with the provided inputs
struct LumNet_CommandRequest* Command_CreateLumNetRequest_internal(
    uint16_t address,
    uint8_t operation,
    uint8_t magic_number,
    uint16_t transaction_id,
    uint16_t payload_length,
    uint8_t const* payload );

// -Requests

#ifdef __cplusplus
}
#endif

#endif
