/*
* CommandLumNetResponse.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief CommandLumNetResponse
 *         This file provides functions for working with `LumNet_CommandResponse` and `LumNet_CommandResponseListPayload` types
 *
 */

#ifndef H_CSC_COMMAND_LUMNET_H
#define H_CSC_COMMAND_LUMNET_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include "CommonDefinitions.h"
#include "LumNet/Command.h"

size_t Command_CalculateLumNet_CommandResponseEncodeSize(
    struct LumNet_CommandResponse *const response );

size_t Command_CalculateLumNet_CommandResponseListPayloadSize(
    struct LumNet_CommandResponse *commands,
    size_t num_commands );

size_t Command_CalculateLumNet_CommandResponseListEncodeSize(
    struct LumNet_CommandResponseListPayload *const response_list );

enum Common_returnCode Command_GetResponseCount(
    uint8_t const *encoded_response_list,
    int *response_count );

enum Common_returnCode Command_GetResponseRange(
    int response_number,
    uint8_t const *encoded_response_list,
    size_t *start,
    size_t *end );

enum Common_returnCode Command_GetResponses(
    uint8_t const *encoded_response_list,
    struct LumNet_CommandResponse **responses,
    int *num_responses );

#ifdef __cplusplus
}
#endif

#endif
