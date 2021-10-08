/*
* CommandSizes.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief CommandSizes
 *         This file provides functions for calculating the sizes of various kinds of request types.
 *
 */

#ifndef CSC_COMMAND_SIZES_H
#define CSC_COMMAND_SIZES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include "LumNet/Command.h"

size_t Command_CalculateLumNet_CommandSize(
    struct LumNet_CommandRequest *const request );

size_t Command_CalculateLumNet_CommandListPayloadSize(
    struct LumNet_CommandRequest *commands,
    size_t num_commands );

size_t Command_CalculateLumNet_CommandListSize(
    struct LumNet_CommandRequest *requests,
    int num_requests );

size_t Command_CalculateLumNet_CommandListEncodeSize(
    struct LumNet_CommandRequestListPayload *const request_list );

#ifdef __cplusplus
}
#endif

#endif
