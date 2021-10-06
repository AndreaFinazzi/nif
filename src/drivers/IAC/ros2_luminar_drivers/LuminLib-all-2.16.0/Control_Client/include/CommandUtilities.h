/*
 * CommandUtilities.h
 *
 * Copyright (c) 2018, Luminar Technologies, Inc.
 *
 * This material contains confidential and trade secret information of Luminar Technologies.
 * Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
 * writing by Luminar Technologies.
 */

/*! \brief CommandUtilities
 *         This file provides misc. utility functions
 *
 */

#ifndef H_CSC_MODEL_H_COMMAND_UTILITIES_H
#define H_CSC_MODEL_H_COMMAND_UTILITIES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "CommandLumNetDefinitions.h"
#include "CommandLumNetRequest.h"
#include "LumNet/Common.h"

// Wraps a `Lumnet_CommandRequest` inside a `LumNet_CommandRequestListPayload` for transmission
//
enum Common_returnCode Command_CreateCommand();

enum Common_returnCode Command_WrapCommand( struct LumNet_CommandRequest* request,
                                            uint16_t listTransactionID,
                                            uint8_t** request_list_buffer,
                                            enum LegacyCommandProtocolSemVer version );

enum LUM_NET_CMD_RETURN_RESULT Command_ValidateScanProfile( struct LumNet_GenericScanProfile profile );

#ifdef __cplusplus
}
#endif

#endif
