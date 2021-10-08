/*
* CommonDefinitions.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief CommonDefinitions
 *         Common definitions used through the C layer.
 *
 */

#ifndef H_CSC_COMMON_DEFINITIONS_H
#define H_CSC_COMMON_DEFINITIONS_H

enum Common_returnCode
{
    COMMAND_RETURN_CODE_SUCCESS = 0,
    COMMAND_RETURN_CODE_FAILURE = -1,
    COMMAND_RETURN_CODE_SENTINEL_MISMATCH = -2,
    COMMAND_RETURN_CODE_VERSION_MISMATCH = -3
};

#endif
