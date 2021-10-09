/*
* CommandPredicates.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#ifndef CSC_COMMAND_PREDICATES_H
#define CSC_COMMAND_PREDICATES_H

// clang-format off

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief CommandPredicates
 *         This file provides functions for interrogating the information inside various response types.
 *
 */

#include "LumNet/Command.h"

#include <stdint.h>

int Command_IsRequestListPayload( uint8_t const* encoded_buffer );
int Command_IsValidationPayload( uint8_t const* encoded_buffer );
int Command_IsResponsePayload( uint8_t const* encoded_buffer );

int Command_SensorValidatedRequest( struct LumNet_CommandRequestListValidationPayload const* response );
int Command_SensorCompletedRequest( struct LumNet_CommandResponse const* response );

/*
* Test if the response is for a(n) GenericScanProfile request
*/
LumNet_Bool Command_IsGenericScanProfileCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) InterlacingConfiguration request
*/
LumNet_Bool Command_IsInterlacingConfigurationCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) InterlockStates request
*/
LumNet_Bool Command_IsInterlockStatesCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) LidarDataEndpoint request
*/
LumNet_Bool Command_IsLidarDataEndpointCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) OptimizeSnapbacks request
*/
LumNet_Bool Command_IsOptimizeSnapbacksCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) PTPDelayInterval request
*/
LumNet_Bool Command_IsPTPDelayIntervalCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) PTPMode request
*/
LumNet_Bool Command_IsPTPModeCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) PTPStatus request
*/
LumNet_Bool Command_IsPTPStatusCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) ScanPlaylist request
*/
LumNet_Bool Command_IsScanPlaylistCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) ScanningSynchronizedState request
*/
LumNet_Bool Command_IsScanningSynchronizedStateCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) SensorIPAddress request
*/
LumNet_Bool Command_IsSensorIPAddressCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) SensorLidarDataEnabled request
*/
LumNet_Bool Command_IsSensorLidarDataEnabledCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) SensorMacAddress request
*/
LumNet_Bool Command_IsSensorMacAddressCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) SensorNetworkType request
*/
LumNet_Bool Command_IsSensorNetworkTypeCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) SensorNickname request
*/
LumNet_Bool Command_IsSensorNicknameCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) SnapScanWholeNumberLinesEnabled request
*/
LumNet_Bool Command_IsSnapScanWholeNumberLinesEnabledCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) SystemTemperature request
*/
LumNet_Bool Command_IsSystemTemperatureCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) UDPStatusEnable request
*/
LumNet_Bool Command_IsUDPStatusEnableCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) UDPStatusEndpoint request
*/
LumNet_Bool Command_IsUDPStatusEndpointCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) VerticalSyncMode request
*/
LumNet_Bool Command_IsVerticalSyncModeCommandResponse(struct LumNet_CommandResponse const* response);

/*
* Test if the response is for a(n) VerticalSyncOffset request
*/
LumNet_Bool Command_IsVerticalSyncOffsetCommandResponse(struct LumNet_CommandResponse const* response);

#ifdef __cplusplus
}
#endif

#endif
