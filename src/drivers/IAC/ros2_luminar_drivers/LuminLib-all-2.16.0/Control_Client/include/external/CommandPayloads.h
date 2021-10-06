/*
* CommandPayloads.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#ifndef CSC_COMMAND_PAYLOADS_H
#define CSC_COMMAND_PAYLOADS_H

// clang-format off

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief CommandPayloads
 *         This file provides functions for extracting payloads from responses.
 *
 */

#include "CommandLumNetDefinitions.h"

#include "LumNet/Command.h"

/*
* Extract the payload from the response of a(n) GenericScanProfile request
*/
struct LumNet_GenericScanProfile Command_GetGenericScanProfilePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) InterlacingConfiguration request
*/
struct LumNet_InterlacingConfiguration Command_GetInterlacingConfigurationPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) InterlockStates request
*/
struct LumNet_Interlocks Command_GetInterlockStatesPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) LidarDataEndpoint request
*/
struct LumNet_Endpoint Command_GetLidarDataEndpointPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) OptimizeSnapbacks request
*/
LumNet_Bool Command_GetOptimizeSnapbacksPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) PTPDelayInterval request
*/
LumNet_PtpTypeSigned Command_GetPTPDelayIntervalPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) PTPMode request
*/
LumNet_PtpMode Command_GetPTPModePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) PTPStatus request
*/
struct LumNet_PtpStatus Command_GetPTPStatusPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) ScanPlaylist request
*/
struct LumNet_ScanPlaylist Command_GetScanPlaylistPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) ScanningSynchronizedState request
*/
LumNet_Bool Command_GetScanningSynchronizedStatePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) SensorIPAddress request
*/
LumNet_IpAddress Command_GetSensorIPAddressPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) SensorLidarDataEnabled request
*/
LumNet_Bool Command_GetSensorLidarDataEnabledPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) SensorMacAddress request
*/
struct LumNet_MacAddress Command_GetSensorMacAddressPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) SensorNetworkType request
*/
LumNet_NetworkType Command_GetSensorNetworkTypePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) SensorNickname request
*/
LumNet_Nickname Command_GetSensorNicknamePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) SnapScanWholeNumberLinesEnabled request
*/
LumNet_Bool Command_GetSnapScanWholeNumberLinesEnabledPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) SystemTemperature request
*/
LumNet_SystemTemperature Command_GetSystemTemperaturePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) UDPStatusEnable request
*/
LumNet_Bool Command_GetUDPStatusEnablePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) UDPStatusEndpoint request
*/
struct LumNet_Endpoint Command_GetUDPStatusEndpointPayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) VerticalSyncMode request
*/
LumNet_VerticalSyncMode Command_GetVerticalSyncModePayload(struct LumNet_CommandResponse const* response);

/*
* Extract the payload from the response of a(n) VerticalSyncOffset request
*/
LumNet_VerticalSyncOffsetMs Command_GetVerticalSyncOffsetPayload(struct LumNet_CommandResponse const* response);

#ifdef __cplusplus
}
#endif

#endif
