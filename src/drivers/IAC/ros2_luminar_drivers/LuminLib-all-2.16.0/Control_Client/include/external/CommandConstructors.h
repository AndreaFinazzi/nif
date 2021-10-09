/*
* CommandConstructors.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief CommandConstructors
 *         This file provides functions for creating `LumNet_CommandRequest` and `LumNet_CommandRequestListPayload` types
 *
 *  The interface for mapping command requests into transmission buffers is highly uniform. An example of their use is described below:
 *
 *   uint8_t* buffer = nullptr;
 *
 *   Command_EncodeSetVolatileTECLevelCommand(
 *       power,
 *       0,
 *       0,
 *       transactionID,
 *       &buffer);
 *
 *   size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);
 *
 *   mSocketService.SendCommand(sensor_fingerprint, buffer, buffer_size );
 *
 *   free(buffer);
 */

#ifndef H_CSC_MODEL_H_COMMANDS_H
#define H_CSC_MODEL_H_COMMANDS_H

// clang-format off

#ifdef __cplusplus
extern "C" {
#endif

#include "CommonDefinitions.h"
#include "CommandLumNetDefinitions.h"

#include "LumNet/Command.h"

#include <stdint.h>

extern const char* CSC_CommandTagStrings[];

static const int CSC_NumCommandTags = 62;

enum CSC_CommandTag {
     CSC_ReadPersistentGenericScanProfile

    ,CSC_ReadPersistentInterlacingConfiguration

    ,CSC_ReadPersistentLidarDataEndpoint

    ,CSC_ReadPersistentOptimizeSnapbacks

    ,CSC_ReadPersistentPTPDelayInterval

    ,CSC_ReadPersistentPTPMode

    ,CSC_ReadPersistentScanPlaylist

    ,CSC_ReadPersistentSensorIPAddress

    ,CSC_ReadPersistentSensorLidarDataEnabled

    ,CSC_ReadPersistentSensorNetworkType

    ,CSC_ReadPersistentSensorNickname

    ,CSC_ReadPersistentSnapScanWholeNumberLinesEnabled

    ,CSC_ReadPersistentVerticalSyncMode

    ,CSC_ReadPersistentVerticalSyncOffset

    ,CSC_ReadVolatileGenericScanProfile

    ,CSC_ReadVolatileInterlacingConfiguration

    ,CSC_ReadVolatileInterlockStates

    ,CSC_ReadVolatileLidarDataEndpoint

    ,CSC_ReadVolatileOptimizeSnapbacks

    ,CSC_ReadVolatilePTPDelayInterval

    ,CSC_ReadVolatilePTPMode

    ,CSC_ReadVolatilePTPStatus

    ,CSC_ReadVolatileScanPlaylist

    ,CSC_ReadVolatileScanningSynchronizedState

    ,CSC_ReadVolatileSensorIPAddress

    ,CSC_ReadVolatileSensorLidarDataEnabled

    ,CSC_ReadVolatileSensorMacAddress

    ,CSC_ReadVolatileSensorNickname

    ,CSC_ReadVolatileSnapScanWholeNumberLinesEnabled

    ,CSC_ReadVolatileSystemTemperature

    ,CSC_ReadVolatileUDPStatusEnable

    ,CSC_ReadVolatileUDPStatusEndpoint

    ,CSC_ReadVolatileVerticalSyncMode

    ,CSC_ReadVolatileVerticalSyncOffset

    ,CSC_SetPersistentGenericScanProfile

    ,CSC_SetPersistentInterlacingConfiguration

    ,CSC_SetPersistentLidarDataEndpoint

    ,CSC_SetPersistentOptimizeSnapbacks

    ,CSC_SetPersistentPTPDelayInterval

    ,CSC_SetPersistentPTPMode

    ,CSC_SetPersistentScanPlaylist

    ,CSC_SetPersistentSensorIPAddress

    ,CSC_SetPersistentSensorLidarDataEnabled

    ,CSC_SetPersistentSensorNetworkType

    ,CSC_SetPersistentSensorNickname

    ,CSC_SetPersistentSnapScanWholeNumberLinesEnabled

    ,CSC_SetPersistentVerticalSyncMode

    ,CSC_SetPersistentVerticalSyncOffset

    ,CSC_SetVolatileGenericScanProfile

    ,CSC_SetVolatileInterlacingConfiguration

    ,CSC_SetVolatileLidarDataEndpoint

    ,CSC_SetVolatileOptimizeSnapbacks

    ,CSC_SetVolatilePTPDelayInterval

    ,CSC_SetVolatilePTPMode

    ,CSC_SetVolatileScanPlaylist

    ,CSC_SetVolatileSensorLidarDataEnabled

    ,CSC_SetVolatileSensorNickname

    ,CSC_SetVolatileSnapScanWholeNumberLinesEnabled

    ,CSC_SetVolatileUDPStatusEnable

    ,CSC_SetVolatileUDPStatusEndpoint

    ,CSC_SetVolatileVerticalSyncMode

    ,CSC_SetVolatileVerticalSyncOffset
};

const char* CSC_GetTagString(enum CSC_CommandTag tag);

/*
* Create a ReadPersistentGenericScanProfileCommand request
*/
enum Common_returnCode Command_CreateReadPersistentGenericScanProfileCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentInterlacingConfigurationCommand request
*/
enum Common_returnCode Command_CreateReadPersistentInterlacingConfigurationCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentLidarDataEndpointCommand request
*/
enum Common_returnCode Command_CreateReadPersistentLidarDataEndpointCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentOptimizeSnapbacksCommand request
*/
enum Common_returnCode Command_CreateReadPersistentOptimizeSnapbacksCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentPTPDelayIntervalCommand request
*/
enum Common_returnCode Command_CreateReadPersistentPTPDelayIntervalCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentPTPModeCommand request
*/
enum Common_returnCode Command_CreateReadPersistentPTPModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentScanPlaylistCommand request
*/
enum Common_returnCode Command_CreateReadPersistentScanPlaylistCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentSensorIPAddressCommand request
*/
enum Common_returnCode Command_CreateReadPersistentSensorIPAddressCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentSensorLidarDataEnabledCommand request
*/
enum Common_returnCode Command_CreateReadPersistentSensorLidarDataEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentSensorNetworkTypeCommand request
*/
enum Common_returnCode Command_CreateReadPersistentSensorNetworkTypeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentSensorNicknameCommand request
*/
enum Common_returnCode Command_CreateReadPersistentSensorNicknameCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentSnapScanWholeNumberLinesEnabledCommand request
*/
enum Common_returnCode Command_CreateReadPersistentSnapScanWholeNumberLinesEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentVerticalSyncModeCommand request
*/
enum Common_returnCode Command_CreateReadPersistentVerticalSyncModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadPersistentVerticalSyncOffsetCommand request
*/
enum Common_returnCode Command_CreateReadPersistentVerticalSyncOffsetCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileGenericScanProfileCommand request
*/
enum Common_returnCode Command_CreateReadVolatileGenericScanProfileCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileInterlacingConfigurationCommand request
*/
enum Common_returnCode Command_CreateReadVolatileInterlacingConfigurationCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileInterlockStatesCommand request
*/
enum Common_returnCode Command_CreateReadVolatileInterlockStatesCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileLidarDataEndpointCommand request
*/
enum Common_returnCode Command_CreateReadVolatileLidarDataEndpointCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileOptimizeSnapbacksCommand request
*/
enum Common_returnCode Command_CreateReadVolatileOptimizeSnapbacksCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatilePTPDelayIntervalCommand request
*/
enum Common_returnCode Command_CreateReadVolatilePTPDelayIntervalCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatilePTPModeCommand request
*/
enum Common_returnCode Command_CreateReadVolatilePTPModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatilePTPStatusCommand request
*/
enum Common_returnCode Command_CreateReadVolatilePTPStatusCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileScanPlaylistCommand request
*/
enum Common_returnCode Command_CreateReadVolatileScanPlaylistCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileScanningSynchronizedStateCommand request
*/
enum Common_returnCode Command_CreateReadVolatileScanningSynchronizedStateCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileSensorIPAddressCommand request
*/
enum Common_returnCode Command_CreateReadVolatileSensorIPAddressCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileSensorLidarDataEnabledCommand request
*/
enum Common_returnCode Command_CreateReadVolatileSensorLidarDataEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileSensorMacAddressCommand request
*/
enum Common_returnCode Command_CreateReadVolatileSensorMacAddressCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileSensorNicknameCommand request
*/
enum Common_returnCode Command_CreateReadVolatileSensorNicknameCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileSnapScanWholeNumberLinesEnabledCommand request
*/
enum Common_returnCode Command_CreateReadVolatileSnapScanWholeNumberLinesEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileSystemTemperatureCommand request
*/
enum Common_returnCode Command_CreateReadVolatileSystemTemperatureCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileUDPStatusEnableCommand request
*/
enum Common_returnCode Command_CreateReadVolatileUDPStatusEnableCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileUDPStatusEndpointCommand request
*/
enum Common_returnCode Command_CreateReadVolatileUDPStatusEndpointCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileVerticalSyncModeCommand request
*/
enum Common_returnCode Command_CreateReadVolatileVerticalSyncModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a ReadVolatileVerticalSyncOffsetCommand request
*/
enum Common_returnCode Command_CreateReadVolatileVerticalSyncOffsetCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentGenericScanProfileCommand request
*/
enum Common_returnCode Command_CreateSetPersistentGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentInterlacingConfigurationCommand request
*/
enum Common_returnCode Command_CreateSetPersistentInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentLidarDataEndpointCommand request
*/
enum Common_returnCode Command_CreateSetPersistentLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentOptimizeSnapbacksCommand request
*/
enum Common_returnCode Command_CreateSetPersistentOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentPTPDelayIntervalCommand request
*/
enum Common_returnCode Command_CreateSetPersistentPTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentPTPModeCommand request
*/
enum Common_returnCode Command_CreateSetPersistentPTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentScanPlaylistCommand request
*/
enum Common_returnCode Command_CreateSetPersistentScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentSensorIPAddressCommand request
*/
enum Common_returnCode Command_CreateSetPersistentSensorIPAddressCommand(
    LumNet_IpAddress ip_address,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentSensorLidarDataEnabledCommand request
*/
enum Common_returnCode Command_CreateSetPersistentSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentSensorNetworkTypeCommand request
*/
enum Common_returnCode Command_CreateSetPersistentSensorNetworkTypeCommand(
    LumNet_NetworkType network_type,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentSensorNicknameCommand request
*/
enum Common_returnCode Command_CreateSetPersistentSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentSnapScanWholeNumberLinesEnabledCommand request
*/
enum Common_returnCode Command_CreateSetPersistentSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentVerticalSyncModeCommand request
*/
enum Common_returnCode Command_CreateSetPersistentVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetPersistentVerticalSyncOffsetCommand request
*/
enum Common_returnCode Command_CreateSetPersistentVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileGenericScanProfileCommand request
*/
enum Common_returnCode Command_CreateSetVolatileGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileInterlacingConfigurationCommand request
*/
enum Common_returnCode Command_CreateSetVolatileInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileLidarDataEndpointCommand request
*/
enum Common_returnCode Command_CreateSetVolatileLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileOptimizeSnapbacksCommand request
*/
enum Common_returnCode Command_CreateSetVolatileOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatilePTPDelayIntervalCommand request
*/
enum Common_returnCode Command_CreateSetVolatilePTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatilePTPModeCommand request
*/
enum Common_returnCode Command_CreateSetVolatilePTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileScanPlaylistCommand request
*/
enum Common_returnCode Command_CreateSetVolatileScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileSensorLidarDataEnabledCommand request
*/
enum Common_returnCode Command_CreateSetVolatileSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileSensorNicknameCommand request
*/
enum Common_returnCode Command_CreateSetVolatileSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileSnapScanWholeNumberLinesEnabledCommand request
*/
enum Common_returnCode Command_CreateSetVolatileSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileUDPStatusEnableCommand request
*/
enum Common_returnCode Command_CreateSetVolatileUDPStatusEnableCommand(
    LumNet_Bool status_enable,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileUDPStatusEndpointCommand request
*/
enum Common_returnCode Command_CreateSetVolatileUDPStatusEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileVerticalSyncModeCommand request
*/
enum Common_returnCode Command_CreateSetVolatileVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Create a SetVolatileVerticalSyncOffsetCommand request
*/
enum Common_returnCode Command_CreateSetVolatileVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request);

/*
* Wrap a ReadPersistentGenericScanProfileCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentGenericScanProfileCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentInterlacingConfigurationCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentInterlacingConfigurationCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentLidarDataEndpointCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentLidarDataEndpointCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentOptimizeSnapbacksCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentOptimizeSnapbacksCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentPTPDelayIntervalCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentPTPDelayIntervalCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentPTPModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentPTPModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentScanPlaylistCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentScanPlaylistCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentSensorIPAddressCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentSensorIPAddressCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentSensorLidarDataEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentSensorLidarDataEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentSensorNetworkTypeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentSensorNetworkTypeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentSensorNicknameCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentSensorNicknameCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentSnapScanWholeNumberLinesEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentSnapScanWholeNumberLinesEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentVerticalSyncModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentVerticalSyncModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadPersistentVerticalSyncOffsetCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadPersistentVerticalSyncOffsetCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileGenericScanProfileCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileGenericScanProfileCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileInterlacingConfigurationCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileInterlacingConfigurationCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileInterlockStatesCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileInterlockStatesCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileLidarDataEndpointCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileLidarDataEndpointCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileOptimizeSnapbacksCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileOptimizeSnapbacksCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatilePTPDelayIntervalCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatilePTPDelayIntervalCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatilePTPModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatilePTPModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatilePTPStatusCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatilePTPStatusCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileScanPlaylistCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileScanPlaylistCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileScanningSynchronizedStateCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileScanningSynchronizedStateCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileSensorIPAddressCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileSensorIPAddressCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileSensorLidarDataEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileSensorLidarDataEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileSensorMacAddressCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileSensorMacAddressCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileSensorNicknameCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileSensorNicknameCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileSnapScanWholeNumberLinesEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileSnapScanWholeNumberLinesEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileSystemTemperatureCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileSystemTemperatureCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileUDPStatusEnableCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileUDPStatusEnableCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileUDPStatusEndpointCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileUDPStatusEndpointCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileVerticalSyncModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileVerticalSyncModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a ReadVolatileVerticalSyncOffsetCommand request inside a list request
*/
enum Common_returnCode Command_EncodeReadVolatileVerticalSyncOffsetCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentGenericScanProfileCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentInterlacingConfigurationCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentLidarDataEndpointCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentOptimizeSnapbacksCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentPTPDelayIntervalCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentPTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentPTPModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentPTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentScanPlaylistCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentSensorIPAddressCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentSensorIPAddressCommand(
    LumNet_IpAddress ip_address,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentSensorLidarDataEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentSensorNetworkTypeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentSensorNetworkTypeCommand(
    LumNet_NetworkType network_type,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentSensorNicknameCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentSnapScanWholeNumberLinesEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentVerticalSyncModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetPersistentVerticalSyncOffsetCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetPersistentVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileGenericScanProfileCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileInterlacingConfigurationCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileLidarDataEndpointCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileOptimizeSnapbacksCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatilePTPDelayIntervalCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatilePTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatilePTPModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatilePTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileScanPlaylistCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileSensorLidarDataEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileSensorNicknameCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileSnapScanWholeNumberLinesEnabledCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileUDPStatusEnableCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileUDPStatusEnableCommand(
    LumNet_Bool status_enable,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileUDPStatusEndpointCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileUDPStatusEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileVerticalSyncModeCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

/*
* Wrap a SetVolatileVerticalSyncOffsetCommand request inside a list request
*/
enum Common_returnCode Command_EncodeSetVolatileVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version);

#ifdef __cplusplus
}
#endif

#endif
