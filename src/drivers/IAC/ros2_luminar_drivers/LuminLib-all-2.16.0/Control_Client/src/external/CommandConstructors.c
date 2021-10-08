/*
* CommandConstructors.c
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

// clang-format off

#include "CommandConstructors.h"
#include "CommandUtilities.h"

#include <assert.h>
#include <stdlib.h>

const char* CSC_CommandTagStrings[] = {
     "CSC_ReadPersistentGenericScanProfile"

    ,"CSC_ReadPersistentInterlacingConfiguration"

    ,"CSC_ReadPersistentLidarDataEndpoint"

    ,"CSC_ReadPersistentOptimizeSnapbacks"

    ,"CSC_ReadPersistentPTPDelayInterval"

    ,"CSC_ReadPersistentPTPMode"

    ,"CSC_ReadPersistentScanPlaylist"

    ,"CSC_ReadPersistentSensorIPAddress"

    ,"CSC_ReadPersistentSensorLidarDataEnabled"

    ,"CSC_ReadPersistentSensorNetworkType"

    ,"CSC_ReadPersistentSensorNickname"

    ,"CSC_ReadPersistentSnapScanWholeNumberLinesEnabled"

    ,"CSC_ReadPersistentVerticalSyncMode"

    ,"CSC_ReadPersistentVerticalSyncOffset"

    ,"CSC_ReadVolatileGenericScanProfile"

    ,"CSC_ReadVolatileInterlacingConfiguration"

    ,"CSC_ReadVolatileInterlockStates"

    ,"CSC_ReadVolatileLidarDataEndpoint"

    ,"CSC_ReadVolatileOptimizeSnapbacks"

    ,"CSC_ReadVolatilePTPDelayInterval"

    ,"CSC_ReadVolatilePTPMode"

    ,"CSC_ReadVolatilePTPStatus"

    ,"CSC_ReadVolatileScanPlaylist"

    ,"CSC_ReadVolatileScanningSynchronizedState"

    ,"CSC_ReadVolatileSensorIPAddress"

    ,"CSC_ReadVolatileSensorLidarDataEnabled"

    ,"CSC_ReadVolatileSensorMacAddress"

    ,"CSC_ReadVolatileSensorNickname"

    ,"CSC_ReadVolatileSnapScanWholeNumberLinesEnabled"

    ,"CSC_ReadVolatileSystemTemperature"

    ,"CSC_ReadVolatileUDPStatusEnable"

    ,"CSC_ReadVolatileUDPStatusEndpoint"

    ,"CSC_ReadVolatileVerticalSyncMode"

    ,"CSC_ReadVolatileVerticalSyncOffset"

    ,"CSC_SetPersistentGenericScanProfile"

    ,"CSC_SetPersistentInterlacingConfiguration"

    ,"CSC_SetPersistentLidarDataEndpoint"

    ,"CSC_SetPersistentOptimizeSnapbacks"

    ,"CSC_SetPersistentPTPDelayInterval"

    ,"CSC_SetPersistentPTPMode"

    ,"CSC_SetPersistentScanPlaylist"

    ,"CSC_SetPersistentSensorIPAddress"

    ,"CSC_SetPersistentSensorLidarDataEnabled"

    ,"CSC_SetPersistentSensorNetworkType"

    ,"CSC_SetPersistentSensorNickname"

    ,"CSC_SetPersistentSnapScanWholeNumberLinesEnabled"

    ,"CSC_SetPersistentVerticalSyncMode"

    ,"CSC_SetPersistentVerticalSyncOffset"

    ,"CSC_SetVolatileGenericScanProfile"

    ,"CSC_SetVolatileInterlacingConfiguration"

    ,"CSC_SetVolatileLidarDataEndpoint"

    ,"CSC_SetVolatileOptimizeSnapbacks"

    ,"CSC_SetVolatilePTPDelayInterval"

    ,"CSC_SetVolatilePTPMode"

    ,"CSC_SetVolatileScanPlaylist"

    ,"CSC_SetVolatileSensorLidarDataEnabled"

    ,"CSC_SetVolatileSensorNickname"

    ,"CSC_SetVolatileSnapScanWholeNumberLinesEnabled"

    ,"CSC_SetVolatileUDPStatusEnable"

    ,"CSC_SetVolatileUDPStatusEndpoint"

    ,"CSC_SetVolatileVerticalSyncMode"

    ,"CSC_SetVolatileVerticalSyncOffset"

    ,"CSC_UnknownCommand"
};

const char* CSC_GetTagString(enum CSC_CommandTag tag)
{
  if (tag >= 0 && tag < CSC_NumCommandTags)
  {
    return CSC_CommandTagStrings[tag];
  }

  return CSC_CommandTagStrings[CSC_NumCommandTags];
}

enum Common_returnCode Command_CreateReadPersistentGenericScanProfileCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentInterlacingConfigurationCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_INTERLACING_CONFIGURATION),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentLidarDataEndpointCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENDPOINT,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentOptimizeSnapbacksCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentPTPDelayIntervalCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentPTPModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentScanPlaylistCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentSensorIPAddressCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_GEN_IP_ADDRESS,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentSensorLidarDataEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENABLE,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentSensorNetworkTypeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_GEN_NETWORK_TYPE,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentSensorNicknameCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_GEN, LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NICKNAME),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentSnapScanWholeNumberLinesEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentVerticalSyncModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadPersistentVerticalSyncOffsetCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileGenericScanProfileCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileInterlacingConfigurationCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_INTERLACING_CONFIGURATION),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileInterlockStatesCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_SYS, LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH, LUM_NET_CMD_ADDR_OFFSET_SYS_HEALTH_INTERLOCK_STATES),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileLidarDataEndpointCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENDPOINT,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileOptimizeSnapbacksCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatilePTPDelayIntervalCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatilePTPModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatilePTPStatusCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_STATUS),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileScanPlaylistCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileScanningSynchronizedStateCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_SYS, LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH, LUM_NET_CMD_ADDR_OFFSET_SYS_SCANNING_SYNCHRONIZED_STATE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileSensorIPAddressCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_GEN_IP_ADDRESS,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileSensorLidarDataEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENABLE,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileSensorMacAddressCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_ADDR_NET_GEN_MAC_ADDRESS,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileSensorNicknameCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_GEN, LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NICKNAME),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileSnapScanWholeNumberLinesEnabledCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileSystemTemperatureCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_SYS, LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH, LUM_NET_CMD_ADDR_OFFSET_SYS_HEALTH_SYSTEM_TEMPERATURE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileUDPStatusEnableCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_STATUS, LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENABLE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileUDPStatusEndpointCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_STATUS, LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENDPOINT),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileVerticalSyncModeCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateReadVolatileVerticalSyncOffsetCommand(
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequestNoPayload(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_READ, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(struct LumNet_GenericScanProfile),
        (uint8_t *) &scan_profile);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_INTERLACING_CONFIGURATION),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(struct LumNet_InterlacingConfiguration),
        (uint8_t *) &interlacing_configuration);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENDPOINT,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(struct LumNet_Endpoint),
        (uint8_t *) &endpoint);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_Bool),
        (uint8_t *) &optimize_snapbacks);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentPTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_PtpTypeSigned),
        (uint8_t *) &ptpDelayInterval);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentPTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_PtpMode),
        (uint8_t *) &ptpMode);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(struct LumNet_ScanPlaylist),
        (uint8_t *) &scanPlaylist);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentSensorIPAddressCommand(
    LumNet_IpAddress ip_address,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_ADDR_NET_GEN_IP_ADDRESS,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_IpAddress),
        (uint8_t *) &ip_address);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENABLE,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_Bool),
        (uint8_t *) &enable);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentSensorNetworkTypeCommand(
    LumNet_NetworkType network_type,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_ADDR_NET_GEN_NETWORK_TYPE,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_NetworkType),
        (uint8_t *) &network_type);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_GEN, LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NICKNAME),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_Nickname),
        (uint8_t *) &nickname);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_Bool),
        (uint8_t *) &snap_scan_whole_number_lines_enabled);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_VerticalSyncMode),
        (uint8_t *) &vertical_sync_mode);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetPersistentVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_PERSISTENT),
        transaction_id,
        sizeof(LumNet_VerticalSyncOffsetMs),
        (uint8_t *) &vertical_sync_offset_ms);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(struct LumNet_GenericScanProfile),
        (uint8_t *) &scan_profile);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_INTERLACING_CONFIGURATION),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(struct LumNet_InterlacingConfiguration),
        (uint8_t *) &interlacing_configuration);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENDPOINT,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(struct LumNet_Endpoint),
        (uint8_t *) &endpoint);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_Bool),
        (uint8_t *) &optimize_snapbacks);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatilePTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_PtpTypeSigned),
        (uint8_t *) &ptpDelayInterval);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatilePTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_PtpMode),
        (uint8_t *) &ptpMode);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(struct LumNet_ScanPlaylist),
        (uint8_t *) &scanPlaylist);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENABLE,
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_Bool),
        (uint8_t *) &enable);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_GEN, LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NICKNAME),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_Nickname),
        (uint8_t *) &nickname);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_Bool),
        (uint8_t *) &snap_scan_whole_number_lines_enabled);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileUDPStatusEnableCommand(
    LumNet_Bool status_enable,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_STATUS, LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENABLE),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_Bool),
        (uint8_t *) &status_enable);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileUDPStatusEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_STATUS, LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENDPOINT),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(struct LumNet_Endpoint),
        (uint8_t *) &endpoint);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_VerticalSyncMode),
        (uint8_t *) &vertical_sync_mode);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_CreateSetVolatileVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t transaction_id,
    struct LumNet_CommandRequest** request)
{
    assert(request != NULL);
    assert(*request == NULL);

    *request = Command_CreateLumNetRequest(
        LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET ),
        LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE, LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
        transaction_id,
        sizeof(LumNet_VerticalSyncOffsetMs),
        (uint8_t *) &vertical_sync_offset_ms);

    assert(*request);

    return *request != NULL ? COMMAND_RETURN_CODE_SUCCESS : COMMAND_RETURN_CODE_FAILURE;
}

enum Common_returnCode Command_EncodeReadPersistentGenericScanProfileCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentGenericScanProfileCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentInterlacingConfigurationCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentInterlacingConfigurationCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentLidarDataEndpointCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentLidarDataEndpointCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentOptimizeSnapbacksCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentOptimizeSnapbacksCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentPTPDelayIntervalCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentPTPDelayIntervalCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentPTPModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentPTPModeCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentScanPlaylistCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentScanPlaylistCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentSensorIPAddressCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentSensorIPAddressCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentSensorLidarDataEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentSensorLidarDataEnabledCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentSensorNetworkTypeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentSensorNetworkTypeCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentSensorNicknameCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentSensorNicknameCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentSnapScanWholeNumberLinesEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentSnapScanWholeNumberLinesEnabledCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentVerticalSyncModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentVerticalSyncModeCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadPersistentVerticalSyncOffsetCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadPersistentVerticalSyncOffsetCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileGenericScanProfileCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileGenericScanProfileCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileInterlacingConfigurationCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileInterlacingConfigurationCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileInterlockStatesCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileInterlockStatesCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileLidarDataEndpointCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileLidarDataEndpointCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileOptimizeSnapbacksCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileOptimizeSnapbacksCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatilePTPDelayIntervalCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatilePTPDelayIntervalCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatilePTPModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatilePTPModeCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatilePTPStatusCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatilePTPStatusCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileScanPlaylistCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileScanPlaylistCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileScanningSynchronizedStateCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileScanningSynchronizedStateCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileSensorIPAddressCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileSensorIPAddressCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileSensorLidarDataEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileSensorLidarDataEnabledCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileSensorMacAddressCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileSensorMacAddressCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileSensorNicknameCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileSensorNicknameCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileSnapScanWholeNumberLinesEnabledCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileSnapScanWholeNumberLinesEnabledCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileSystemTemperatureCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileSystemTemperatureCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileUDPStatusEnableCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileUDPStatusEnableCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileUDPStatusEndpointCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileUDPStatusEndpointCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileVerticalSyncModeCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileVerticalSyncModeCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeReadVolatileVerticalSyncOffsetCommand(
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateReadVolatileVerticalSyncOffsetCommand(
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentGenericScanProfileCommand(
        scan_profile,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentInterlacingConfigurationCommand(
        interlacing_configuration,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentLidarDataEndpointCommand(
        endpoint,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentOptimizeSnapbacksCommand(
        optimize_snapbacks,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentPTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentPTPDelayIntervalCommand(
        ptpDelayInterval,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentPTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentPTPModeCommand(
        ptpMode,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentScanPlaylistCommand(
        scanPlaylist,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentSensorIPAddressCommand(
    LumNet_IpAddress ip_address,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentSensorIPAddressCommand(
        ip_address,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentSensorLidarDataEnabledCommand(
        enable,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentSensorNetworkTypeCommand(
    LumNet_NetworkType network_type,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentSensorNetworkTypeCommand(
        network_type,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentSensorNicknameCommand(
        nickname,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentSnapScanWholeNumberLinesEnabledCommand(
        snap_scan_whole_number_lines_enabled,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentVerticalSyncModeCommand(
        vertical_sync_mode,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetPersistentVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetPersistentVerticalSyncOffsetCommand(
        vertical_sync_offset_ms,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileGenericScanProfileCommand(
    struct LumNet_GenericScanProfile scan_profile,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileGenericScanProfileCommand(
        scan_profile,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileInterlacingConfigurationCommand(
    struct LumNet_InterlacingConfiguration interlacing_configuration,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileInterlacingConfigurationCommand(
        interlacing_configuration,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileLidarDataEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileLidarDataEndpointCommand(
        endpoint,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileOptimizeSnapbacksCommand(
    LumNet_Bool optimize_snapbacks,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileOptimizeSnapbacksCommand(
        optimize_snapbacks,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatilePTPDelayIntervalCommand(
    LumNet_PtpTypeSigned ptpDelayInterval,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatilePTPDelayIntervalCommand(
        ptpDelayInterval,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatilePTPModeCommand(
    LumNet_PtpMode ptpMode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatilePTPModeCommand(
        ptpMode,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileScanPlaylistCommand(
    struct LumNet_ScanPlaylist scanPlaylist,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileScanPlaylistCommand(
        scanPlaylist,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileSensorLidarDataEnabledCommand(
    LumNet_Bool enable,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileSensorLidarDataEnabledCommand(
        enable,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileSensorNicknameCommand(
    LumNet_Nickname nickname,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileSensorNicknameCommand(
        nickname,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileSnapScanWholeNumberLinesEnabledCommand(
    LumNet_Bool snap_scan_whole_number_lines_enabled,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileSnapScanWholeNumberLinesEnabledCommand(
        snap_scan_whole_number_lines_enabled,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileUDPStatusEnableCommand(
    LumNet_Bool status_enable,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileUDPStatusEnableCommand(
        status_enable,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileUDPStatusEndpointCommand(
    struct LumNet_Endpoint endpoint,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileUDPStatusEndpointCommand(
        endpoint,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileVerticalSyncModeCommand(
    LumNet_VerticalSyncMode vertical_sync_mode,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileVerticalSyncModeCommand(
        vertical_sync_mode,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}

enum Common_returnCode Command_EncodeSetVolatileVerticalSyncOffsetCommand(
    LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms,
    uint16_t   transactionID,
    uint8_t**  request_list_buffer,
    enum LegacyCommandProtocolSemVer version)
{
    struct LumNet_CommandRequest* request = NULL;

    enum Common_returnCode status = Command_CreateSetVolatileVerticalSyncOffsetCommand(
        vertical_sync_offset_ms,
        transactionID,
        &request);

    assert(request);
    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    status = Command_WrapCommand(request,
                                 transactionID,
                                 request_list_buffer,
                                 version);

    assert(status == COMMAND_RETURN_CODE_SUCCESS);

    free(request);

    return status;
}