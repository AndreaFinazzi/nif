/*
* CommandPredicates.c
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

// clang-format off

#include "CommandPredicates.h"

#include "assert.h"

int Command_IsRequestListPayload( uint8_t const* encoded_buffer )
{
    assert( encoded_buffer );
    struct LumNet_CommandListPreamble const* preamble = (struct LumNet_CommandListPreamble const*)encoded_buffer;

    return preamble->versionPayloadHeader.semVersionedHeader.payloadTypeID == LUM_NET_PAYLOAD_TYPE_ID_CMD_REQUEST_LIST;
}

int Command_IsValidationPayload( uint8_t const* encoded_buffer )
{
    assert( encoded_buffer );
    struct LumNet_CommandListPreamble const* preamble = (struct LumNet_CommandListPreamble const*)encoded_buffer;

    return preamble->versionPayloadHeader.semVersionedHeader.payloadTypeID == LUM_NET_PAYLOAD_TYPE_ID_CMD_REQUEST_LIST_RETURN_RESULT;
}

int Command_IsResponsePayload( uint8_t const* encoded_buffer )
{
    assert( encoded_buffer );
    struct LumNet_CommandListPreamble const* preamble = (struct LumNet_CommandListPreamble const*)encoded_buffer;

    return preamble->versionPayloadHeader.semVersionedHeader.payloadTypeID == LUM_NET_PAYLOAD_TYPE_ID_CMD_RESPONSE_LIST;
}

int Command_SensorValidatedRequest( struct LumNet_CommandRequestListValidationPayload const* response )
{
    assert( response );
    return response->commandRequestListReturnResultHeader.returnResult == LUM_NET_CMD_RESPONSE_LIST_RETURN_RESULT_SUCCESS;
}

int Command_SensorCompletedRequest( struct LumNet_CommandResponse const* response )
{
    assert( response );
    return response->commandHeader.commandReturnResult == LUM_NET_CMD_RETURN_RESULT_SUCCESS;
}

LumNet_Bool Command_IsGenericScanProfileCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE);
}

LumNet_Bool Command_IsInterlacingConfigurationCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_INTERLACING_CONFIGURATION);
}

LumNet_Bool Command_IsInterlockStatesCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_SYS, LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH, LUM_NET_CMD_ADDR_OFFSET_SYS_HEALTH_INTERLOCK_STATES);
}

LumNet_Bool Command_IsLidarDataEndpointCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENDPOINT;
}

LumNet_Bool Command_IsOptimizeSnapbacksCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS);
}

LumNet_Bool Command_IsPTPDelayIntervalCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL);
}

LumNet_Bool Command_IsPTPModeCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE);
}

LumNet_Bool Command_IsPTPStatusCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_PTP, LUM_NET_CMD_ADDR_OFFSET_NET_PTP_STATUS);
}

LumNet_Bool Command_IsScanPlaylistCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST);
}

LumNet_Bool Command_IsScanningSynchronizedStateCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_SYS, LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH, LUM_NET_CMD_ADDR_OFFSET_SYS_SCANNING_SYNCHRONIZED_STATE);
}

LumNet_Bool Command_IsSensorIPAddressCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_ADDR_NET_GEN_IP_ADDRESS;
}

LumNet_Bool Command_IsSensorLidarDataEnabledCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENABLE;
}

LumNet_Bool Command_IsSensorMacAddressCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_ADDR_NET_GEN_MAC_ADDRESS;
}

LumNet_Bool Command_IsSensorNetworkTypeCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_ADDR_NET_GEN_NETWORK_TYPE;
}

LumNet_Bool Command_IsSensorNicknameCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_GEN, LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NICKNAME);
}

LumNet_Bool Command_IsSnapScanWholeNumberLinesEnabledCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES);
}

LumNet_Bool Command_IsSystemTemperatureCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_SYS, LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH, LUM_NET_CMD_ADDR_OFFSET_SYS_HEALTH_SYSTEM_TEMPERATURE);
}

LumNet_Bool Command_IsUDPStatusEnableCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_STATUS, LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENABLE);
}

LumNet_Bool Command_IsUDPStatusEndpointCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_STATUS, LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENDPOINT);
}

LumNet_Bool Command_IsVerticalSyncModeCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE );
}

LumNet_Bool Command_IsVerticalSyncOffsetCommandResponse(struct LumNet_CommandResponse const* response)
{
    return response->commandHeader.address == LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER, LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET );
}
