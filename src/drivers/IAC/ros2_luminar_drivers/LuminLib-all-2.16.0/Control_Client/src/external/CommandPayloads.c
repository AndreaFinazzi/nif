/*
* CommandPayloads.c
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

// clang-format off

#include "CommandPayloads.h"

LumNet_Bool Command_GetOptimizeSnapbacksPayload(struct LumNet_CommandResponse const* response)
{
    LumNet_Bool temp = *((LumNet_Bool const*) response->payload);

    return temp;
}

LumNet_Bool Command_GetScanningSynchronizedStatePayload(struct LumNet_CommandResponse const* response)
{
    LumNet_Bool temp = *((LumNet_Bool const*) response->payload);

    return temp;
}

LumNet_Bool Command_GetSensorLidarDataEnabledPayload(struct LumNet_CommandResponse const* response)
{
    LumNet_Bool temp = *((LumNet_Bool const*) response->payload);

    return temp;
}

LumNet_Bool Command_GetSnapScanWholeNumberLinesEnabledPayload(struct LumNet_CommandResponse const* response)
{
    LumNet_Bool temp = *((LumNet_Bool const*) response->payload);

    return temp;
}

LumNet_Bool Command_GetUDPStatusEnablePayload(struct LumNet_CommandResponse const* response)
{
    LumNet_Bool temp = *((LumNet_Bool const*) response->payload);

    return temp;
}

LumNet_IpAddress Command_GetSensorIPAddressPayload(struct LumNet_CommandResponse const* response)
{
    LumNet_IpAddress temp = *((LumNet_IpAddress const*) response->payload);

    return temp;
}

LumNet_NetworkType Command_GetSensorNetworkTypePayload(struct LumNet_CommandResponse const* response)
{
    LumNet_NetworkType temp = *((LumNet_NetworkType const*) response->payload);

    return temp;
}

LumNet_Nickname Command_GetSensorNicknamePayload(struct LumNet_CommandResponse const* response)
{
    LumNet_Nickname temp = *((LumNet_Nickname const*) response->payload);

    return temp;
}

LumNet_PtpMode Command_GetPTPModePayload(struct LumNet_CommandResponse const* response)
{
    LumNet_PtpMode temp = *((LumNet_PtpMode const*) response->payload);

    return temp;
}

LumNet_PtpTypeSigned Command_GetPTPDelayIntervalPayload(struct LumNet_CommandResponse const* response)
{
    LumNet_PtpTypeSigned temp = *((LumNet_PtpTypeSigned const*) response->payload);

    return temp;
}

LumNet_SystemTemperature Command_GetSystemTemperaturePayload(struct LumNet_CommandResponse const* response)
{
    LumNet_SystemTemperature temp = *((LumNet_SystemTemperature const*) response->payload);

    return temp;
}

LumNet_VerticalSyncMode Command_GetVerticalSyncModePayload(struct LumNet_CommandResponse const* response)
{
    LumNet_VerticalSyncMode temp = *((LumNet_VerticalSyncMode const*) response->payload);

    return temp;
}

LumNet_VerticalSyncOffsetMs Command_GetVerticalSyncOffsetPayload(struct LumNet_CommandResponse const* response)
{
    LumNet_VerticalSyncOffsetMs temp = *((LumNet_VerticalSyncOffsetMs const*) response->payload);

    return temp;
}

struct LumNet_Endpoint Command_GetLidarDataEndpointPayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_Endpoint temp = *((struct LumNet_Endpoint const*) response->payload);

    return temp;
}

struct LumNet_Endpoint Command_GetUDPStatusEndpointPayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_Endpoint temp = *((struct LumNet_Endpoint const*) response->payload);

    return temp;
}

struct LumNet_GenericScanProfile Command_GetGenericScanProfilePayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_GenericScanProfile temp = *((struct LumNet_GenericScanProfile const*) response->payload);

    return temp;
}

struct LumNet_InterlacingConfiguration Command_GetInterlacingConfigurationPayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_InterlacingConfiguration temp = *((struct LumNet_InterlacingConfiguration const*) response->payload);

    return temp;
}

struct LumNet_Interlocks Command_GetInterlockStatesPayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_Interlocks temp = *((struct LumNet_Interlocks const*) response->payload);

    return temp;
}

struct LumNet_MacAddress Command_GetSensorMacAddressPayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_MacAddress temp = *((struct LumNet_MacAddress const*) response->payload);

    return temp;
}

struct LumNet_PtpStatus Command_GetPTPStatusPayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_PtpStatus temp = *((struct LumNet_PtpStatus const*) response->payload);

    return temp;
}

struct LumNet_ScanPlaylist Command_GetScanPlaylistPayload(struct LumNet_CommandResponse const* response)
{
    struct LumNet_ScanPlaylist temp = *((struct LumNet_ScanPlaylist const*) response->payload);

    return temp;
}
