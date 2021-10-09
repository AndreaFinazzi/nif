/*
 * ModelHControllerCommands.cpp
 *
 * Copyright (c) 2018, Luminar Technologies, Inc.
 *
 * This material contains confidential and trade secret information of Luminar Technologies.
 * Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
 * writing by Luminar Technologies.
 */

// clang-format off

#include "ModelHControllerCommands.h"

#include "CommandPayloads.h"
#include "CommandPredicates.h"
#include "ModelHCommandClientSubscriber.h"

#include "CommandSizes.h"
#include "DiscoveryResponse.h"

#include "LumNet/Command.h"

#include <assert.h>

using namespace lum;

bool ModelHControllerCommands::SendCommand( lum::DiscoveryResponse commandedSensor, uint8_t* command, size_t command_size )
{
    assert( false );
    return false;
}

TransactionID ModelHControllerCommands::NextTransactionId()
{
    assert( false );
    return 0;
}

LegacyCommandProtocolSemVer ModelHControllerCommands::GetCurrentSemVer()
{
    assert( false );
    return LegacyCommandProtocolSemVer::COMMAND_PROTOCOL_SEM_VER_LATEST;
}

void ModelHControllerCommands::HandleCommandResponse( ModelHCommandClientSubscriber* subscriber, FingerprintID fingerprint, uint8_t* response )
{
    assert( subscriber );

    if ( Command_IsValidationPayload( response ) )
    {
        const LumNet_CommandRequestListValidationPayload* payload = reinterpret_cast<LumNet_CommandRequestListValidationPayload*>( response );
        assert( payload );

        const TransactionID transactionID = payload->commandRequestListReturnResultHeader.listTransactionID;

        const CSCTransactionStatus transactionStatus = Command_SensorValidatedRequest( payload ) ? VALIDATED : FAILED_TO_VALIDATE;

        if ( subscriber )
        {
            subscriber->TransactionStatus( fingerprint, transactionID, transactionStatus, response );
        }

        return;
    }

    const LumNet_CommandResponseListPayload* payload = reinterpret_cast<LumNet_CommandResponseListPayload*>( response );
    assert( payload );

    const uint16_t transactionID = payload->commands->commandHeader.transactionID;

    // This does not handle multiple command lists
    const LumNet_CommandResponse* command_response = payload->commands;

    assert( command_response );

    if ( !Command_SensorCompletedRequest( command_response ) )
    {
        subscriber->TransactionStatus( fingerprint, transactionID, FAILED_TO_COMPLETE, response );
        return;
    }

    if(Command_IsGenericScanProfileCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->GenericScanProfileUpdated(fingerprint, transactionID, Command_GetGenericScanProfilePayload(command_response));
        }
    }

    if(Command_IsInterlacingConfigurationCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->InterlacingConfigurationUpdated(fingerprint, transactionID, Command_GetInterlacingConfigurationPayload(command_response));
        }
    }

    if(Command_IsInterlockStatesCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->InterlockStatesUpdated(fingerprint, transactionID, Command_GetInterlockStatesPayload(command_response));
        }
    }

    if(Command_IsLidarDataEndpointCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->LidarDataEndpointUpdated(fingerprint, transactionID, Command_GetLidarDataEndpointPayload(command_response));
        }
    }

    if(Command_IsOptimizeSnapbacksCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->OptimizeSnapbacksUpdated(fingerprint, transactionID, Command_GetOptimizeSnapbacksPayload(command_response));
        }
    }

    if(Command_IsPTPDelayIntervalCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->PTPDelayIntervalUpdated(fingerprint, transactionID, Command_GetPTPDelayIntervalPayload(command_response));
        }
    }

    if(Command_IsPTPModeCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->PTPModeUpdated(fingerprint, transactionID, Command_GetPTPModePayload(command_response));
        }
    }

    if(Command_IsPTPStatusCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->PTPStatusUpdated(fingerprint, transactionID, Command_GetPTPStatusPayload(command_response));
        }
    }

    if(Command_IsScanPlaylistCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->ScanPlaylistUpdated(fingerprint, transactionID, Command_GetScanPlaylistPayload(command_response));
        }
    }

    if(Command_IsScanningSynchronizedStateCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->ScanningSynchronizedStateUpdated(fingerprint, transactionID, Command_GetScanningSynchronizedStatePayload(command_response));
        }
    }

    if(Command_IsSensorIPAddressCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->SensorIPAddressUpdated(fingerprint, transactionID, Command_GetSensorIPAddressPayload(command_response));
        }
    }

    if(Command_IsSensorLidarDataEnabledCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->SensorLidarDataEnabledUpdated(fingerprint, transactionID, Command_GetSensorLidarDataEnabledPayload(command_response));
        }
    }

    if(Command_IsSensorMacAddressCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->SensorMacAddressUpdated(fingerprint, transactionID, Command_GetSensorMacAddressPayload(command_response));
        }
    }

    if(Command_IsSensorNetworkTypeCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->SensorNetworkTypeUpdated(fingerprint, transactionID, Command_GetSensorNetworkTypePayload(command_response));
        }
    }

    if(Command_IsSensorNicknameCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->SensorNicknameUpdated(fingerprint, transactionID, Command_GetSensorNicknamePayload(command_response));
        }
    }

    if(Command_IsSnapScanWholeNumberLinesEnabledCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->SnapScanWholeNumberLinesEnabledUpdated(fingerprint, transactionID, Command_GetSnapScanWholeNumberLinesEnabledPayload(command_response));
        }
    }

    if(Command_IsSystemTemperatureCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->SystemTemperatureUpdated(fingerprint, transactionID, Command_GetSystemTemperaturePayload(command_response));
        }
    }

    if(Command_IsUDPStatusEnableCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->UDPStatusEnableUpdated(fingerprint, transactionID, Command_GetUDPStatusEnablePayload(command_response));
        }
    }

    if(Command_IsUDPStatusEndpointCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->UDPStatusEndpointUpdated(fingerprint, transactionID, Command_GetUDPStatusEndpointPayload(command_response));
        }
    }

    if(Command_IsVerticalSyncModeCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->VerticalSyncModeUpdated(fingerprint, transactionID, Command_GetVerticalSyncModePayload(command_response));
        }
    }

    if(Command_IsVerticalSyncOffsetCommandResponse(command_response))
    {
        if (subscriber)
        {
            subscriber->VerticalSyncOffsetUpdated(fingerprint, transactionID, Command_GetVerticalSyncOffsetPayload(command_response));
        }
    }

    subscriber->TransactionStatus( fingerprint, transactionID, COMPLETED, response );
}

TransactionID ModelHControllerCommands::ReadPersistentGenericScanProfile(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentGenericScanProfileCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentGenericScanProfileStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentGenericScanProfile(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentGenericScanProfile");
}

TransactionID ModelHControllerCommands::ReadPersistentInterlacingConfiguration(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentInterlacingConfigurationCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentInterlacingConfigurationStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentInterlacingConfiguration(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentInterlacingConfiguration");
}

TransactionID ModelHControllerCommands::ReadPersistentLidarDataEndpoint(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentLidarDataEndpointCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentLidarDataEndpointStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentLidarDataEndpoint(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentLidarDataEndpoint");
}

TransactionID ModelHControllerCommands::ReadPersistentOptimizeSnapbacks(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentOptimizeSnapbacksCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentOptimizeSnapbacksStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentOptimizeSnapbacks(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentOptimizeSnapbacks");
}

TransactionID ModelHControllerCommands::ReadPersistentPTPDelayInterval(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentPTPDelayIntervalCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentPTPDelayIntervalStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentPTPDelayInterval(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentPTPDelayInterval");
}

TransactionID ModelHControllerCommands::ReadPersistentPTPMode(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentPTPModeCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentPTPModeStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentPTPMode(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentPTPMode");
}

TransactionID ModelHControllerCommands::ReadPersistentScanPlaylist(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentScanPlaylistCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentScanPlaylistStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentScanPlaylist(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentScanPlaylist");
}

TransactionID ModelHControllerCommands::ReadPersistentSensorIPAddress(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentSensorIPAddressCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentSensorIPAddressStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentSensorIPAddress(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentSensorIPAddress");
}

TransactionID ModelHControllerCommands::ReadPersistentSensorLidarDataEnabled(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentSensorLidarDataEnabledCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentSensorLidarDataEnabledStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentSensorLidarDataEnabled(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentSensorLidarDataEnabled");
}

TransactionID ModelHControllerCommands::ReadPersistentSensorNetworkType(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentSensorNetworkTypeCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentSensorNetworkTypeStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentSensorNetworkType(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentSensorNetworkType");
}

TransactionID ModelHControllerCommands::ReadPersistentSensorNickname(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentSensorNicknameCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentSensorNicknameStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentSensorNickname(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentSensorNickname");
}

TransactionID ModelHControllerCommands::ReadPersistentSnapScanWholeNumberLinesEnabled(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentSnapScanWholeNumberLinesEnabledCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentSnapScanWholeNumberLinesEnabledStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentSnapScanWholeNumberLinesEnabled(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentSnapScanWholeNumberLinesEnabled");
}

TransactionID ModelHControllerCommands::ReadPersistentVerticalSyncMode(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentVerticalSyncModeCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentVerticalSyncModeStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentVerticalSyncMode(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentVerticalSyncMode");
}

TransactionID ModelHControllerCommands::ReadPersistentVerticalSyncOffset(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadPersistentVerticalSyncOffsetCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadPersistentVerticalSyncOffsetStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadPersistentVerticalSyncOffset(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadPersistentVerticalSyncOffset");
}

TransactionID ModelHControllerCommands::ReadVolatileGenericScanProfile(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileGenericScanProfileCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileGenericScanProfileStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileGenericScanProfile(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileGenericScanProfile");
}

TransactionID ModelHControllerCommands::ReadVolatileInterlacingConfiguration(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileInterlacingConfigurationCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileInterlacingConfigurationStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileInterlacingConfiguration(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileInterlacingConfiguration");
}

TransactionID ModelHControllerCommands::ReadVolatileInterlockStates(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileInterlockStatesCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileInterlockStatesStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileInterlockStates(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileInterlockStates");
}

TransactionID ModelHControllerCommands::ReadVolatileLidarDataEndpoint(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileLidarDataEndpointCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileLidarDataEndpointStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileLidarDataEndpoint(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileLidarDataEndpoint");
}

TransactionID ModelHControllerCommands::ReadVolatileOptimizeSnapbacks(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileOptimizeSnapbacksCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileOptimizeSnapbacksStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileOptimizeSnapbacks(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileOptimizeSnapbacks");
}

TransactionID ModelHControllerCommands::ReadVolatilePTPDelayInterval(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatilePTPDelayIntervalCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatilePTPDelayIntervalStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatilePTPDelayInterval(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatilePTPDelayInterval");
}

TransactionID ModelHControllerCommands::ReadVolatilePTPMode(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatilePTPModeCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatilePTPModeStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatilePTPMode(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatilePTPMode");
}

TransactionID ModelHControllerCommands::ReadVolatilePTPStatus(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatilePTPStatusCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatilePTPStatusStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatilePTPStatus(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatilePTPStatus");
}

TransactionID ModelHControllerCommands::ReadVolatileScanPlaylist(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileScanPlaylistCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileScanPlaylistStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileScanPlaylist(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileScanPlaylist");
}

TransactionID ModelHControllerCommands::ReadVolatileScanningSynchronizedState(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileScanningSynchronizedStateCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileScanningSynchronizedStateStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileScanningSynchronizedState(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileScanningSynchronizedState");
}

TransactionID ModelHControllerCommands::ReadVolatileSensorIPAddress(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileSensorIPAddressCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileSensorIPAddressStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileSensorIPAddress(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileSensorIPAddress");
}

TransactionID ModelHControllerCommands::ReadVolatileSensorLidarDataEnabled(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileSensorLidarDataEnabledCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileSensorLidarDataEnabledStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileSensorLidarDataEnabled(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileSensorLidarDataEnabled");
}

TransactionID ModelHControllerCommands::ReadVolatileSensorMacAddress(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileSensorMacAddressCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileSensorMacAddressStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileSensorMacAddress(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileSensorMacAddress");
}

TransactionID ModelHControllerCommands::ReadVolatileSensorNickname(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileSensorNicknameCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileSensorNicknameStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileSensorNickname(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileSensorNickname");
}

TransactionID ModelHControllerCommands::ReadVolatileSnapScanWholeNumberLinesEnabled(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileSnapScanWholeNumberLinesEnabledCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileSnapScanWholeNumberLinesEnabledStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileSnapScanWholeNumberLinesEnabled(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileSnapScanWholeNumberLinesEnabled");
}

TransactionID ModelHControllerCommands::ReadVolatileSystemTemperature(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileSystemTemperatureCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileSystemTemperatureStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileSystemTemperature(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileSystemTemperature");
}

TransactionID ModelHControllerCommands::ReadVolatileUDPStatusEnable(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileUDPStatusEnableCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileUDPStatusEnableStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileUDPStatusEnable(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileUDPStatusEnable");
}

TransactionID ModelHControllerCommands::ReadVolatileUDPStatusEndpoint(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileUDPStatusEndpointCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileUDPStatusEndpointStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileUDPStatusEndpoint(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileUDPStatusEndpoint");
}

TransactionID ModelHControllerCommands::ReadVolatileVerticalSyncMode(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileVerticalSyncModeCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileVerticalSyncModeStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileVerticalSyncMode(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileVerticalSyncMode");
}

TransactionID ModelHControllerCommands::ReadVolatileVerticalSyncOffset(lum::DiscoveryResponse commandedSensor)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeReadVolatileVerticalSyncOffsetCommand(
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::ReadVolatileVerticalSyncOffsetStringified(lum::DiscoveryResponse commandedSensor)
{
    const TransactionID transactionID = ReadVolatileVerticalSyncOffset(commandedSensor);
    return std::make_pair(transactionID, "CSC_ReadVolatileVerticalSyncOffset");
}

TransactionID ModelHControllerCommands::SetPersistentGenericScanProfile(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentGenericScanProfileCommand(
        scan_profile,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentGenericScanProfileStringified(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile)
{
    const TransactionID transactionID = SetPersistentGenericScanProfile(commandedSensor, scan_profile);
    return std::make_pair(transactionID, "CSC_SetPersistentGenericScanProfile");
}

TransactionID ModelHControllerCommands::SetPersistentInterlacingConfiguration(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentInterlacingConfigurationCommand(
        interlacing_configuration,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentInterlacingConfigurationStringified(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration)
{
    const TransactionID transactionID = SetPersistentInterlacingConfiguration(commandedSensor, interlacing_configuration);
    return std::make_pair(transactionID, "CSC_SetPersistentInterlacingConfiguration");
}

TransactionID ModelHControllerCommands::SetPersistentLidarDataEndpoint(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentLidarDataEndpointCommand(
        endpoint,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentLidarDataEndpointStringified(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint)
{
    const TransactionID transactionID = SetPersistentLidarDataEndpoint(commandedSensor, endpoint);
    return std::make_pair(transactionID, "CSC_SetPersistentLidarDataEndpoint");
}

TransactionID ModelHControllerCommands::SetPersistentOptimizeSnapbacks(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentOptimizeSnapbacksCommand(
        optimize_snapbacks,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentOptimizeSnapbacksStringified(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks)
{
    const TransactionID transactionID = SetPersistentOptimizeSnapbacks(commandedSensor, optimize_snapbacks);
    return std::make_pair(transactionID, "CSC_SetPersistentOptimizeSnapbacks");
}

TransactionID ModelHControllerCommands::SetPersistentPTPDelayInterval(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentPTPDelayIntervalCommand(
        ptpDelayInterval,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentPTPDelayIntervalStringified(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval)
{
    const TransactionID transactionID = SetPersistentPTPDelayInterval(commandedSensor, ptpDelayInterval);
    return std::make_pair(transactionID, "CSC_SetPersistentPTPDelayInterval");
}

TransactionID ModelHControllerCommands::SetPersistentPTPMode(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentPTPModeCommand(
        ptpMode,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentPTPModeStringified(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode)
{
    const TransactionID transactionID = SetPersistentPTPMode(commandedSensor, ptpMode);
    return std::make_pair(transactionID, "CSC_SetPersistentPTPMode");
}

TransactionID ModelHControllerCommands::SetPersistentScanPlaylist(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentScanPlaylistCommand(
        scanPlaylist,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentScanPlaylistStringified(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist)
{
    const TransactionID transactionID = SetPersistentScanPlaylist(commandedSensor, scanPlaylist);
    return std::make_pair(transactionID, "CSC_SetPersistentScanPlaylist");
}

TransactionID ModelHControllerCommands::SetPersistentSensorIPAddress(DiscoveryResponse commandedSensor, LumNet_IpAddress ip_address)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentSensorIPAddressCommand(
        ip_address,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentSensorIPAddressStringified(DiscoveryResponse commandedSensor, LumNet_IpAddress ip_address)
{
    const TransactionID transactionID = SetPersistentSensorIPAddress(commandedSensor, ip_address);
    return std::make_pair(transactionID, "CSC_SetPersistentSensorIPAddress");
}

TransactionID ModelHControllerCommands::SetPersistentSensorLidarDataEnabled(DiscoveryResponse commandedSensor, LumNet_Bool enable)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentSensorLidarDataEnabledCommand(
        enable,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentSensorLidarDataEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool enable)
{
    const TransactionID transactionID = SetPersistentSensorLidarDataEnabled(commandedSensor, enable);
    return std::make_pair(transactionID, "CSC_SetPersistentSensorLidarDataEnabled");
}

TransactionID ModelHControllerCommands::SetPersistentSensorNetworkType(DiscoveryResponse commandedSensor, LumNet_NetworkType network_type)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentSensorNetworkTypeCommand(
        network_type,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentSensorNetworkTypeStringified(DiscoveryResponse commandedSensor, LumNet_NetworkType network_type)
{
    const TransactionID transactionID = SetPersistentSensorNetworkType(commandedSensor, network_type);
    return std::make_pair(transactionID, "CSC_SetPersistentSensorNetworkType");
}

TransactionID ModelHControllerCommands::SetPersistentSensorNickname(DiscoveryResponse commandedSensor, LumNet_Nickname nickname)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentSensorNicknameCommand(
        nickname,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentSensorNicknameStringified(DiscoveryResponse commandedSensor, LumNet_Nickname nickname)
{
    const TransactionID transactionID = SetPersistentSensorNickname(commandedSensor, nickname);
    return std::make_pair(transactionID, "CSC_SetPersistentSensorNickname");
}

TransactionID ModelHControllerCommands::SetPersistentSnapScanWholeNumberLinesEnabled(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentSnapScanWholeNumberLinesEnabledCommand(
        snap_scan_whole_number_lines_enabled,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentSnapScanWholeNumberLinesEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled)
{
    const TransactionID transactionID = SetPersistentSnapScanWholeNumberLinesEnabled(commandedSensor, snap_scan_whole_number_lines_enabled);
    return std::make_pair(transactionID, "CSC_SetPersistentSnapScanWholeNumberLinesEnabled");
}

TransactionID ModelHControllerCommands::SetPersistentVerticalSyncMode(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentVerticalSyncModeCommand(
        vertical_sync_mode,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentVerticalSyncModeStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode)
{
    const TransactionID transactionID = SetPersistentVerticalSyncMode(commandedSensor, vertical_sync_mode);
    return std::make_pair(transactionID, "CSC_SetPersistentVerticalSyncMode");
}

TransactionID ModelHControllerCommands::SetPersistentVerticalSyncOffset(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetPersistentVerticalSyncOffsetCommand(
        vertical_sync_offset_ms,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetPersistentVerticalSyncOffsetStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms)
{
    const TransactionID transactionID = SetPersistentVerticalSyncOffset(commandedSensor, vertical_sync_offset_ms);
    return std::make_pair(transactionID, "CSC_SetPersistentVerticalSyncOffset");
}

TransactionID ModelHControllerCommands::SetVolatileGenericScanProfile(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileGenericScanProfileCommand(
        scan_profile,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileGenericScanProfileStringified(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile)
{
    const TransactionID transactionID = SetVolatileGenericScanProfile(commandedSensor, scan_profile);
    return std::make_pair(transactionID, "CSC_SetVolatileGenericScanProfile");
}

TransactionID ModelHControllerCommands::SetVolatileInterlacingConfiguration(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileInterlacingConfigurationCommand(
        interlacing_configuration,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileInterlacingConfigurationStringified(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration)
{
    const TransactionID transactionID = SetVolatileInterlacingConfiguration(commandedSensor, interlacing_configuration);
    return std::make_pair(transactionID, "CSC_SetVolatileInterlacingConfiguration");
}

TransactionID ModelHControllerCommands::SetVolatileLidarDataEndpoint(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileLidarDataEndpointCommand(
        endpoint,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileLidarDataEndpointStringified(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint)
{
    const TransactionID transactionID = SetVolatileLidarDataEndpoint(commandedSensor, endpoint);
    return std::make_pair(transactionID, "CSC_SetVolatileLidarDataEndpoint");
}

TransactionID ModelHControllerCommands::SetVolatileOptimizeSnapbacks(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileOptimizeSnapbacksCommand(
        optimize_snapbacks,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileOptimizeSnapbacksStringified(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks)
{
    const TransactionID transactionID = SetVolatileOptimizeSnapbacks(commandedSensor, optimize_snapbacks);
    return std::make_pair(transactionID, "CSC_SetVolatileOptimizeSnapbacks");
}

TransactionID ModelHControllerCommands::SetVolatilePTPDelayInterval(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatilePTPDelayIntervalCommand(
        ptpDelayInterval,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatilePTPDelayIntervalStringified(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval)
{
    const TransactionID transactionID = SetVolatilePTPDelayInterval(commandedSensor, ptpDelayInterval);
    return std::make_pair(transactionID, "CSC_SetVolatilePTPDelayInterval");
}

TransactionID ModelHControllerCommands::SetVolatilePTPMode(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatilePTPModeCommand(
        ptpMode,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatilePTPModeStringified(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode)
{
    const TransactionID transactionID = SetVolatilePTPMode(commandedSensor, ptpMode);
    return std::make_pair(transactionID, "CSC_SetVolatilePTPMode");
}

TransactionID ModelHControllerCommands::SetVolatileScanPlaylist(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileScanPlaylistCommand(
        scanPlaylist,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileScanPlaylistStringified(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist)
{
    const TransactionID transactionID = SetVolatileScanPlaylist(commandedSensor, scanPlaylist);
    return std::make_pair(transactionID, "CSC_SetVolatileScanPlaylist");
}

TransactionID ModelHControllerCommands::SetVolatileSensorLidarDataEnabled(DiscoveryResponse commandedSensor, LumNet_Bool enable)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileSensorLidarDataEnabledCommand(
        enable,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileSensorLidarDataEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool enable)
{
    const TransactionID transactionID = SetVolatileSensorLidarDataEnabled(commandedSensor, enable);
    return std::make_pair(transactionID, "CSC_SetVolatileSensorLidarDataEnabled");
}

TransactionID ModelHControllerCommands::SetVolatileSensorNickname(DiscoveryResponse commandedSensor, LumNet_Nickname nickname)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileSensorNicknameCommand(
        nickname,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileSensorNicknameStringified(DiscoveryResponse commandedSensor, LumNet_Nickname nickname)
{
    const TransactionID transactionID = SetVolatileSensorNickname(commandedSensor, nickname);
    return std::make_pair(transactionID, "CSC_SetVolatileSensorNickname");
}

TransactionID ModelHControllerCommands::SetVolatileSnapScanWholeNumberLinesEnabled(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileSnapScanWholeNumberLinesEnabledCommand(
        snap_scan_whole_number_lines_enabled,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileSnapScanWholeNumberLinesEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled)
{
    const TransactionID transactionID = SetVolatileSnapScanWholeNumberLinesEnabled(commandedSensor, snap_scan_whole_number_lines_enabled);
    return std::make_pair(transactionID, "CSC_SetVolatileSnapScanWholeNumberLinesEnabled");
}

TransactionID ModelHControllerCommands::SetVolatileUDPStatusEnable(DiscoveryResponse commandedSensor, LumNet_Bool status_enable)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileUDPStatusEnableCommand(
        status_enable,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileUDPStatusEnableStringified(DiscoveryResponse commandedSensor, LumNet_Bool status_enable)
{
    const TransactionID transactionID = SetVolatileUDPStatusEnable(commandedSensor, status_enable);
    return std::make_pair(transactionID, "CSC_SetVolatileUDPStatusEnable");
}

TransactionID ModelHControllerCommands::SetVolatileUDPStatusEndpoint(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileUDPStatusEndpointCommand(
        endpoint,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileUDPStatusEndpointStringified(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint)
{
    const TransactionID transactionID = SetVolatileUDPStatusEndpoint(commandedSensor, endpoint);
    return std::make_pair(transactionID, "CSC_SetVolatileUDPStatusEndpoint");
}

TransactionID ModelHControllerCommands::SetVolatileVerticalSyncMode(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileVerticalSyncModeCommand(
        vertical_sync_mode,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileVerticalSyncModeStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode)
{
    const TransactionID transactionID = SetVolatileVerticalSyncMode(commandedSensor, vertical_sync_mode);
    return std::make_pair(transactionID, "CSC_SetVolatileVerticalSyncMode");
}

TransactionID ModelHControllerCommands::SetVolatileVerticalSyncOffset(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms)
{
    uint8_t* buffer = nullptr;

    const TransactionID transactionID = NextTransactionId();

    Command_EncodeSetVolatileVerticalSyncOffsetCommand(
        vertical_sync_offset_ms,
        transactionID,
        &buffer,
        GetCurrentSemVer());

    const size_t buffer_size = Command_CalculateLumNet_CommandListEncodeSize((struct LumNet_CommandRequestListPayload*) buffer);

    SendCommand(commandedSensor, buffer, buffer_size);

    free(buffer);

    return transactionID;
}

std::pair<TransactionID, const char*> ModelHControllerCommands::SetVolatileVerticalSyncOffsetStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms)
{
    const TransactionID transactionID = SetVolatileVerticalSyncOffset(commandedSensor, vertical_sync_offset_ms);
    return std::make_pair(transactionID, "CSC_SetVolatileVerticalSyncOffset");
}

std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> ModelHControllerCommands::PrepareInterlaceRequests(struct LumNet_InterlacingConfiguration interlacing_configuration)
{
    std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> thunks;

    thunks.push_back([interlacing_configuration, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentInterlacingConfiguration(sensor, interlacing_configuration);
      return std::make_pair(transactionID, CSC_SetPersistentInterlacingConfiguration);
    });

    return thunks;
}

std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> ModelHControllerCommands::PrepareNetworkSettingsRequests(LumNet_Bool enable, LumNet_IpAddress ip_address, LumNet_NetworkType network_type, LumNet_Nickname nickname, struct LumNet_Endpoint endpoint)
{
    std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> thunks;

    thunks.push_back([enable, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentSensorLidarDataEnabled(sensor, enable);
      return std::make_pair(transactionID, CSC_SetPersistentSensorLidarDataEnabled);
    });
    thunks.push_back([endpoint, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentLidarDataEndpoint(sensor, endpoint);
      return std::make_pair(transactionID, CSC_SetPersistentLidarDataEndpoint);
    });
    thunks.push_back([ip_address, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentSensorIPAddress(sensor, ip_address);
      return std::make_pair(transactionID, CSC_SetPersistentSensorIPAddress);
    });
    thunks.push_back([network_type, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentSensorNetworkType(sensor, network_type);
      return std::make_pair(transactionID, CSC_SetPersistentSensorNetworkType);
    });
    thunks.push_back([nickname, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentSensorNickname(sensor, nickname);
      return std::make_pair(transactionID, CSC_SetPersistentSensorNickname);
    });

    return thunks;
}

std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> ModelHControllerCommands::PrepareScannerRequests(LumNet_Bool optimize_snapbacks, LumNet_Bool snap_scan_whole_number_lines_enabled, LumNet_VerticalSyncMode vertical_sync_mode, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms, struct LumNet_GenericScanProfile scan_profile, struct LumNet_ScanPlaylist scanPlaylist)
{
    std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> thunks;

    thunks.push_back([optimize_snapbacks, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentOptimizeSnapbacks(sensor, optimize_snapbacks);
      return std::make_pair(transactionID, CSC_SetPersistentOptimizeSnapbacks);
    });
    thunks.push_back([scanPlaylist, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentScanPlaylist(sensor, scanPlaylist);
      return std::make_pair(transactionID, CSC_SetPersistentScanPlaylist);
    });
    thunks.push_back([scan_profile, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentGenericScanProfile(sensor, scan_profile);
      return std::make_pair(transactionID, CSC_SetPersistentGenericScanProfile);
    });
    thunks.push_back([snap_scan_whole_number_lines_enabled, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentSnapScanWholeNumberLinesEnabled(sensor, snap_scan_whole_number_lines_enabled);
      return std::make_pair(transactionID, CSC_SetPersistentSnapScanWholeNumberLinesEnabled);
    });
    thunks.push_back([vertical_sync_mode, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentVerticalSyncMode(sensor, vertical_sync_mode);
      return std::make_pair(transactionID, CSC_SetPersistentVerticalSyncMode);
    });
    thunks.push_back([vertical_sync_offset_ms, this](lum::DiscoveryResponse sensor)
    {
      const auto transactionID = SetPersistentVerticalSyncOffset(sensor, vertical_sync_offset_ms);
      return std::make_pair(transactionID, CSC_SetPersistentVerticalSyncOffset);
    });

    return thunks;
}
