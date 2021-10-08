/*
* ModelHControllerCommands.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHControllerCommands
 *         `ModelHControllerCommands` wraps the commands in `CommandConstructors.h`.
 *
 *  This class creates command buffers and delegates sending them to "SendCommand(...)".
 *
 *  The intended use is to sublcass `ModelHControllerCommands` and implement the "SendCommand(...)" and
 *  "NextTransactionId()" methods as you please.
 *
 *  See `ModelHController` for an example.
 *
 */

#pragma once

// clang-format off

#include "CommandConstructors.h"
#include "CommandLumNetDefinitions.h"

#include <stddef.h>
#include <stdint.h>
#include <vector>
#include <functional>

namespace lum
{
    using TransactionID = uint16_t;
    using FingerprintID = uint16_t;

    class DiscoveryResponse;
    class ModelHCommandClientSubscriber;

    class ModelHControllerCommands
    {
    public:
        /*
         * `NextTransactionID` should be implemented by client classes to correctly generate strictly increasing
         * transaction IDs.
         *
        */
        virtual TransactionID NextTransactionId();

        /*
         * `SendCommand` should be implemented by client classes.
         *
        */
        virtual bool SendCommand( lum::DiscoveryResponse commandedSensor, uint8_t* command, size_t command_size );

        /*
         * `GetCurrentSemVer` should be implemented by client classes to provide the current command semver
         *
        */
        virtual LegacyCommandProtocolSemVer GetCurrentSemVer();

        /*
         * A reasonable default implementation for `HandleCommandResponse` exists but is marked virtual
         * so clients can implement their own behavior if desired.
         *
        */
        virtual void HandleCommandResponse( ModelHCommandClientSubscriber* subscriber, FingerprintID fingerprint, uint8_t* response );

        /*
        * Issue a ReadPersistentGenericScanProfileCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentGenericScanProfile(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentGenericScanProfileCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentGenericScanProfileStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentInterlacingConfigurationCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentInterlacingConfiguration(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentInterlacingConfigurationCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentInterlacingConfigurationStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentLidarDataEndpointCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentLidarDataEndpoint(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentLidarDataEndpointCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentLidarDataEndpointStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentOptimizeSnapbacksCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentOptimizeSnapbacks(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentOptimizeSnapbacksCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentOptimizeSnapbacksStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentPTPDelayIntervalCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentPTPDelayInterval(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentPTPDelayIntervalCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentPTPDelayIntervalStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentPTPModeCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentPTPMode(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentPTPModeCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentPTPModeStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentScanPlaylistCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentScanPlaylist(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentScanPlaylistCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentScanPlaylistStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorIPAddressCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentSensorIPAddress(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorIPAddressCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentSensorIPAddressStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorLidarDataEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentSensorLidarDataEnabled(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorLidarDataEnabledCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentSensorLidarDataEnabledStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorNetworkTypeCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentSensorNetworkType(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorNetworkTypeCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentSensorNetworkTypeStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorNicknameCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentSensorNickname(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSensorNicknameCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentSensorNicknameStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSnapScanWholeNumberLinesEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentSnapScanWholeNumberLinesEnabled(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentSnapScanWholeNumberLinesEnabledCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentSnapScanWholeNumberLinesEnabledStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentVerticalSyncModeCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentVerticalSyncMode(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentVerticalSyncModeCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentVerticalSyncModeStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentVerticalSyncOffsetCommand request and return the corresponding TransactionID
        */
        TransactionID ReadPersistentVerticalSyncOffset(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadPersistentVerticalSyncOffsetCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadPersistentVerticalSyncOffsetStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileGenericScanProfileCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileGenericScanProfile(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileGenericScanProfileCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileGenericScanProfileStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileInterlacingConfigurationCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileInterlacingConfiguration(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileInterlacingConfigurationCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileInterlacingConfigurationStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileInterlockStatesCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileInterlockStates(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileInterlockStatesCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileInterlockStatesStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileLidarDataEndpointCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileLidarDataEndpoint(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileLidarDataEndpointCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileLidarDataEndpointStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileOptimizeSnapbacksCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileOptimizeSnapbacks(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileOptimizeSnapbacksCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileOptimizeSnapbacksStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatilePTPDelayIntervalCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatilePTPDelayInterval(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatilePTPDelayIntervalCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatilePTPDelayIntervalStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatilePTPModeCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatilePTPMode(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatilePTPModeCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatilePTPModeStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatilePTPStatusCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatilePTPStatus(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatilePTPStatusCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatilePTPStatusStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileScanPlaylistCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileScanPlaylist(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileScanPlaylistCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileScanPlaylistStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileScanningSynchronizedStateCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileScanningSynchronizedState(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileScanningSynchronizedStateCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileScanningSynchronizedStateStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorIPAddressCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileSensorIPAddress(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorIPAddressCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileSensorIPAddressStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorLidarDataEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileSensorLidarDataEnabled(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorLidarDataEnabledCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileSensorLidarDataEnabledStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorMacAddressCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileSensorMacAddress(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorMacAddressCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileSensorMacAddressStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorNicknameCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileSensorNickname(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSensorNicknameCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileSensorNicknameStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSnapScanWholeNumberLinesEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileSnapScanWholeNumberLinesEnabled(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSnapScanWholeNumberLinesEnabledCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileSnapScanWholeNumberLinesEnabledStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSystemTemperatureCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileSystemTemperature(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileSystemTemperatureCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileSystemTemperatureStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileUDPStatusEnableCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileUDPStatusEnable(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileUDPStatusEnableCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileUDPStatusEnableStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileUDPStatusEndpointCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileUDPStatusEndpoint(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileUDPStatusEndpointCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileUDPStatusEndpointStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileVerticalSyncModeCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileVerticalSyncMode(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileVerticalSyncModeCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileVerticalSyncModeStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileVerticalSyncOffsetCommand request and return the corresponding TransactionID
        */
        TransactionID ReadVolatileVerticalSyncOffset(lum::DiscoveryResponse commandedSensor);

        /*
        * Issue a ReadVolatileVerticalSyncOffsetCommand request and return the corresponding TransactionID paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> ReadVolatileVerticalSyncOffsetStringified(DiscoveryResponse commandedSensor);

        /*
        * Issue a SetPersistentGenericScanProfileCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentGenericScanProfile(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile);

        /*
        * Issue a SetPersistentGenericScanProfileCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentGenericScanProfileStringified(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile);

        /*
        * Issue a SetPersistentInterlacingConfigurationCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentInterlacingConfiguration(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration);

        /*
        * Issue a SetPersistentInterlacingConfigurationCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentInterlacingConfigurationStringified(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration);

        /*
        * Issue a SetPersistentLidarDataEndpointCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentLidarDataEndpoint(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint);

        /*
        * Issue a SetPersistentLidarDataEndpointCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentLidarDataEndpointStringified(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint);

        /*
        * Issue a SetPersistentOptimizeSnapbacksCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentOptimizeSnapbacks(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks);

        /*
        * Issue a SetPersistentOptimizeSnapbacksCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentOptimizeSnapbacksStringified(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks);

        /*
        * Issue a SetPersistentPTPDelayIntervalCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentPTPDelayInterval(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval);

        /*
        * Issue a SetPersistentPTPDelayIntervalCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentPTPDelayIntervalStringified(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval);

        /*
        * Issue a SetPersistentPTPModeCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentPTPMode(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode);

        /*
        * Issue a SetPersistentPTPModeCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentPTPModeStringified(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode);

        /*
        * Issue a SetPersistentScanPlaylistCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentScanPlaylist(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist);

        /*
        * Issue a SetPersistentScanPlaylistCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentScanPlaylistStringified(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist);

        /*
        * Issue a SetPersistentSensorIPAddressCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentSensorIPAddress(DiscoveryResponse commandedSensor, LumNet_IpAddress ip_address);

        /*
        * Issue a SetPersistentSensorIPAddressCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentSensorIPAddressStringified(DiscoveryResponse commandedSensor, LumNet_IpAddress ip_address);

        /*
        * Issue a SetPersistentSensorLidarDataEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentSensorLidarDataEnabled(DiscoveryResponse commandedSensor, LumNet_Bool enable);

        /*
        * Issue a SetPersistentSensorLidarDataEnabledCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentSensorLidarDataEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool enable);

        /*
        * Issue a SetPersistentSensorNetworkTypeCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentSensorNetworkType(DiscoveryResponse commandedSensor, LumNet_NetworkType network_type);

        /*
        * Issue a SetPersistentSensorNetworkTypeCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentSensorNetworkTypeStringified(DiscoveryResponse commandedSensor, LumNet_NetworkType network_type);

        /*
        * Issue a SetPersistentSensorNicknameCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentSensorNickname(DiscoveryResponse commandedSensor, LumNet_Nickname nickname);

        /*
        * Issue a SetPersistentSensorNicknameCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentSensorNicknameStringified(DiscoveryResponse commandedSensor, LumNet_Nickname nickname);

        /*
        * Issue a SetPersistentSnapScanWholeNumberLinesEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentSnapScanWholeNumberLinesEnabled(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled);

        /*
        * Issue a SetPersistentSnapScanWholeNumberLinesEnabledCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentSnapScanWholeNumberLinesEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled);

        /*
        * Issue a SetPersistentVerticalSyncModeCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentVerticalSyncMode(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode);

        /*
        * Issue a SetPersistentVerticalSyncModeCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentVerticalSyncModeStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode);

        /*
        * Issue a SetPersistentVerticalSyncOffsetCommand request and return the corresponding TransactionID
        */
        TransactionID SetPersistentVerticalSyncOffset(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms);

        /*
        * Issue a SetPersistentVerticalSyncOffsetCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetPersistentVerticalSyncOffsetStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms);

        /*
        * Issue a SetVolatileGenericScanProfileCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileGenericScanProfile(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile);

        /*
        * Issue a SetVolatileGenericScanProfileCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileGenericScanProfileStringified(DiscoveryResponse commandedSensor, struct LumNet_GenericScanProfile scan_profile);

        /*
        * Issue a SetVolatileInterlacingConfigurationCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileInterlacingConfiguration(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration);

        /*
        * Issue a SetVolatileInterlacingConfigurationCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileInterlacingConfigurationStringified(DiscoveryResponse commandedSensor, struct LumNet_InterlacingConfiguration interlacing_configuration);

        /*
        * Issue a SetVolatileLidarDataEndpointCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileLidarDataEndpoint(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint);

        /*
        * Issue a SetVolatileLidarDataEndpointCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileLidarDataEndpointStringified(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint);

        /*
        * Issue a SetVolatileOptimizeSnapbacksCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileOptimizeSnapbacks(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks);

        /*
        * Issue a SetVolatileOptimizeSnapbacksCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileOptimizeSnapbacksStringified(DiscoveryResponse commandedSensor, LumNet_Bool optimize_snapbacks);

        /*
        * Issue a SetVolatilePTPDelayIntervalCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatilePTPDelayInterval(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval);

        /*
        * Issue a SetVolatilePTPDelayIntervalCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatilePTPDelayIntervalStringified(DiscoveryResponse commandedSensor, LumNet_PtpTypeSigned ptpDelayInterval);

        /*
        * Issue a SetVolatilePTPModeCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatilePTPMode(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode);

        /*
        * Issue a SetVolatilePTPModeCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatilePTPModeStringified(DiscoveryResponse commandedSensor, LumNet_PtpMode ptpMode);

        /*
        * Issue a SetVolatileScanPlaylistCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileScanPlaylist(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist);

        /*
        * Issue a SetVolatileScanPlaylistCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileScanPlaylistStringified(DiscoveryResponse commandedSensor, struct LumNet_ScanPlaylist scanPlaylist);

        /*
        * Issue a SetVolatileSensorLidarDataEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileSensorLidarDataEnabled(DiscoveryResponse commandedSensor, LumNet_Bool enable);

        /*
        * Issue a SetVolatileSensorLidarDataEnabledCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileSensorLidarDataEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool enable);

        /*
        * Issue a SetVolatileSensorNicknameCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileSensorNickname(DiscoveryResponse commandedSensor, LumNet_Nickname nickname);

        /*
        * Issue a SetVolatileSensorNicknameCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileSensorNicknameStringified(DiscoveryResponse commandedSensor, LumNet_Nickname nickname);

        /*
        * Issue a SetVolatileSnapScanWholeNumberLinesEnabledCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileSnapScanWholeNumberLinesEnabled(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled);

        /*
        * Issue a SetVolatileSnapScanWholeNumberLinesEnabledCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileSnapScanWholeNumberLinesEnabledStringified(DiscoveryResponse commandedSensor, LumNet_Bool snap_scan_whole_number_lines_enabled);

        /*
        * Issue a SetVolatileUDPStatusEnableCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileUDPStatusEnable(DiscoveryResponse commandedSensor, LumNet_Bool status_enable);

        /*
        * Issue a SetVolatileUDPStatusEnableCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileUDPStatusEnableStringified(DiscoveryResponse commandedSensor, LumNet_Bool status_enable);

        /*
        * Issue a SetVolatileUDPStatusEndpointCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileUDPStatusEndpoint(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint);

        /*
        * Issue a SetVolatileUDPStatusEndpointCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileUDPStatusEndpointStringified(DiscoveryResponse commandedSensor, struct LumNet_Endpoint endpoint);

        /*
        * Issue a SetVolatileVerticalSyncModeCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileVerticalSyncMode(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode);

        /*
        * Issue a SetVolatileVerticalSyncModeCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileVerticalSyncModeStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncMode vertical_sync_mode);

        /*
        * Issue a SetVolatileVerticalSyncOffsetCommand request and return the corresponding TransactionID
        */
        TransactionID SetVolatileVerticalSyncOffset(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms);

        /*
        * Issue a SetVolatileVerticalSyncOffsetCommand request paired with a string describing the command.
        */
        std::pair<TransactionID, const char*> SetVolatileVerticalSyncOffsetStringified(DiscoveryResponse commandedSensor, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms);

        std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> PrepareInterlaceRequests(struct LumNet_InterlacingConfiguration interlacing_configuration);

        std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> PrepareNetworkSettingsRequests(LumNet_Bool enable, LumNet_IpAddress ip_address, LumNet_NetworkType network_type, LumNet_Nickname nickname, struct LumNet_Endpoint endpoint);

        std::vector<std::function<std::pair<lum::TransactionID, CSC_CommandTag>(lum::DiscoveryResponse)>> PrepareScannerRequests(LumNet_Bool optimize_snapbacks, LumNet_Bool snap_scan_whole_number_lines_enabled, LumNet_VerticalSyncMode vertical_sync_mode, LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms, struct LumNet_GenericScanProfile scan_profile, struct LumNet_ScanPlaylist scanPlaylist);
    };
}  // namespace lum
