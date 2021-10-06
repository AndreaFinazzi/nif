/*
* ModelHCommandClientSubscriber.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelhCommandClientSubscriber
 *         The `ModelHCommandClientSubscriber` describes the interface for receiving command request status and replies.
 *
 */

#pragma once

// clang-format off

#include "luminlib_export.h"

#include "CommandLumNetDefinitions.h"

#include "LumNet/Command.h"

#include <stdint.h>

namespace lum
{
    enum CSCTransactionStatus {
        VALIDATED,
        FAILED_TO_VALIDATE,
        COMPLETED,
        FAILED_TO_COMPLETE
    };

    class LUMINLIB_EXPORT ModelHCommandClientSubscriber
    {
    public:
        /*
        * Notification handler for transaction status: accepted / rejected for processing, and process complete
        */
        virtual void TransactionStatus( uint16_t fingerprint,
                                        uint16_t transactionID,
                                        CSCTransactionStatus transactionStatus,
                                        const uint8_t* response ){}

        /*
        * Notification handler for responses from a(n) GenericScanProfile request
        */
        virtual void GenericScanProfileUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_GenericScanProfile scan_profile) {}

        /*
        * Notification handler for responses from a(n) InterlacingConfiguration request
        */
        virtual void InterlacingConfigurationUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_InterlacingConfiguration interlacing_configuration) {}

        /*
        * Notification handler for responses from a(n) InterlockStates request
        */
        virtual void InterlockStatesUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_Interlocks interlock_states) {}

        /*
        * Notification handler for responses from a(n) LidarDataEndpoint request
        */
        virtual void LidarDataEndpointUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_Endpoint endpoint) {}

        /*
        * Notification handler for responses from a(n) OptimizeSnapbacks request
        */
        virtual void OptimizeSnapbacksUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_Bool optimize_snapbacks) {}

        /*
        * Notification handler for responses from a(n) PTPDelayInterval request
        */
        virtual void PTPDelayIntervalUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_PtpTypeSigned ptpDelayInterval) {}

        /*
        * Notification handler for responses from a(n) PTPMode request
        */
        virtual void PTPModeUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_PtpMode ptpMode) {}

        /*
        * Notification handler for responses from a(n) PTPStatus request
        */
        virtual void PTPStatusUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_PtpStatus ptpStatus) {}

        /*
        * Notification handler for responses from a(n) ScanPlaylist request
        */
        virtual void ScanPlaylistUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_ScanPlaylist scanPlaylist) {}

        /*
        * Notification handler for responses from a(n) ScanningSynchronizedState request
        */
        virtual void ScanningSynchronizedStateUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_Bool scanning_synchronized_state) {}

        /*
        * Notification handler for responses from a(n) SensorIPAddress request
        */
        virtual void SensorIPAddressUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_IpAddress ip_address) {}

        /*
        * Notification handler for responses from a(n) SensorLidarDataEnabled request
        */
        virtual void SensorLidarDataEnabledUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_Bool enable) {}

        /*
        * Notification handler for responses from a(n) SensorMacAddress request
        */
        virtual void SensorMacAddressUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_MacAddress mac_address) {}

        /*
        * Notification handler for responses from a(n) SensorNetworkType request
        */
        virtual void SensorNetworkTypeUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_NetworkType network_type) {}

        /*
        * Notification handler for responses from a(n) SensorNickname request
        */
        virtual void SensorNicknameUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_Nickname nickname) {}

        /*
        * Notification handler for responses from a(n) SnapScanWholeNumberLinesEnabled request
        */
        virtual void SnapScanWholeNumberLinesEnabledUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_Bool snap_scan_whole_number_lines_enabled) {}

        /*
        * Notification handler for responses from a(n) SystemTemperature request
        */
        virtual void SystemTemperatureUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_SystemTemperature systemTemperature) {}

        /*
        * Notification handler for responses from a(n) UDPStatusEnable request
        */
        virtual void UDPStatusEnableUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_Bool status_enable) {}

        /*
        * Notification handler for responses from a(n) UDPStatusEndpoint request
        */
        virtual void UDPStatusEndpointUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          struct LumNet_Endpoint endpoint) {}

        /*
        * Notification handler for responses from a(n) VerticalSyncMode request
        */
        virtual void VerticalSyncModeUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_VerticalSyncMode vertical_sync_mode) {}

        /*
        * Notification handler for responses from a(n) VerticalSyncOffset request
        */
        virtual void VerticalSyncOffsetUpdated(uint16_t fingerprint,
                                          uint16_t transactionID,
                                          LumNet_VerticalSyncOffsetMs vertical_sync_offset_ms) {}
    };
}  // namespace lum
