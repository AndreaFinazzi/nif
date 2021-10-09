/**
 * @file   PayloadTypeID.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Enumeration of different types of payloads that LumNet supports.
 */

#ifndef LUM_NET_COMMON_PAYLOAD_TYPE_ID_H
#define LUM_NET_COMMON_PAYLOAD_TYPE_ID_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <stdint.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

typedef uint8_t LumNet_PayloadTypeID;

enum LumNet_PayloadTypeIDs {
  LUM_NET_PAYLOAD_TYPE_ID_LIDAR_DATA  = 0x01,
  LUM_NET_PAYLOAD_TYPE_ID_DSC_REQUEST = 0x02, //!< LumNet_DiscoveryRequest
  LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE_DEPRECATED =
      0x03, //!< LegacyLumNet_V2_DiscoveryResponse or
            //!< LegacyLumNet_DiscoveryResponse
  LUM_NET_PAYLOAD_TYPE_ID_DSC_NETWORK_CONNECTION_ASSIGNMENT =
      0x04, //!< LumNet_NetworkConnectionAssignment
  LUM_NET_PAYLOAD_TYPE_ID_DSC_NETWORK_SN_RESPONSE = 0x05, //!< LumNet_SnResponse
  LUM_NET_PAYLOAD_TYPE_ID_CMD_REQUEST_LIST =
      0x06, //!< LumNet_CommandRequestListPayload
  LUM_NET_PAYLOAD_TYPE_ID_CMD_REQUEST_LIST_RETURN_RESULT =
      0x07, //!< LumNet_CommandRequestListValidationPayload
  LUM_NET_PAYLOAD_TYPE_ID_CMD_RESPONSE_LIST =
      0x08, //!< LumNet_CommandResponseListPayload
  LUM_NET_PAYLOAD_TYPE_ID_CMD_URGENT_REQUEST_LIST =
      0x09, //!< LumNet_CommandUrgentRequestListPayload
  LUM_NET_PAYLOAD_TYPE_ID_STATUS_RESPONSE_LIST = 0x0A,
  LUM_NET_PAYLOAD_TYPE_ID_LIDAR_DATA_START_STREAM =
      0x0B, // LumNet_LidarDataStartStream
  LUM_NET_PAYLOAD_TYPE_ID_LIDAR_DATA_STOP_STREAM =
      0x0C, // LumNet_LidarDataStopStream
  LUM_NET_PAYLOAD_TYPE_ID_DSC_NETWORK_CONNECTION_RECOVERY =
      0x0D, // LumNet_NetworkConnectionRecover
  LUM_NET_PAYLOAD_TYPE_SET_UNIX_TIME =
      0x10, // LumNet set PTP seconds with Unix seconds
  LUM_NET_PAYLOAD_TYPE_REQUEST_PTP_SECONDS = 0x11, // LumNet request PTP time
  LUM_NET_PAYLOAD_TYPE_RESPONSE_PTP_SECONDS =
      0x12, // LumNet responds with PTP seconds and nanosecond counts
  LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE_DEPRECATED_V4 =
      0x13, //!< LegacyLumNet_V4_DiscoveryResponse
  LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE =
      0x14, //!< LegacyLumNet_V4_DiscoveryResponse
  LUM_NET_PAYLOAD_TYPE_ID_STATUS_PACKET = 0x15, //!<
};

//@}

/*********************************/
/* GLOBAL Variable Declarations  */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Function Declarations  */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Function Definitions   */
/*********************************/
//@{

//@}

#endif // LUM_NET_COMMON_PAYLOAD_TYPE_ID_H
