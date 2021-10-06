/**
 * @file   Discovery.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-level file to include all of LumNet's discovery functionality
 *         which provides networking and assignmment based payloads via UDP.
 */

#ifndef LUM_NET_DISCOVERY_H
#define LUM_NET_DISCOVERY_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <LumNet/Common.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief Semantic Major Version of the LumNet Discovery functionality.
 */
#define LUM_NET_DSC_PROTOCOL_SEM_VER_MAJOR (0)

/**
 * @brief Semantic Minor Version of the LumNet Discovery functionality.
 */
#define LUM_NET_DSC_PROTOCOL_SEM_VER_MINOR (5)

/**
 * @brief Semantic Patch Version of the LumNet Discovery functionality.
 */
#define LUM_NET_DSC_PROTOCOL_SEM_VER_PATCH (0)

/**
 * @brief Default port which the Sensor Head listens to UDP Discovery commands
 * on
 */
#define LUM_NET_DEFAULT_UDP_DISCOVERY_PORT (11000)
//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

/**
 * @brief Structure representing relevant networking fields that user may want
 * to know when a Sensor Head is discovered.
 */
LUM_NET_PACK(struct LumNet_NetworkConfig { // TODO: Reorder fields more sensibly
  LumNet_Bool lidarDataEnable; //!< Whether the lidar data is set to stream to
                               //!< the endpoint.
  struct LumNet_Endpoint
              lidarDataEndPoint; //!< The endpoint where the lidar data is streamed.
  LumNet_Port dscUdpPort; //!< Port where UDP discovery messages should be sent
                          //!< to and received from.
  LumNet_Port cmdTcpPort; //!< Port where TCP command messages should be sent to
                          //!< and recieved from.
  struct LumNet_NetworkConnection networkConnection; //!< The details regarding
                                                     //!< the Sensor Head
                                                     //!< network connection.
  struct LumNet_MacAddress macAddress; //!< The MAC Address of the Sensor Head
  LumNet_MTU mtu; //!< The Maximum Transmission Unit (MTU) of the network link.
  LumNet_NetworkType networkType; //!< The type of network connection that has
                                  //!< been set up for the Sensor Head.
});

/**
 * @brief Structure representing a request to the Sensor Head to identify itself
 * for discovery purposes. This message is meant to be broadcasted on a subnet.
 */
LUM_NET_PACK(struct LumNet_DiscoveryRequest {
  struct LumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_DSC_REQUEST.
});

/**
 * @brief Structure representing a Sensor Head's response to a discovery
 *        request. This structure represents information for unique
 * identification, networking information, and state machine information.
 */
LUM_NET_PACK(struct LumNet_DiscoveryResponse {
  struct LumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE.
  struct LumNet_SerialNumber
      serialNumber; //!< Sensor Head's unique serial number.
  struct LumNet_Description
      description; //!< An optional description about the Sensor Head.

  LumNet_Nickname nickName; //!< Sensor Head's nickname identifier.
  struct LumNet_NetworkConfig
                      networkConfig; //!< Sensor Head's current networking configuration.
  struct LumNet_State state; //!< Sensor Head's current state machine states.
  struct LumNet_Version
      buildVersion; //!< Sensor Head's currently running build version.
});

/**
 * @brief Structure representing the payload to send the Sensor Head to
 *        to stream Lidar Data to an endpoint.
 */
LUM_NET_PACK(struct LumNet_StartLidarDataStream {
  struct LumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_START_LIDAR_DATA_STREAM_CMD.
  struct LumNet_Endpoint
      destEndpoint; //!< Client that Sensor Head should stream Lidar Data to.
});

/**
 * @brief Structure representing the payload to send the Sensor Head to
 *        to stop sending Lidar Data.
 */
LUM_NET_PACK(struct LumNet_StopLidarDataStream {
  struct LumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_STOP_LIDAR_DATA_STREAM_CMD.
});

/**
 * @brief Structure representing the payload to send the Sensor Head to change
 * its IP Address, netmask, and gateway. If and only if the serial number
 * matches, will the network connection become updated.
 */
LUM_NET_PACK(struct LumNet_NetworkConnectionAssignment {
  struct LumNet_CommonHeader
      header; //!< Common Header that all payloads contain
              //!< where payloadTypeID should be set to
              //!< LUM_NET_PAYLOAD_TYPE_ID_DSC_NETWORK_CONNECTION_ASSIGNMENT.
  struct LumNet_NetworkConnection
      networkConnection; //!< New Network Connection to assign Sensor Head
  struct LumNet_SerialNumber
      serialNumber; //!< Serial Number of Sensor Head found
                    //!< via LumNet_DiscoveryRequest.
});

/**
 * @brief Structure representing the payload to send the Sensor Head to reset
 * it's network connection (including IP Address) to the default. Sensor Head
 * connections can be reset on a per head basis using the Serial Number
 * or unconditionally using an override.
 */
LUM_NET_PACK(struct LumNet_NetworkConnectionRecovery {
  struct LumNet_CommonHeader
      header; //!< Common Header that all payloads contain
              //!< where payloadTypeID should be set to
              //!< LUM_NET_PAYLOAD_TYPE_ID_DSC_NETWORK_CONNECTION_RECOVERY.
  LumNet_Bool overrideSerialNumber;
  //!< If True, any Sensor Head that recieves this will have their interface
  //!< connection reset. If False, only Sensor Heads whose Serial Number who
  //!< matches the second argument will be reset.
  struct LumNet_SerialNumber
      serialNumber; //!< Serial Number of Sensor Head found
                    //!< via LumNet_DiscoveryRequest.
});
LUM_NET_PACK(struct LumNet_SetPtpSensor {
  struct LumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_START_LIDAR_DATA_STREAM_CMD.
  struct LumNet_PtpTime clientTime;
});

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

/**
 * @brief Given a Common Header from a Discovery payload, check whether the
 * header is using the current expected version.
 */
static inline bool
LumNet_HasDiscoveryCommonHeader(struct LumNet_CommonHeader *pCommonHeader) {
  if (pCommonHeader->semVersionedHeader.semver.major ==
          LUM_NET_DSC_PROTOCOL_SEM_VER_MAJOR &&
      pCommonHeader->semVersionedHeader.semver.minor ==
          LUM_NET_DSC_PROTOCOL_SEM_VER_MINOR &&
      pCommonHeader->semVersionedHeader.semver.patch ==
          LUM_NET_DSC_PROTOCOL_SEM_VER_PATCH) {
    return true;
  } else {
    return false;
  }
}

static inline struct LumNet_Version LumNet_GetDiscoveryProtocolSemVer(void) {
  return LumNet_VersionBuilder(LUM_NET_DSC_PROTOCOL_SEM_VER_MAJOR,
                               LUM_NET_DSC_PROTOCOL_SEM_VER_MINOR,
                               LUM_NET_DSC_PROTOCOL_SEM_VER_PATCH);
}

  //@}

#endif // LUM_NET_DISCOVERY_H
