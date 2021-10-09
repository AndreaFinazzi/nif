/**
 * @file   Discovery.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-level file to include the subset of LumNet's discovery
 * functionality in order to discover legacy sensors.
 */

#ifndef LEGACY_LUM_NET_DISCOVERY_H
#define LEGACY_LUM_NET_DISCOVERY_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <LegacyLumNet/Common.h>
#include <LumNet/Discovery.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief Semantic Major Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_MAJOR (0)

/**
 * @brief Semantic Minor Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_MINOR (1)

/**
 * @brief Semantic Patch Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_PATCH (0)

/**
 * @brief Semantic Major Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_MAJOR (0)

/**
 * @brief Semantic Minor Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_MINOR (2)

/**
 * @brief Semantic Patch Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_PATCH (0)
//@}

/**
 * @brief Semantic Major Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_MAJOR (0)

/**
 * @brief Semantic Minor Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_MINOR (4)

/**
 * @brief Semantic Patch Version of the LumNet Discovery functionality
 * when legacy headers were used.
 */
#define LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_PATCH (0)
//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

/**
 * @brief Legacy structure representing a request to the Sensor Head to
 * identify itself for discovery purposes. This message is meant to be
 * broadcasted on a subnet.
 */
LUM_NET_PACK(struct LegacyLumNet_DiscoveryRequest {
  struct LegacyLumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_DSC_REQUEST.
});

/**
 * @brief Legacy structure representing a Sensor Head's response to a discovery
 *        request. This structure represents information for unique
 *        identification, networking information, and state machine
 *        information.
 */
LUM_NET_PACK(struct LegacyLumNet_DiscoveryResponse {
  struct LegacyLumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE.
  struct LegacyLumNet_SerialNumber
                               serialNumber; //!< Sensor Head's unique serial number.
  struct LegacyLumNet_Nickname nickName; //!< Sensor Head's nickname identifier.
  struct LumNet_NetworkConfig
      networkConfig; //!< Sensor Head's current networking configuration.
  struct LegacyLumNet_State
      state; //!< Sensor Head's current state machine states.
  struct LegacyLumNet_Version
      buildVersion; //!< Sensor Head's currently running build version.
});

/**
 * @brief Structure representing a Sensor Head's response to a discovery
 *        request. This structure represents information for unique
 * identification, networking information, and state machine information.
 */
LUM_NET_PACK(struct LegacyLumNet_V2_DiscoveryResponse {
  struct LumNet_CommonHeader
      commonHeader; //!< Common Header that all payloads contain
                    //!< where payloadTypeID should be set to
                    //!< LUM_NET_PAYLOAD_TYPE_ID_DSC_RESPONSE.
  struct LegacyLumNet_SerialNumber
                               serialNumber; //!< Sensor Head's unique serial number.
  struct LegacyLumNet_Nickname nickName; //!< Sensor Head's nickname identifier.
  struct LumNet_NetworkConfig
      networkConfig; //!< Sensor Head's current networking configuration.
  struct LegacyLumNet_State
      state; //!< Sensor Head's current state machine states.
  struct LumNet_Version
      buildVersion; //!< Sensor Head's currently running build version.
});

/**
 * @brief Structure representing a Sensor Head's response to a discovery
 *        request. This structure represents information for unique
 * identification, networking information, and state machine information.
 */
LUM_NET_PACK(struct LegacyLumNet_V4_DiscoveryResponse {
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
  struct LegacyLumNet_State
      state; //!< Sensor Head's current state machine states.
  struct LumNet_Version
      buildVersion; //!< Sensor Head's currently running build version.
});

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
 * header is using the legacy protocol where payload and versioning fields were
 * ordered in a different way.
 */
static inline bool LegacyLumNet_HasLegacyDiscoveryCommonHeader(
    struct LumNet_CommonHeader *pCommonHeader) {

  // LegacyLumNet_CommonHeader and LumNet_CommonHeader are of the same size
  struct LegacyLumNet_CommonHeader *pLegacyCommonHeader =
      ((struct LegacyLumNet_CommonHeader *) pCommonHeader);

  if (pLegacyCommonHeader != NULL &&
      pLegacyCommonHeader->semVersionedHeader.semver.major ==
          LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_MAJOR &&
      pLegacyCommonHeader->semVersionedHeader.semver.minor ==
          LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_MINOR &&
      pLegacyCommonHeader->semVersionedHeader.semver.patch ==
          LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_PATCH) {
    return true;
  } else {
    return false;
  }
}
/**
 * @brief Given a Common Header from a Discovery payload, check whether the
 * header is using the legacy V2 protocol where payload and versioning fields
 * were ordered in a different way.
 */
static inline bool LegacyLumNet_HasLegacyV2DiscoveryCommonHeader(
    struct LumNet_CommonHeader *pCommonHeader) {

  if (pCommonHeader != NULL &&
      pCommonHeader->semVersionedHeader.semver.major ==
          LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_MAJOR &&
      pCommonHeader->semVersionedHeader.semver.minor ==
          LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_MINOR &&
      pCommonHeader->semVersionedHeader.semver.patch ==
          LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_PATCH) {
    return true;
  } else {
    return false;
  }
}

/**
 * @brief Given a Common Header from a Discovery payload, check whether the
 * header is using the legacy V4 protocol where payload and versioning fields
 * were ordered in a different way.
 */
static inline bool LegacyLumNet_HasLegacyV4DiscoveryCommonHeader(
    struct LumNet_CommonHeader *pCommonHeader) {

  if (pCommonHeader != NULL &&
      pCommonHeader->semVersionedHeader.semver.major ==
          LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_MAJOR &&
      pCommonHeader->semVersionedHeader.semver.minor ==
          LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_MINOR &&
      pCommonHeader->semVersionedHeader.semver.patch ==
          LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_PATCH) {
    return true;
  } else {
    return false;
  }
}

static inline struct LumNet_Version
LegacyLumNet_GetDiscoveryProtocolSemVer(void) {
  return LumNet_VersionBuilder(LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_MAJOR,
                               LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_MINOR,
                               LUM_NET_DSC_LEGACY_COMMON_HEADER_SEM_VER_PATCH);
}

static inline struct LumNet_Version
LegacyLumNet_V2_GetDiscoveryProtocolSemVer(void) {
  return LumNet_VersionBuilder(
      LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_MAJOR,
      LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_MINOR,
      LUM_NET_DSC_LEGACY_V2_COMMON_HEADER_SEM_VER_PATCH);
}

static inline struct LumNet_Version
LegacyLumNet_V4_GetDiscoveryProtocolSemVer(void) {
  return LumNet_VersionBuilder(
      LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_MAJOR,
      LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_MINOR,
      LUM_NET_DSC_LEGACY_V4_COMMON_HEADER_SEM_VER_PATCH);
}
  //@}

#endif // LEGACY_LUM_NET_DISCOVERY_H
