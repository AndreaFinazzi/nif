/*
 * @file   Command.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-level file to include the subset of LumNet's command
 * functionality in order to reject incoming commands via TCP.
 */

#ifndef LEGACY_LUM_NET_COMMAND_H
#define LEGACY_LUM_NET_COMMAND_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <LumNet/Command.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief Semantic Major Version of the LumNet Command functionality
 * when legacy headers were used.
 */
#define LUM_NET_CMD_LEGACY_COMMON_HEADER_SEM_VER_MAJOR (0)

/**
 * @brief Semantic Minor Version of the LumNet Command functionality
 * when legacy headers were used.
 */
#define LUM_NET_CMD_LEGACY_COMMON_HEADER_SEM_VER_MINOR (2)

/**
 * @brief Semantic Patch Version of the LumNet Command functionality
 * when legacy headers were used.
 */
#define LUM_NET_CMD_LEGACY_COMMON_HEADER_SEM_VER_PATCH (0)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

LUM_NET_PACK(struct LegacyLumNet_CommandListPreamble {
  struct LegacyLumNet_CommonHeader versionPayloadHeader;
});

LUM_NET_PACK(struct LegacyLumNet_CommandRequestListValidationPayload {
  struct LegacyLumNet_CommandListPreamble preamble;
  struct LumNet_CommandRequestListReturnResultHeader
      commandRequestListReturnResultHeader;
});

LUM_NET_PACK(struct LegacyLumNet_CommandRequestListPayload {
  struct LegacyLumNet_CommandListPreamble preamble;
  struct LumNet_CommandRequestListHeader  commandRequestListHeader;
  struct LumNet_CommandRequest            commands[1];
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
 * @brief Given a Common Header from a Command payload, check whether the header
 * is using the legacy protocol where payload and versioning fields were ordered
 * in a different way.
 */
static inline bool LegacyLumNet_HasLegacyCommandCommonHeader(
    struct LumNet_CommonHeader *pCommonHeader) {

  // LegacyLumNet_CommonHeader and LumNet_CommonHeader are of the same size
  struct LegacyLumNet_CommonHeader *pLegacyCommonHeader =
      ((struct LegacyLumNet_CommonHeader *) pCommonHeader);

  if (pLegacyCommonHeader != NULL &&
      pLegacyCommonHeader->semVersionedHeader.semver.major ==
          LUM_NET_CMD_LEGACY_COMMON_HEADER_SEM_VER_MAJOR &&
      pLegacyCommonHeader->semVersionedHeader.semver.minor ==
          LUM_NET_CMD_LEGACY_COMMON_HEADER_SEM_VER_MINOR &&
      pLegacyCommonHeader->semVersionedHeader.semver.patch ==
          LUM_NET_CMD_LEGACY_COMMON_HEADER_SEM_VER_PATCH) {
    return true;
  } else {
    return false;
  }
}

  //@}

#endif /* LEGACY_LUM_NET_COMMAND_H */
