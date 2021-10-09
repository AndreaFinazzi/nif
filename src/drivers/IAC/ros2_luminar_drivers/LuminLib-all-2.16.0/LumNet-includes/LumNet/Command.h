/*
 * @file   Command.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-level file to include all of LumNet's command functionality
 *         which provides network, lidar, and system payloads via TCP.
 */

#ifndef LUM_NET_COMMAND_H
#define LUM_NET_COMMAND_H

/*********************************/
/* Includes                      */
/*********************************/
//@{
#include <stdint.h>
#include <LumNet/Command/AddrOps.h>
#include <LumNet/Command/Payloads.h>
#include <LumNet/Common.h>
//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief Semantic Major Version of the LumNet Command functionality.
 */
#define LUM_NET_CMD_PROTOCOL_SEM_VER_MAJOR (0)

/**
 * @brief Semantic Minor Version of the LumNet Command functionality.
 */
#define LUM_NET_CMD_PROTOCOL_SEM_VER_MINOR (6)

/**
 * @brief Semantic Patch Version of the LumNet Command functionality.
 */
#define LUM_NET_CMD_PROTOCOL_SEM_VER_PATCH (0)

#define LUM_NET_CMD_MAGIC_NUMBER (0xAA)

/**
 * @brief Default port which the Sensor Head listens for TCP command connections
 */
#define LUM_NET_DEFAULT_TCP_COMMAND_PORT (50000)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

typedef uint16_t LumNet_CmdRespListReturnResult;

enum LumNet_CmdRespListReturnResults {
  LUM_NET_CMD_RESPONSE_LIST_RETURN_RESULT_SUCCESS = 0,
  LUM_NET_CMD_RESPONSE_LIST_RETURN_RESULT_FAILURE = 1,
};

enum LUM_NET_CMD_RETURN_RESULT {
  LUM_NET_CMD_RETURN_RESULT_SUCCESS             = 0x00,
  LUM_NET_CMD_RETURN_RESULT_FAILURE             = 0x01,
  LUM_NET_CMD_RETURN_RESULT_INDEX_OUT_RANGE     = 0x02,
  LUM_NET_CMD_RETURN_RESULT_PROTECTION_FAILURE  = 0x03,
  LUM_NET_CMD_RETURN_RESULT_IMPROPER_PROTECTION = 0x04,
  // Error codes specific to scan pattern generation
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_NEGATIVE_FOV        = 0x05,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_INVALID_FREQUENCY   = 0x06,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXCEEDS_MAX_EXTENTS = 0x07,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXCEEDS_MIN_EXTENTS = 0x08,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_TOO_SLOW_TO_ACHIEVE_SCAN =
      0x09,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_SIGMA_TOO_SMALL        = 0x0A,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MEAN_OUTSIDE_FOV       = 0x0B,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_BELOW_MIN = 0x0C,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_ABOVE_MAX = 0x0D,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_DENSITY_MULTIPLIER_OUT_OF_BOUNDS =
      0x0E,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_WIDTH_NOT_GREATER_THAN_ZERO =
      0x0F,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_WIDTH_LARGER_THAN_FOV =
      0x10,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_MIN_EXCEEDS_MIN_FOV_EXTENTS =
      0x11,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_MAX_EXCEEDS_MAX_FOV_EXTENTS =
      0x12,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_SHAPE_PARAMETERS_OUT_OF_ORDER =
      0x13,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_PARAMETERS_OUTSIDE_FOV =
      0x14,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXPONENT_OUT_OF_BOUNDS = 0x15,
  LUM_NET_CMD_RETURN_RESULT_SCAN_PATTERN_TRANSITION_FAILED              = 0x16,
  LUM_NET_CMD_RETURN_RESULT_SCAN_PATTERN_TIMEOUT_WAITING_FOR_PREVIOUS_REQUEST_TO_TAKE_EFFECT =
      0x17,
  LUM_NET_CMD_RETURN_RESULT_SCAN_PATTERN_EXCEEDS_MAXIMUM_PERMISSIBLE_EXPOSURE =
      0x18, // Based on minimum velocity
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_INVALID_PROFILE_TYPE = 0x19,
  LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PLAYLIST_INVALID_LENGTH      = 0x1A,
};

typedef uint8_t LumNet_CmdReturnResult;

typedef uint8_t LumNet_Operation;

typedef uint16_t LumNet_Address;

LUM_NET_PACK(struct LumNet_CommandRequestHeader {
  LumNet_Address   address;
  LumNet_Operation operation; // high nibble contains command type and low
                              // nibble contains command sub-type
  uint8_t  magicNumber;       // Client should set this to magic number
  uint16_t transactionID;
  uint16_t payloadLength;
});

LUM_NET_PACK(struct LumNet_CommandResponseHeader {
  LumNet_Address   address;
  LumNet_Operation operation; // high nibble contains command type and low
                              // nibble contains command sub-type
  LumNet_CmdReturnResult commandReturnResult;
  uint16_t               transactionID;
  uint16_t               payloadLength;
});

LUM_NET_PACK(struct LumNet_CommandRequest {
  struct LumNet_CommandRequestHeader commandHeader;
  uint8_t                            payload[1];
});

LUM_NET_PACK(struct LumNet_CommandResponse {
  struct LumNet_CommandResponseHeader commandHeader;
  uint8_t                             payload[1];
});

LUM_NET_PACK(struct LumNet_CommandListPreamble {
  struct LumNet_CommonHeader versionPayloadHeader;
});

LUM_NET_PACK(struct LumNet_CommandRequestListHeader {
  uint16_t listTransactionID;
  uint16_t listPayloadLength; // Used by client in request list
});

LUM_NET_PACK(struct LumNet_CommandRequestListReturnResultHeader {
  uint16_t listTransactionID;
  LumNet_CmdRespListReturnResult
      returnResult; // Used by sensor in validation of request list
});

LUM_NET_PACK(struct LumNet_CommandResponseListHeader {
  uint16_t reserved;
  uint16_t listPayloadLength;
});

LUM_NET_PACK(struct LumNet_CommandRequestListValidationPayload {
  struct LumNet_CommandListPreamble preamble;
  struct LumNet_CommandRequestListReturnResultHeader
      commandRequestListReturnResultHeader;
});

LUM_NET_PACK(struct LumNet_CommandRequestListPayload {
  struct LumNet_CommandListPreamble      preamble;
  struct LumNet_CommandRequestListHeader commandRequestListHeader;
  struct LumNet_CommandRequest           commands[1];
});

LUM_NET_PACK(struct LumNet_CommandUrgentRequestListPayload {
  struct LumNet_CommandListPreamble      preamble;
  struct LumNet_CommandRequestListHeader commandRequestListHeader;
  struct LumNet_CommandRequest           commands[1];
});

LUM_NET_PACK(struct LumNet_CommandResponseListPayload {
  struct LumNet_CommandListPreamble       preamble;
  struct LumNet_CommandResponseListHeader responseListHeader;
  struct LumNet_CommandResponse           commands[1];
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

static inline struct LumNet_Version LumNet_GetCommandProtocolSemVer(void) {
  return LumNet_VersionBuilder(LUM_NET_CMD_PROTOCOL_SEM_VER_MAJOR,
                               LUM_NET_CMD_PROTOCOL_SEM_VER_MINOR,
                               LUM_NET_CMD_PROTOCOL_SEM_VER_PATCH);
}

  //@}

#endif /* LUM_NET_COMMAND_H */
