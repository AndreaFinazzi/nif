/**
 * @file   Common.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-Level file to include the subset of common definitions and
 *         common functionality to support interfacing with legacy sensors.
 */

#ifndef LEGACY_LUM_NET_COMMON_H
#define LEGACY_LUM_NET_COMMON_H

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
 * @brief The number of bytes in a Serial Number.
 */
#define LEGACY_LUM_NET_SERIAL_NUMBER_BYTE_LEN (18)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

LUM_NET_PACK(struct LegacyLumNet_Version {
  LumNet_VersionType major;
  LumNet_VersionType minor;
  LumNet_VersionType patch;
});

LUM_NET_PACK(struct LegacyLumNet_SemVersionedPayloadHeader {
  struct LegacyLumNet_Version semver;
  LumNet_PayloadTypeID        payloadTypeID;
});

LUM_NET_PACK(struct LegacyLumNet_CommonHeader {
  LumNet_Sentinel                               sentinel;
  struct LegacyLumNet_SemVersionedPayloadHeader semVersionedHeader;
});

LUM_NET_PACK(struct LegacyLumNet_GenericResponse {
  struct LegacyLumNet_CommonHeader commonHeader;
  LumNet_ErrorTypeID               error;
});

LUM_NET_PACK(struct LegacyLumNet_SerialNumber {
  uint8_t value[LEGACY_LUM_NET_SERIAL_NUMBER_BYTE_LEN];
});

LUM_NET_PACK(struct LegacyLumNet_GenericSnResponse {
  struct LegacyLumNet_GenericResponse response;
  struct LegacyLumNet_SerialNumber    serialNumber;
  LumNet_PayloadTypeID                requestPayloadTypeID;
});

LUM_NET_PACK(struct LegacyLumNet_Nickname { uint32_t value; });

LUM_NET_PACK(struct LegacyV2LumNet_GenericSnResponse {
  struct LumNet_GenericResponse    response;
  struct LegacyLumNet_SerialNumber serialNumber;
  LumNet_PayloadTypeID             requestPayloadTypeID;
});

LUM_NET_PACK(struct LegacyLumNet_Interlocks {
  LumNet_Bool systemOk;
  LumNet_Bool laserReady;
  LumNet_Bool scanning;
  LumNet_Bool laserArmed;
  LumNet_Bool dataStreaming;
});

LUM_NET_PACK(struct LegacyLumNet_State {
  struct LegacyLumNet_Interlocks interLocks;
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

//@}

#endif // LEGACY_LUM_NET_COMMON_H
