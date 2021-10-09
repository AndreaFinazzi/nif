/**
 * @file   Status.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-level file to include all of LumNet's status functionality
 *         which provides subscription based payloads via TCP.
 */

#ifndef LUM_NET_STATUS_H
#define LUM_NET_STATUS_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <LumNet/Common.h>
#include <LumNet/Command.h>
#include <LumNet/Status/AddrOps.h>
#include <LumNet/Status/Payloads.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief Semantic Major Version of the LumNet Status functionality.
 */
#define LUM_NET_STATUS_PROTOCOL_SEM_VER_MAJOR 0

/**
 * @brief Semantic Minor Version of the LumNet Status functionality.
 */
#define LUM_NET_STATUS_PROTOCOL_SEM_VER_MINOR 0

/**
 * @brief Semantic Patch Version of the LumNet Status functionality.
 */
#define LUM_NET_STATUS_PROTOCOL_SEM_VER_PATCH 0

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

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

static inline struct LumNet_Version LumNet_GetStatusProtocolSemVer(void) {
  static struct LumNet_Version version = {
      LUM_NET_STATUS_PROTOCOL_SEM_VER_MAJOR,
      LUM_NET_STATUS_PROTOCOL_SEM_VER_MINOR,
      LUM_NET_STATUS_PROTOCOL_SEM_VER_PATCH};

  return version;
}

  //@}

#endif // LUM_NET_STATUS_H
