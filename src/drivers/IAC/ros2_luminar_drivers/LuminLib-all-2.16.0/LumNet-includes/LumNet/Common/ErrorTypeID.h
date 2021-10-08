/**
 * @file   ErrorTypeID.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Enumeration of different types of payloads that LumNet supports.
 */

#ifndef LUM_NET_COMMON_ERROR_TYPE_ID_H
#define LUM_NET_COMMON_ERROR_TYPE_ID_H

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

typedef uint8_t LumNet_ErrorTypeID;

enum LumNet_ErrorTypeIDs {
  LUM_NET_ERR_TYPE_ID_NONE                           = 0x00,
  LUM_NET_ERR_TYPE_ID_VERSION                        = 0x01,
  LUM_NET_ERR_TYPE_ID_DSC_SETUP_DEST_ENDPOINT        = 0x02,
  LUM_NET_ERR_TYPE_ID_DSC_START_LIDAR_DATA_STREAM    = 0x03,
  LUM_NET_ERR_TYPE_ID_DSC_INVALID_NETWORK_CONNECTION = 0x04,
  LUM_NET_ERR_TYPE_ID_DSC_INIT_LIDAR_DATA_STREAM     = 0x05
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
