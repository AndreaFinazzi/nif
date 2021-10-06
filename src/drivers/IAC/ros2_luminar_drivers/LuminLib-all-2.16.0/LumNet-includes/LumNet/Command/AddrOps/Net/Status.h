/**
 * @file   Status.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Command address and operation definitions specific to
 *         the Network Status.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_NET_STATUS_H
#define LUM_NET_COMMAND_ADDR_OPS_NET_STATUS_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief Verify whether an address can perform any of the variadic pairs of
 *        operation types and subtypes by routing it to the appropriate address
 *        offset macro.
 */
#define _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_STATUS_OPS_ALLOWED(addr, ...) (false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

enum LumNet_CmdAddrOffsetNetStatus {
  LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENDPOINT = 0x01, //!> LumNet_EndPoint
  LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENABLE   = 0x02  //!> LumNet_Bool

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

#endif // LUM_NET_COMMAND_ADDR_OPS_NET_STATUS_H
