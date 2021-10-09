/**
 * @file   Net.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  TODO
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_NET_H
#define LUM_NET_COMMAND_ADDR_OPS_NET_H

/*********************************/
/* Includes                      */
/*********************************/

//@{

#include <LumNet/Command/AddrOps.h>
#include <LumNet/Command/AddrOps/Net/Cmd.h>
#include <LumNet/Command/AddrOps/Net/Dsc.h>
#include <LumNet/Command/AddrOps/Net/Gen.h>
#include <LumNet/Command/AddrOps/Net/Ptp.h>
#include <LumNet/Command/AddrOps/Net/Status.h>
#include <LumNet/Command/AddrOps/Net/Data.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

#define _LUM_NET_CMD_VERIFY_ADDR_BASE_NET_OPS_ALLOWED(addr, ...)                            \
  ((LUM_NET_CMD_ADDR_INDEX_NET_GEN == _LUM_NET_GET_ADDR_INDEX(addr))                        \
       ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_GEN_OPS_ALLOWED(addr, __VA_ARGS__)              \
       : (LUM_NET_CMD_ADDR_INDEX_NET_CMD == _LUM_NET_GET_ADDR_INDEX(addr))                  \
             ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_CMD_OPS_ALLOWED(addr,                     \
                                                                  __VA_ARGS__)              \
             : (LUM_NET_CMD_ADDR_INDEX_NET_DSC ==                                           \
                _LUM_NET_GET_ADDR_INDEX(addr))                                              \
                   ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_DSC_OPS_ALLOWED(                    \
                         addr, __VA_ARGS__)                                                 \
                   : (LUM_NET_CMD_ADDR_INDEX_NET_STATUS ==                                  \
                      _LUM_NET_GET_ADDR_INDEX(addr))                                        \
                         ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_STATUS_OPS_ALLOWED(           \
                               addr, __VA_ARGS__)                                           \
                         : (LUM_NET_CMD_ADDR_INDEX_NET_PTP ==                               \
                            _LUM_NET_GET_ADDR_INDEX(addr))                                  \
                               ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_PTP_OPS_ALLOWED(        \
                                     addr, __VA_ARGS__)                                     \
                               : (LUM_NET_CMD_ADDR_INDEX_NET_DATA ==                        \
                                  _LUM_NET_GET_ADDR_INDEX(addr))                            \
                                     ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_DATA_OPS_ALLOWED( \
                                           addr, __VA_ARGS__)                               \
                                     : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

// TODO: static assert that high address didn't go over the limit
enum LumNet_CmdAddrIndexNet {
  LUM_NET_CMD_ADDR_INDEX_NET_GEN    = 0x01,
  LUM_NET_CMD_ADDR_INDEX_NET_CMD    = 0x02,
  LUM_NET_CMD_ADDR_INDEX_NET_DSC    = 0x03,
  LUM_NET_CMD_ADDR_INDEX_NET_STATUS = 0x04,
  LUM_NET_CMD_ADDR_INDEX_NET_PTP    = 0x05,
  LUM_NET_CMD_ADDR_INDEX_NET_DATA   = 0x06,
  LUM_NET_CMD_HIGH_ADDR_INDEX_NET   = 0x07
};

// TODO
/*
// Lidar Data Endpoint
// Read Volatile, Read Persistent, Read Factory Default, Write Volatile, Write
Persistent, Factory Reset Persistent #define
_LUM_NET_CMD_NET_DATA_INDEX_LIDAR_DATA_ENDPOINT_OPS_ALLOWED(...) (false)

// Lidar Data Enable
// Read Volatile, Read Persistent, Read Factory Default, Write Volatile, Write
Persistent, Factory Reset Persistent #define
LUM_NET_CMD_NET_DATA_INDEX_LIDAR_DATA_ENABLE(...) (false)
*/

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

#endif // LUM_NET_COMMAND_ADDR_OPS_NET_H
