/**
 * @file   Sys.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  TODO
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_SYS_H
#define LUM_NET_COMMAND_ADDR_OPS_SYS_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <LumNet/Command/AddrOps.h>
#include <LumNet/Command/AddrOps/Sys/Factory.h>
#include <LumNet/Command/AddrOps/Sys/Health.h>
#include <LumNet/Command/AddrOps/Sys/Ident.h>
#include <LumNet/Command/AddrOps/Sys/Mode.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

#define _LUM_NET_CMD_VERIFY_ADDR_BASE_SYS_OPS_ALLOWED(addr, ...)                \
  ((LUM_NET_CMD_ADDR_INDEX_SYS_FACTORY == _LUM_NET_GET_ADDR_INDEX(addr))        \
       ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_SYS_FACTORY_OPS_ALLOWED(addr,           \
                                                                __VA_ARGS__)    \
       : (LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH == _LUM_NET_GET_ADDR_INDEX(addr))   \
             ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_SYS_HEALTH_OPS_ALLOWED(           \
                   addr, __VA_ARGS__)                                           \
             : (LUM_NET_CMD_ADDR_INDEX_SYS_IDENT ==                             \
                _LUM_NET_GET_ADDR_INDEX(addr))                                  \
                   ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_SYS_IDENT_OPS_ALLOWED(      \
                         addr, __VA_ARGS__)                                     \
                   : (LUM_NET_CMD_ADDR_INDEX_SYS_MODE ==                        \
                      _LUM_NET_GET_ADDR_INDEX(addr))                            \
                         ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_SYS_MODE_OPS_ALLOWED( \
                               addr, __VA_ARGS__)                               \
                         : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

enum LumNet_CmdAddrIndexSys {
  LUM_NET_CMD_ADDR_INDEX_SYS_FACTORY = 0x01,
  LUM_NET_CMD_ADDR_INDEX_SYS_HEALTH  = 0x02,
  LUM_NET_CMD_ADDR_INDEX_SYS_IDENT   = 0x03,
  LUM_NET_CMD_ADDR_INDEX_SYS_MODE    = 0x04,
  LUM_NET_CMD_HIGH_ADDR_INDEX_SYS    = 0x05
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

#endif // LUM_NET_COMMAND_ADDR_OPS_SYS_H
