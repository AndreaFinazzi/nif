/**
 * @file   Health.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Command address and operation definitions specific to
 *         the System Health.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_SYS_HEALTH_H
#define LUM_NET_COMMAND_ADDR_OPS_SYS_HEALTH_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

#define _LUM_NET_CMD_VERIFY_ADDR_INDEX_SYS_HEALTH_OPS_ALLOWED(addr, ...) (false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{
enum LumNet_CmdAddrOffsetSysHealth {
  LUM_NET_CMD_ADDR_OFFSET_SYS_HEALTH_INTERLOCK_STATES =
      0x01, //!> LumNet_Interlocks
  LUM_NET_CMD_ADDR_OFFSET_SYS_HEALTH_SYSTEM_TEMPERATURE =
      0x02, //!> LumNet_SystemTemperature
  LUM_NET_CMD_ADDR_OFFSET_SYS_SCANNING_SYNCHRONIZED_STATE =
      0x03, //!> LumNet_Bool
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

#endif // LUM_NET_COMMAND_ADDR_OPS_SYS_HEALTH_H
