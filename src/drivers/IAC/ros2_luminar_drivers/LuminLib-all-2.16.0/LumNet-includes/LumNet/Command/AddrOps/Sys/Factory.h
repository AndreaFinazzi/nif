/**
 * @file   Factory.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Command address and operation definitions specific to
 *         the System Factory.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_SYS_FACTORY_H
#define LUM_NET_COMMAND_ADDR_OPS_SYS_FACTORY_H

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
#define _LUM_NET_CMD_VERIFY_ADDR_INDEX_SYS_FACTORY_OPS_ALLOWED(addr, ...)      \
  (false)

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

//@}

#endif // LUM_NET_COMMAND_ADDR_OPS_SYS_FACTORY_H
