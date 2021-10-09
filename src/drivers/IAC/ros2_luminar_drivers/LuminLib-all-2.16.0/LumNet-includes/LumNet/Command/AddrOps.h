/**
 * @file   AddrOps.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-level file to include all addresses and operations
 *         available to LumNet's command functionality.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_H
#define LUM_NET_COMMAND_ADDR_OPS_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <LumNet/Common.h>
#include <LumNet/Command/VerifyOps.h>
#include <LumNet/Command/AddrOps/Sys.h>
#include <LumNet/Command/AddrOps/Lidar.h>
#include <LumNet/Command/AddrOps/Net.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief Given an 8-bit operation, extract the type from the high nibble.
 */
#define LUM_NET_EXTRACT_OP_TYPE(op)                                            \
  ((op & _LUM_NET_HIGH_NIBBLE_MASK) >> LUM_NET_NIBBLE_LEN)

/**
 * @brief Given an 8-bit operation, extract the subtype from the low nibble.
 */
#define LUM_NET_EXTRACT_OP_SUBTYPE(op) (op & _LUM_NET_LOW_NIBBLE_MASK)

/**
 * @brief Given an operation type and operation subtype, create an
 *        8-bit operation.
 */
#define LUM_NET_MAKE_OP(opType, opSubType)                                     \
  ((opType << LUM_NET_NIBBLE_LEN) | (opSubType))

/**
 * @brief Given a 4-bit base, 4-bit index, and a 8-bit offset, construct a
 *        16-bit address.
 */
#define LUM_NET_CMD_MAKE_ADDRESS(base, index, offset)                          \
  (((((base & _LUM_NET_LOW_NIBBLE_MASK) << LUM_NET_NIBBLE_LEN) +               \
     (index & _LUM_NET_LOW_NIBBLE_MASK))                                       \
    << LUM_NET_BYTE_LEN) +                                                     \
   (offset & _LUM_NET_BYTE_MASK))

/**
 * @brief Given a 16-bit address, extract the 4-bit base.
 */
#define _LUM_NET_GET_ADDR_BASE(addr) ((addr >> 12) & (0x0F))

/**
 * @brief Given a 16-bit address, extract the 4-bit index.
 */
#define _LUM_NET_GET_ADDR_INDEX(addr) ((addr >> 8) & (0x0F))

/**
 * @brief Given a 16-bit address, extract the 8-bit offset.
 */
#define _LUM_NET_GET_ADDR_OFFSET(addr) (addr & (0xFF))

/**
 * @brief Verify whether an address can perform any of the variadic pairs of
 *        operation types and subtypes by routing it to the appropriate address
 *        base macro.
 */
#define LUM_NET_CMD_VERIFY_ADDR_OPS_ALLOWED(addr, ...)                         \
  ((LUM_NET_CMD_ADDR_BASE_SYS == _LUM_NET_GET_ADDR_BASE(addr))                 \
       ? _LUM_NET_CMD_VERIFY_ADDR_BASE_SYS_OPS_ALLOWED(addr, __VA_ARGS__)      \
       : (LUM_NET_CMD_ADDR_BASE_NET == _LUM_NET_GET_ADDR_BASE(addr))           \
             ? _LUM_NET_CMD_VERIFY_ADDR_BASE_NET_OPS_ALLOWED(addr,             \
                                                             __VA_ARGS__)      \
             : (LUM_NET_CMD_ADDR_BASE_LIDAR == _LUM_NET_GET_ADDR_BASE(addr))   \
                   ? _LUM_NET_CMD_VERIFY_ADDR_BASE_LIDAR_OPS_ALLOWED(          \
                         addr, __VA_ARGS__)                                    \
                   : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

/**
 * @brief Enumeration representing values for the address base which is
 *        the first 4-bits of the 16-bit command address field.
 */
enum LumNet_CmdAddrBase {
  LUM_NET_CMD_ADDR_BASE_SYS   = 0x01,
  LUM_NET_CMD_ADDR_BASE_NET   = 0x02,
  LUM_NET_CMD_ADDR_BASE_LIDAR = 0x03,
  LUM_NET_CMD_ADDR_BASE_HIGH  = 0x04,
  // Constrain upper ranges of the defined base groups
  LUM_NET_CMD_HIGH_ADDR_BASE_SYS   = LUM_NET_CMD_ADDR_BASE_NET,
  LUM_NET_CMD_HIGH_ADDR_BASE_NET   = LUM_NET_CMD_ADDR_BASE_LIDAR,
  LUM_NET_CMD_HIGH_ADDR_BASE_LIDAR = LUM_NET_CMD_ADDR_BASE_HIGH
};

/**
 * @brief Enumeration representing values for the operation type which is
 *        the first 4-bits from the 16-bit command address field.
 */
enum LumNet_CmdOpType {
  LUM_NET_CMD_OP_TYPE_READ          = 0x01,
  LUM_NET_CMD_OP_TYPE_WRITE         = 0x02,
  LUM_NET_CMD_OP_TYPE_FACTORY_RESET = 0x03,
};

/**
 * @brief Enumeration representing values for the operation type which is
 *        the second set of 4-bits from the 16-bit command address field.
 */
enum LumNet_CmdOpSubtype {
  LUM_NET_CMD_OP_SUBTYPE_VOLATILE               = 0x01,
  LUM_NET_CMD_OP_SUBTYPE_PERSISTENT             = 0x02,
  LUM_NET_CMD_OP_SUBTYPE_FACTORY_DEFAULT        = 0x03,
  LUM_NET_CMD_OP_SUBTYPE_VOLATILE_TO_PERSISTENT = 0x04,
  LUM_NET_CMD_OP_SUBTYPE_PERSISTENT_TO_VOLATILE = 0x05,
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

#endif // LUM_NET_COMMAND_ADDR_OPS_H
