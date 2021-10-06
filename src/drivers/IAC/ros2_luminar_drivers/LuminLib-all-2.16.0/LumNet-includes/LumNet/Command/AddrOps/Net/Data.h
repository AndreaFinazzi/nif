/**
 * @file   Data.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Command address and operation definitions specific to
 *         the Network Data.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_NET_DATA_H
#define LUM_NET_COMMAND_ADDR_OPS_NET_DATA_H

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
 * @brief Command Address for Network Data Endpoint
 */
#define LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENDPOINT                          \
  LUM_NET_CMD_MAKE_ADDRESS(                                                    \
      LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_DATA,              \
      LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENDPOINT)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENDPOINT_OPS_ALLOWED(...) \
  _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                         \
                             LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||  \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                     \
                                 LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,            \
                                 __VA_ARGS__) ||                               \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                    \
                                 LUM_NET_CMD_OP_SUBTYPE_VOLATILE,              \
                                 __VA_ARGS__) ||                               \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                    \
                                 LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,            \
                                 __VA_ARGS__)

/**
 * @brief Command Address for Network Data Enable
 */
#define LUM_NET_CMD_ADDR_NET_DATA_LIDAR_DATA_ENABLE                            \
  LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET,                          \
                           LUM_NET_CMD_ADDR_INDEX_NET_DATA,                    \
                           LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENABLE)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENABLE_OPS_ALLOWED(...)   \
  _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                         \
                             LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||  \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                     \
                                 LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,            \
                                 __VA_ARGS__) ||                               \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                    \
                                 LUM_NET_CMD_OP_SUBTYPE_VOLATILE,              \
                                 __VA_ARGS__) ||                               \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                    \
                                 LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,            \
                                 __VA_ARGS__)

/**
 * @brief Verify whether an address can perform any of the variadic pairs of
 *        operation types and subtypes by routing it to the appropriate address
 *        offset macro.
 */
#define _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_DATA_OPS_ALLOWED(addr, ...)          \
  ((LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENDPOINT ==                     \
    _LUM_NET_GET_ADDR_OFFSET(addr))                                             \
       ? _LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENDPOINT_OPS_ALLOWED(     \
             __VA_ARGS__)                                                       \
       : (LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENABLE ==                 \
          _LUM_NET_GET_ADDR_OFFSET(addr))                                       \
             ? _LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENABLE_OPS_ALLOWED( \
                   __VA_ARGS__)                                                 \
             : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

/**
 * @brief Enumeration of Address Offsets available for Network Data.
 */
enum LumNet_CmdAddrOffsetNetData {
  LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENDPOINT =
      0x01, //!> LumNet_EndPoint
  LUM_NET_CMD_ADDR_OFFSET_NET_DATA_LIDAR_DATA_ENABLE = 0x02, //!> LumNet_Bool
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

#endif // LUM_NET_COMMAND_ADDR_OPS_NET_DATA_H
