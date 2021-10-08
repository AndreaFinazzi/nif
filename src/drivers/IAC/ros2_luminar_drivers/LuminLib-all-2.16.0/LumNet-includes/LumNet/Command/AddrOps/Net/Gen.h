/**
 * @file   Gen.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Command address and operation definitions specific to
 *         the General Networking.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_NET_GEN_H
#define LUM_NET_COMMAND_ADDR_OPS_NET_GEN_H

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
 * @brief Command Address for General Networking Network Type
 */
#define LUM_NET_CMD_ADDR_NET_GEN_NETWORK_TYPE                                  \
  LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET,                          \
                           LUM_NET_CMD_ADDR_INDEX_NET_GEN,                     \
                           LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NETWORK_TYPE)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_NETWORK_TYPE_OPS_ALLOWED(...)             \
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
 * @brief Command Address for General Networking Mac Address
 */
#define LUM_NET_CMD_ADDR_NET_GEN_MAC_ADDRESS                                   \
  LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET,                          \
                           LUM_NET_CMD_ADDR_INDEX_NET_GEN,                     \
                           LUM_NET_CMD_ADDR_OFFSET_NET_GEN_MAC_ADDRESS)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_GEN_MAC_ADDRESS_OPS_ALLOWED(...)          \
  _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                         \
                             LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__)

/**
 * @brief Command Address for General Networking IP Address
 */
#define LUM_NET_CMD_ADDR_NET_GEN_IP_ADDRESS                                    \
  LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET,                          \
                           LUM_NET_CMD_ADDR_INDEX_NET_GEN,                     \
                           LUM_NET_CMD_ADDR_OFFSET_NET_GEN_IP_ADDRESS)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_GEN_IP_ADDRESS_OPS_ALLOWED(...)           \
  _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                         \
                             LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||  \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                     \
                                 LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,            \
                                 __VA_ARGS__) ||                               \
      _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                    \
                                 LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,            \
                                 __VA_ARGS__)

/**
 * @brief Verify whether an address can perform any of the variadic pairs of
 *        operation types and subtypes by routing it to the appropriate address
 *        offset macro.
 */
#define _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_GEN_OPS_ALLOWED(addr, ...)          \
  ((LUM_NET_CMD_ADDR_OFFSET_NET_GEN_IP_ADDRESS ==                              \
    _LUM_NET_GET_ADDR_OFFSET(addr))                                            \
       ? _LUM_NET_CMD_ADDR_OFFSET_NET_GEN_IP_ADDRESS_OPS_ALLOWED(__VA_ARGS__)  \
       : (LUM_NET_CMD_ADDR_OFFSET_NET_GEN_MAC_ADDRESS ==                       \
          _LUM_NET_GET_ADDR_OFFSET(addr))                                      \
             ? _LUM_NET_CMD_ADDR_OFFSET_NET_GEN_MAC_ADDRESS_OPS_ALLOWED(       \
                   __VA_ARGS__)                                                \
             : (LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NETWORK_TYPE ==                \
                _LUM_NET_GET_ADDR_OFFSET(addr))                                \
                   ? _LUM_NET_CMD_ADDR_OFFSET_NET_NETWORK_TYPE_OPS_ALLOWED(    \
                         __VA_ARGS__)                                          \
                   : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

/**
 * @brief Enumeration of Address Offsets available for General Networking.
 */
enum LumNet_CmdAddrOffsetNetGen {
  LUM_NET_CMD_ADDR_OFFSET_NET_GEN_IP_ADDRESS   = 0x01, //!> LumNet_IpAddress
  LUM_NET_CMD_ADDR_OFFSET_NET_GEN_MAC_ADDRESS  = 0x02, //!> LumNet_MacAddress
  LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NETWORK_TYPE = 0x03, //!> LumNet_NetworkType
  LUM_NET_CMD_ADDR_OFFSET_NET_GEN_NICKNAME     = 0x04, //!> LumNet_Nickname
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

#endif // LUM_NET_COMMAND_ADDR_OPS_NET_GEN_H
