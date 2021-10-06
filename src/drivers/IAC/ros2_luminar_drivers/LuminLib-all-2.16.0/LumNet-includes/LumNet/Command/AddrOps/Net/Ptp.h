/**
 * @file   Ptp.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Command address and operation definitions specific to
 *         the Configuring Network PTP.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_NET_PTP_H
#define LUM_NET_COMMAND_ADDR_OPS_NET_PTP_H

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
 * @brief Command Address for setting/getting ptp mode
 */
#define LUM_NET_CMD_ADDR_NET_PTP_MODE                                          \
  LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET,                          \
                           LUM_NET_CMD_ADDR_INDEX_NET_PTP,                     \
                           LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE_OPS_ALLOWED(...)                 \
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
 * @brief Command Address for setting delay interval
 */
#define LUM_NET_CMD_ADDR_NET_PTP_DELAY_INTERVAL                                \
  LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET,                          \
                           LUM_NET_CMD_ADDR_INDEX_NET_PTP,                     \
                           LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_ALLOWED(...)                    \
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
 * @brief Command Address for getting ptp status
 */
#define LUM_NET_CMD_ADDR_NET_PTP_STATUS                                        \
  LUM_NET_CMD_MAKE_ADDRESS(LUM_NET_CMD_ADDR_BASE_NET,                          \
                           LUM_NET_CMD_ADDR_INDEX_NET_PTP,                     \
                           LUM_NET_CMD_ADDR_OFFSET_NET_PTP_STATUS)

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_NET_PTP_STATUS_ALLOWED(...)                   \
  _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                         \
                             LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__)

/**
 * @brief Verify whether an address can perform any of the variadic pairs of
 *        operation types and subtypes by routing it to the appropriate address
 *        offset macro.
 */
#define _LUM_NET_CMD_VERIFY_ADDR_INDEX_NET_PTP_OPS_ALLOWED(addr, ...)          \
  ((LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE == _LUM_NET_GET_ADDR_OFFSET(addr))    \
       ? _LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE_OPS_ALLOWED(__VA_ARGS__)        \
       : (LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL ==                    \
          _LUM_NET_GET_ADDR_OFFSET(addr))                                      \
             ? _LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_ALLOWED(__VA_ARGS__)     \
             : (LUM_NET_CMD_ADDR_OFFSET_NET_PTP_STATUS ==                      \
                _LUM_NET_GET_ADDR_OFFSET(addr))                                \
                   ? _LUM_NET_CMD_ADDR_OFFSET_NET_PTP_STATUS_ALLOWED(          \
                         __VA_ARGS__)                                          \
                   : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{
enum LumNet_CmdAddrOffsetNetPtp {
  LUM_NET_CMD_ADDR_OFFSET_NET_PTP_MODE = 0x01, //!> LumNet_PtpMode
  LUM_NET_CMD_ADDR_OFFSET_NET_PTP_DELAY_INTERVAL =
      0x02, //!> LumNet_PtpTypeSigned , range of delay interval is -1 to 5 that
            // corresponds to 1/2 of a second - 32 seconds
  LUM_NET_CMD_ADDR_OFFSET_NET_PTP_STATUS = 0x03 //!> LumNet_PtpStatus
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

#endif // LUM_NET_COMMAND_ADDR_OPS_NET_PTP_H
