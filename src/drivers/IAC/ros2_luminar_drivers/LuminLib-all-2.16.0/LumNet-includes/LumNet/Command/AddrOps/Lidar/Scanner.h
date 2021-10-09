/**
 * @file   Scanner.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Command address and operation definitions specific to
 *         the Lidar's Scanners.
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_LIDAR_SCANNER_H
#define LUM_NET_COMMAND_ADDR_OPS_LIDAR_SCANNER_H

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
 * @brief Command Address for Lidar Scanner Profile Generation
 */
#define LUM_NET_CMD_ADDR_LIDAR_SCANNER_GENERATE_SCAN_PROFILE                   \
  (LUM_NET_CMD_MAKE_ADDRESS(                                                   \
      LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER,       \
      LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE))

/**
 * @brief Verify whether an address offset can perform any of the variadic
 *        pairs of operation types and subtypes.
 */
#define _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_SCAN_PROFILE_OPS_ALLOWED( \
    ...)                                                                          \
  (_LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                          \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__))

#define LUM_NET_CMD_ADDR_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST       \
  (LUM_NET_CMD_MAKE_ADDRESS(                                                   \
      LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER,       \
      LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST))

#define _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST_OPS_ALLOWED( \
    ...)                                                                                      \
  (_LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                                       \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||                \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                                       \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,                              \
                              __VA_ARGS__) ||                                                 \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                                      \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||                \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                                      \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT, __VA_ARGS__))

#define LUM_NET_CMD_ADDR_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES     \
  (LUM_NET_CMD_MAKE_ADDRESS(                                                   \
      LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER,       \
      LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES))

#define _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES_OPS_ALLOWED( \
    ...)                                                                                        \
  (_LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                                         \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||                  \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                                         \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,                                \
                              __VA_ARGS__) ||                                                   \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                                        \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||                  \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                                        \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT, __VA_ARGS__))

#define LUM_NET_CMD_ADDR_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS                      \
  (LUM_NET_CMD_MAKE_ADDRESS(                                                   \
      LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER,       \
      LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS))

#define _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS_OPS_ALLOWED( \
    ...)                                                                       \
  (_LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                        \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) || \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                        \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,               \
                              __VA_ARGS__) ||                                  \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                       \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) || \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                       \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT, __VA_ARGS__))

#define LUM_NET_CMD_ADDR_LIDAR_SCANNER_VERTICAL_SYNC_MODE                      \
  (LUM_NET_CMD_MAKE_ADDRESS(                                                   \
      LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER,       \
      LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE))

#define _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE_OPS_ALLOWED( \
    ...)                                                                       \
  (_LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                        \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) || \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                        \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,               \
                              __VA_ARGS__) ||                                  \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                       \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) || \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                       \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT, __VA_ARGS__))

#define LUM_NET_CMD_ADDR_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET                    \
  (LUM_NET_CMD_MAKE_ADDRESS(                                                   \
      LUM_NET_CMD_ADDR_BASE_LIDAR, LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER,       \
      LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET))

#define _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET_OPS_ALLOWED( \
    ...)                                                                         \
  (_LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                          \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||   \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_READ,                          \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT,                 \
                              __VA_ARGS__) ||                                    \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                         \
                              LUM_NET_CMD_OP_SUBTYPE_VOLATILE, __VA_ARGS__) ||   \
   _LUM_NET_VERIFY_VARDIAC_OP(LUM_NET_CMD_OP_TYPE_WRITE,                         \
                              LUM_NET_CMD_OP_SUBTYPE_PERSISTENT, __VA_ARGS__))

/**
 * @brief Verify whether an address can perform any of the variadic pairs of
 *        operation types and subtypes by routing it to the appropriate address
 *        offset macro.
 */
#define _LUM_NET_CMD_VERIFY_ADDR_INDEX_LIDAR_SCANNER_OPS_ALLOWED(addr, ...)                                     \
  ((LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE ==                                   \
    _LUM_NET_GET_ADDR_OFFSET(addr))                                                                             \
       ? _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_SCAN_PROFILE_OPS_ALLOWED(                              \
             __VA_ARGS__)                                                                                       \
       : (LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST ==                            \
          _LUM_NET_GET_ADDR_OFFSET(addr))                                                                       \
             ? _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST_OPS_ALLOWED(            \
                   __VA_ARGS__)                                                                                 \
             : (LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES ==                    \
                _LUM_NET_GET_ADDR_OFFSET(addr))                                                                 \
                   ? _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES_OPS_ALLOWED(    \
                         __VA_ARGS__)                                                                           \
                   : (LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS ==                               \
                      _LUM_NET_GET_ADDR_OFFSET(addr))                                                           \
                         ? _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS_OPS_ALLOWED(               \
                               __VA_ARGS__)                                                                     \
                         : (LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE ==                         \
                            _LUM_NET_GET_ADDR_OFFSET(addr))                                                     \
                               ? _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE_OPS_ALLOWED(         \
                                     __VA_ARGS__)                                                               \
                               : (LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET ==                 \
                                  _LUM_NET_GET_ADDR_OFFSET(addr))                                               \
                                     ? _LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET_OPS_ALLOWED( \
                                           __VA_ARGS__)                                                         \
                                     : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

/**
 * @brief Enumeration of Address Offsets available for Lidar Scanner.
 */
enum LumNet_CmdAddrOffsetLidarScanner {
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_UNIFORM_SCAN_PROFILE_DEPRECATED =
      0x01, //!< LumNet_ScanProfileDeprecated
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_UNIFORM_UNIDIRECTIONAL_SCAN_PROFILE_DEPRECATED_V4 =
      0x02, //!< LumNet_ScanProfileDeprecatedV4
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_GAUSSIAN_UNIDIRECTIONAL_SCAN_PROFILE_DEPRECATED =
      0x03, //!< LumNet_GaussianScanProfileDeprecated
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PROFILE =
      0x04, //!< LumNet_GenericScanProfile
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST =
      0x05, //!< LumNet_ScanPlaylist
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_INTERLACING_CONFIGURATION =
      0x06, //!< LumNet_InterlacingConfiguration
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_SNAP_SCANS_TO_WHOLE_NUMBER_OF_LINES =
      0x07, //!< LumNet_Bool
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_OPTIMIZE_SNAPBACKS =
      0x08, //!< LumNet_Bool
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_MODE =
      0x09, //!< LumNet_VerticalSyncMode
  LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_VERTICAL_SYNC_OFFSET =
      0x0A, //!< LumNet_VerticalSyncOffsetMs
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

#endif // LUM_NET_COMMAND_ADDR_OPS_LIDAR_SCANNER_H
