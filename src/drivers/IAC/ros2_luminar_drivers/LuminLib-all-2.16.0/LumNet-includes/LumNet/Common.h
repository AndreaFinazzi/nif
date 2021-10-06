/**
 * @file   Common.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-Level file to include common definitions and functionality
 *         across LumNet.
 */

#ifndef LUM_NET_COMMON_H
#define LUM_NET_COMMON_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <stdbool.h>
#include <stdint.h>
#include <LumNet/Common/PayloadTypeID.h>
#include <LumNet/Common/ErrorTypeID.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

/**
 * @brief A mask that can be used to retrieve the high nibble of a byte.
 */
#define _LUM_NET_HIGH_NIBBLE_MASK (0xF0)

/**
 * @brief A mask that can be used to retrieve the low nibble of a byte.
 */
#define _LUM_NET_LOW_NIBBLE_MASK (0x0F)

/**
 * @brief A mask that can be used to retrieve the least significant byte.
 */
#define _LUM_NET_BYTE_MASK (0xFF)

/**
 * @brief The length of a nibble.
 */
#define LUM_NET_NIBBLE_LEN (4)

/**
 * @brief The length of a byte.
 */
#define LUM_NET_BYTE_LEN (8)

/**
 * @brief A special 32-bit value used in LumNet messages, found at the begining,
 *        denoting that the payload was directed towards a Sensor Head. The
 *        special value, in ASCII, is 'H' '2' 'S' 'H' which is an abbreviation
 *        for H2 Sensor Head.
 */
#define LUM_NET_SENTINEL (0x48325348)

/**
 * @brief Typically enumerations of value 0 are invalid types in LumNet.
 */
#define LUM_NET_INVALID_TYPE (0x00)

/**
 * @brief The number of bytes in a Serial Number.
 */
#define LUM_NET_SERIAL_NUMBER_BYTE_LEN (20)

/**
 * @brief The number of bytes in a Description.
 */
#define LUM_NET_DESCR_BYTE_LEN (64)
/**
 * @brief The number of bytes in a Nickname.
 */
#define LUM_NET_NICKNAME_BYTE_LEN (sizeof(uint16_t))

/**
 * @brief The number of bytes in a MAC Address
 */
#define LUM_NET_MAC_ADDRESS_BYTE_LEN (6)

#ifndef LUM_NET_DISABLE_PACK

#if defined(_MSC_VER)
#define LUM_NET_PACK(...)                                                      \
  __pragma(pack(push, 1)) __VA_ARGS__ __pragma(pack(pop))
#else
#define LUM_NET_PACK(...) __VA_ARGS__ __attribute__((__packed__))
#endif

#else
#define LUM_NET_PACK(...) __VA_ARGS__
#endif

/**
 * @brief Expand a macro to itself.
 */
#define _LUM_NET_EXPAND(x) x

/**
 * @brief Prefix an extra argument in the variadic argument list.
 */
#define _LUM_NET_PREFIX(...) 0, ##__VA_ARGS__

/**
 * @brief Return the 10th parameter inputted into the variadic argument list.
 */
#define _LUM_NET_COUNT_ARGS_SELECT_IMPL(a, b, c, d, e, f, g, h, i, j, ...) j

/**
 * @brief An implementation of counting the amount of variadic arguments.
 */
#define _LUM_NET_COUNT_ARGS_IMPL(...)                                          \
  _LUM_NET_EXPAND(                                                             \
      _LUM_NET_COUNT_ARGS_SELECT_IMPL(__VA_ARGS__, 8, 7, 6, 5, 4, 3, 2, 1, 0))

/**
 * @brief A macro which returns the numeric amount of variadic arguments passed
 * in. This function works for N > 0, compatible with:
 * - C99
 * - C99 GNU GCC
 * - C99 clang GCC
 * - C99 vc
 * - C11
 * - C11 GNU GCC
 * - C11 clang GCC
 * - C++ GNU GCC
 * - C++ clang GCC
 * - C++ vc
 */
#define _LUM_NET_COUNT_ARGS(...)                                               \
  _LUM_NET_COUNT_ARGS_IMPL(_LUM_NET_PREFIX(__VA_ARGS__))

/**
 * @brief Macro concatenation of two arguments.
 */
#define _LUM_NET_PRIMITIVE_CAT(x, y) x##y

/**
 * @brief Macro expansion of the concatenation of two arguments.
 */
#define _LUM_NET_CAT(x, y) _LUM_NET_PRIMITIVE_CAT(x, y)

/**
 * @brief Macro expansion of the left parenthesis.
 */
#define _LUM_NET_L_PARENS (

/**
 * @brief Macro expansion of the right parenthesis.
 */
#define _LUM_NET_R_PARENS )

/**
 * @brief Create a function name with the number of arguments as the suffix and
 * call the function with the supplied variadic argument list.
 */
#define _LUM_NET_MAKE_VARIADIC_NAME(fnc, ...)                                  \
  _LUM_NET_CAT(fnc, _LUM_NET_COUNT_ARGS(__VA_ARGS__))                          \
  _LUM_NET_L_PARENS __VA_ARGS__ _LUM_NET_R_PARENS

#define _LUM_NET_MAKE_BOOL(arg)                                                \
  ((arg) ? LUM_NET_BOOL_VALUE_TRUE : LUM_NET_BOOL_VALUE_FALSE)
//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

typedef uint8_t LumNet_PtpMode;

enum LumNet_PtpModes {
  LUM_NET_PTP_MODE_E2E  = 0,
  LUM_NET_PTP_MODE_P2P  = 1,
  LUM_NET_PTP_NUM_MODES = 2
};

typedef uint32_t LumNet_Sentinel;

/**
 * @brief Typedef for boolean to ensure its size is the length of a byte.
 */
typedef uint8_t LumNet_Bool;

enum LumNet_BoolValues {
  LUM_NET_BOOL_VALUE_TRUE  = 1,
  LUM_NET_BOOL_VALUE_FALSE = 0
};

typedef uint8_t LumNet_VersionType;

enum LumNet_VersionTypes {
  LUM_NET_SEM_VERSION_CUSTOMER_TYPE = 0x00,
  LUM_NET_SEM_VERSION_ENGR_TYPE     = 0x01
};

LUM_NET_PACK(struct LumNet_Version {
  LumNet_VersionType patch;
  LumNet_VersionType minor;
  LumNet_VersionType major;
});

static inline struct LumNet_Version
LumNet_VersionBuilder(LumNet_VersionType major, LumNet_VersionType minor,
                      LumNet_VersionType patch) {
  struct LumNet_Version version = {patch, minor, major};
  return version;
}

typedef int32_t  LumNet_PtpTypeSigned;
typedef uint32_t LumNet_PtpType;
typedef float    LumNet_PtpTypeFloat;
typedef double   LumNet_PtpTypeDouble;

LUM_NET_PACK(struct LumNet_PtpTime {
  LumNet_PtpType seconds;
  LumNet_PtpType nanoSeconds;
});

/**
 * @brief Version compatibility check, conforming to semver versioning scheme
 */
static inline bool
LumNet_VersionIsCompatible(struct LumNet_Version semverParser,
                           struct LumNet_Version semverData) {
  // Handle the pre-release case
  if (semverData.major == 0 && (semverParser.minor != semverData.minor ||
                                semverParser.patch != semverData.patch)) {
    return false;
  }

  // Handle the normal case
  if (semverParser.major == semverData.major &&
      (semverParser.minor > semverData.minor ||
       (semverParser.minor == semverData.minor &&
        semverParser.patch >= semverData.patch))) {
    return true;
  } else {
    return false;
  }
}

LUM_NET_PACK(struct LumNet_SemVersionedPayloadHeader {
  LumNet_PayloadTypeID  payloadTypeID;
  struct LumNet_Version semver;
});

LUM_NET_PACK(struct LumNet_CommonHeader {
  LumNet_Sentinel                         sentinel;
  struct LumNet_SemVersionedPayloadHeader semVersionedHeader;
});

LUM_NET_PACK(struct LumNet_SerialNumber {
  uint8_t value[LUM_NET_SERIAL_NUMBER_BYTE_LEN];
});

LUM_NET_PACK(struct LumNet_Description {
  uint8_t value[LUM_NET_DESCR_BYTE_LEN];
});

typedef uint8_t LumNet_BootState;

/**
 * @brief Boot states that the system can be in, which are reported via
 * discovery.
 */
enum LumNet_BootStates {
  LUM_NET_BOOT_STATE_UNKNOWN      = 0,
  LUM_NET_BOOT_STATE_IN_PROGRESS  = 1,
  LUM_NET_BOOT_STATE_FAIL_TIMEOUT = 2,
  LUM_NET_BOOT_STATE_FAIL         = 3,
  LUM_NET_BOOT_STATE_SUCCESS      = 4,
};

LUM_NET_PACK(struct LumNet_Interlocks {
  LumNet_Bool systemOk;
  LumNet_Bool laserReady;
  LumNet_Bool azimuthScanning;
  LumNet_Bool elevationScanning;
  LumNet_Bool laserArmed;
  LumNet_Bool dataStreaming;
});

LUM_NET_PACK(struct LumNet_State {
  struct LumNet_Interlocks interLocks;
  uint8_t                  reserved;
  float                    bootCompletionPercentage;
  LumNet_BootState         bootState;
});

typedef uint32_t LumNet_IpAddress;

typedef uint16_t LumNet_Port;
typedef uint16_t LumNet_Nickname;

typedef uint16_t LumNet_MTU;

LUM_NET_PACK(struct LumNet_MacAddress {
  uint8_t value[LUM_NET_MAC_ADDRESS_BYTE_LEN];
});

LUM_NET_PACK(struct LumNet_NetworkConnection {
  LumNet_IpAddress ipAddress;
  LumNet_IpAddress subNetMask;
  LumNet_IpAddress gateway;
});
LUM_NET_PACK(struct LumNet_Alias { LumNet_Nickname alias; });

LUM_NET_PACK(struct LumNet_Endpoint {
  LumNet_IpAddress ipAddress;
  LumNet_Port      port;
});

LUM_NET_PACK(struct LumNet_NetworkConnectionEndpoint {
  LumNet_IpAddress ipAddress;
  LumNet_IpAddress subNetMask;
  LumNet_IpAddress gateway;
  LumNet_Port      port;
});

typedef uint8_t LumNet_NetworkType;

enum LumNet_NetworkTypes {
  LUM_NET_NETWORK_TYPE_UNDEFINED = 0x00,
  LUM_NET_NETWORK_TYPE_DEFAULT   = 0x01,
  LUM_NET_NETWORK_TYPE_DYNAMIC   = 0x02,
  LUM_NET_NETWORK_TYPE_STATIC    = 0x03
};

LUM_NET_PACK(struct LumNet_GenericResponse {
  struct LumNet_CommonHeader commonHeader;
  LumNet_ErrorTypeID         error;
});

LUM_NET_PACK(struct LumNet_GenericSnResponse {
  struct LumNet_GenericResponse response;
  struct LumNet_SerialNumber    serialNumber;
  LumNet_PayloadTypeID          requestPayloadTypeID;
});

LUM_NET_PACK(struct LumNet_PtpTimeResponse {
  struct LumNet_GenericResponse response;
  struct LumNet_PtpTime         time;
  LumNet_Nickname nickName; //!< Sensor Head's nickname identifier.
});
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

#endif // LUM_NET_COMMON_H
