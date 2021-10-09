/**
 * @file   LidarData.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Facilities for parsing Luminar Point Cloud Data packets
 *
 * The following shows the bit layout of Luminar Lidar Data Packets in version
 * 2.0.0 of the Luminar H Series Point Cloud Output Protocol as well as data
 * structures and definitions that can be used to parse and process the packets.
 *
 * PREAMBLE BIT LAYOUT:
 *  31           24 23           16 15            8 7            0
 * +---------------+---------------+---------------+--------------+
 * | Major         | Minor         | Patch         | Packet Type  |
 * +---------------+---------------+---------------+--------------+
 *
 *  31                           16 15            8 7            0
 * +-------------------------------+---------------+--------------+
 * | Fingerprint                   | seq number    | num rays     |
 * +-------------------------------+---------------+--------------+
 *
 *  31                                                           0
 * +--------------------------------------------------------------+
 * | Second timestamp according to the sender                     |
 * +--------------------------------------------------------------+
 *
 *  31           24 23           16 15           8 7             0
 * +---------------+---------------+--------------+---------------+
 * | Scan Count    | Checkpoint    | Scan Profile | RESERVED      |
 * +---------------+---------------+--------------+---------------+
 * Note: all offsets are relative to the 32-bit word boundaries
 *
 * RAY HEADER BIT LAYOUT:
 *  31  30      24 23      20 19                          0
 * +---+----------+----------+-----------------------------+
 * | 1 | SSI      | RESERVED | Timestamp                   |
 * +---+----------+----------+-----------------------------+
 *
 *  31                       16 15                        0
 * +---------------------------+---------------------------+
 * | Azimuth Angle             | Elevation Angle           |
 * +---------------------------+---------------------------+
 * Note: all offsets are relative to the 32-bit word boundaries
 *
 * RAY RETURN BIT LAYOUT:
 *  31  30                       12 11                    0
 * +---+---------------------------+-----------------------+
 * | 0 | Range N                   |         Reflectance N |
 * +---+---------------------------+-----------------------+
 * Note: all offsets are relative to the 32-bit word boundaries
 *
 */

#ifndef LUM_NET_LIDAR_DATA_H
#define LUM_NET_LIDAR_DATA_H

/*********************************/
/* Includes                      */
/*********************************/
/* Includes that anyone who includes this header needs                        */
//@{

#ifdef __cplusplus
#include <cstddef>
#else
#include <stddef.h>
#endif

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <LumNet/Common.h>
#include <LumNet/Common/ShiftAndMask.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
/* Naming Examples:                                                           */
/* - #define EXAMPLE_TEMPLATE_VERSION_NUMBER (1)                              */
/* - #define EXAMPLE_TEMPLATE_FORMAT(a)      (a)                              */
//@{

////////////////////////////////////////////////////////////////////////////////
// CONSTANTS

/**
 * @brief Semantic Major Version of the LumNet Lidar Data functionality.
 */
#define LUM_NET_LIDAR_DATA_PROTOCOL_SEMVER_MAJOR (2)

/**
 * @brief Semantic Minor Version of the LumNet Lidar Data functionality.
 */
#define LUM_NET_LIDAR_DATA_PROTOCOL_SEMVER_MINOR (1)

/**
 * @brief Semantic Patch Version of the LumNet Lidar Data functionality.
 */
#define LUM_NET_LIDAR_DATA_PROTOCOL_SEMVER_PATCH (0)

/**
 * @brief Max number of bytes contained in a Lidar Data packet
 */
#define LUM_NET_MAX_LIDAR_DATA_PAYLOAD_SIZE (1000)

/*
 * PI is not guaranteed to be defined on all systems, so it is defined here
 */
#define LUM_NET_PI (3.14159265359f)

/**
 * @brief Number of bits in the angle fields
 */
#define LUM_NET_FIXED_POINT_ANGLE_NUM_BITS (16)

/**
 * @brief Number of fractional bits in the range fields
 */
#define LUM_NET_FIXED_POINT_RANGE_NUM_FRACTIONAL_BITS (10)

/**
 * @brief Number of fractional bits in the reflectance field
 */
#define LUM_NET_FIXED_POINT_REFLECTANCE_NUM_FRACTIONAL_BITS (11)
//@}

#define LUM_NET_FIXED_POINT_ANGLE_SCALE_UNSIGNED                               \
  (65536.f) // 2 ^ LUM_NET_FIXED_POINT_ANGLE_NUM_BITS

#define LUM_NET_FIXED_POINT_ANGLE_SCALE_SIGNED                                 \
  (32768.f) // 2 ^ (LUM_NET_FIXED_POINT_ANGLE_NUM_BITS - 1)

#define LUM_NET_FIXED_POINT_RANGE_SCALE                                        \
  (1024.f) // 2 ^ LUM_NET_FIXED_POINT_RANGE_NUM_FRACTIONAL_BITS

#define LUM_NET_FIXED_POINT_REFLECTANCE_SCALE                                  \
  (2048.f) // 2 ^ LUM_NET_FIXED_POINT_REFLECTANCE_NUM_FRACTIONAL_BITS

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
/* Don't typedef structs/unions                                               */
/* Naming Examples:                                                           */
/* - union   Template_parameterOverlay {...};                                 */
/* - typedef unsigned int ExampleTemplate_columnIndex;                        */
/* - enum Template_itemType {TEMPLATE_ITEM_TYPE_A, TEMPLATE_ITEM_TYPE_B};     */
//@{

LUM_NET_PACK(struct LumNet_LidarDataPreamble {
  struct LumNet_SemVersionedPayloadHeader semVersionedHeader;

  uint8_t  num_rays; // Number of rays contained in the data of this packet
  uint8_t  sequence_number; // Sequence in lidar data packets from this head
  uint16_t fingerprint;     // Fingerprint of the head sending this packet

  uint32_t timestamp_seconds; // 32-bit Unix timestamp

  uint8_t _RESERVED;
  uint8_t scan_profile;
  uint8_t checkpoint;
  uint8_t scan_counters;
});

typedef uint64_t LumNet_RayHeader;

typedef uint32_t LumNet_RayReturn;

LUM_NET_PACK(struct LumNet_Ray {
  LumNet_RayHeader rayHeader;
  LumNet_RayReturn rayReturn[1]; // Flexible Array Member, but to preserve
                                 // compatibility with C++ uses dummy length of
                                 // 1
});

//@}

/*********************************/
/* GLOBAL Variable Declarations  */
/*********************************/
/* Declare as extern                                                          */
/* Naming Examples:                                                           */
/* - extern int ExampleTemplate_variableName;                                 */
//@{

//@}

/*********************************/
/* GLOBAL Function Declarations  */
/*********************************/
/* Don't declare as extern                                                    */
/* Naming Examples:                                                           */
/* - void ExampleTemplate_Init( void );                                       */
/* - void Template_ReturnZero( void );                                        */
//@{

//@}

/*********************************/
/* GLOBAL Function Definitions   */
/*********************************/
/* Declare as inline                                                          */
/* These should only be small, performance-critical functions in the header   */
//@{

static inline struct LumNet_Version LumNet_GetLidarDataProtocolSemVer(void) {
  return LumNet_VersionBuilder(LUM_NET_LIDAR_DATA_PROTOCOL_SEMVER_MAJOR,
                               LUM_NET_LIDAR_DATA_PROTOCOL_SEMVER_MINOR,
                               LUM_NET_LIDAR_DATA_PROTOCOL_SEMVER_PATCH);
}

/**
 * @brief Conversion factor from signed azimuth/elevation bits to radians
 *
 * Reported angles can be interpreted as signed Q0.20. In this interpretation,
 * the range of the angle is [-PI, PI) (inclusive of -PI, exclusive of PI).
 *
 * The fractional component has the range [-1, 1) with each LSB having the value
 * 1/2^(LUM_NET_FIXED_POINT_ANGLE_NUM_BITS - 1), -1 to exclude the sign bit.
 */
static inline float LumNet_SignedFixedToRadians(int16_t angle) {
  return angle * LUM_NET_PI / LUM_NET_FIXED_POINT_ANGLE_SCALE_SIGNED;
}

/**
 * @brief Conversion factor from unsigned azimuth/elevation bits to radians
 *
 * Reported angles can be interpreted as unsigned Q0.21. In this interpretation,
 * the range of the angle is [0, 2*PI) (inclusive of 0, exclusive of 2*PI)
 *
 * The fractional component has the range [-1, 1) with each LSB having the value
 * 1/2^(LUM_NET_FIXED_POINT_ANGLE_NUM_BITS).
 */
static inline float LumNet_UnsignedFixedToRadians(uint16_t angle) {
  return angle * 2 * LUM_NET_PI / LUM_NET_FIXED_POINT_ANGLE_SCALE_UNSIGNED;
}

/**
 * @brief Conversion factor from signed azimuth/elevation bits to radians
 *
 * Reported angles can be interpreted as signed Q0.20. In this interpretation,
 * the range of the angle is [-180, 180) (inclusive of -180, exclusive of 180)
 *
 * The fractional component has the range [-1, 1) with each LSB having the value
 * 1/2^(LUM_NET_FIXED_POINT_ANGLE_NUM_BITS - 1), -1 to exclude the sign bit.
 */
static inline float LumNet_SignedFixedToDegrees(int16_t angle) {
  return angle * 180 / LUM_NET_FIXED_POINT_ANGLE_SCALE_SIGNED;
}

/**
 * @brief Conversion factor from unsigned azimuth/elevation bits to radians
 *
 * Reported angles can be interpreted as unsigned Q0.21. In this interpretation,
 * the range of the angle is [0, 360) (inclusive of 0, exclusive of 360).
 *
 * The fractional component has the range [-1, 1) with each LSB having the value
 * 1/2^(LUM_NET_FIXED_POINT_ANGLE_NUM_BITS).
 */
static inline float LumNet_UnsignedFixedToDegrees(uint16_t angle) {
  return angle * 360 / LUM_NET_FIXED_POINT_ANGLE_SCALE_UNSIGNED;
}

/**
 * @brief Convert fixed point range representation to float
 *
 * Range is reported as an unsigned Q9.10 (9 bits before the decimal point, 10
 * after).
 */
static inline float LumNet_RangeFixedToFloat(uint32_t range) {
  return ((float) range) / LUM_NET_FIXED_POINT_RANGE_SCALE;
}

/**
 * @brief Convert fixed point reflectance representation to float
 *
 * Reflectance is reported as an unsigned Q1.11 to provide range of [0-2].
 */
static inline float LumNet_ReflectanceFixedToFloat(uint32_t reflectance) {
  return ((float) reflectance) / LUM_NET_FIXED_POINT_REFLECTANCE_SCALE;
}

/**
 * @brief  Get Scan Count
 * @param  Valid Scan Counters field
 * @return Scan count (top 4 bits)
 */
static inline uint8_t
LumNet_RayHeader_GetScanCount(const uint8_t scanCounters) {
  return scanCounters >> 4;
}

/**
 * @brief  Get Interlace Index
 * @param  Valid Scan Counters field
 * @return Interlace Index (bottom 4 bits)
 */
static inline uint8_t
LumNet_RayHeader_GetInterlaceIndex(const uint8_t scanCounters) {
  return scanCounters & 0xF;
}

/**
 * @brief  Get Track Number
 * @param  Valid checkpoint field
 * @return Track Number
 */
static inline uint8_t
LumNet_RayHeader_GetTrackNumber(const uint8_t checkpoint) {
  return checkpoint >> 4;
}

/**
 * @brief  Get Track Checkpoint
 * @param  Valid checkpoint field
 * @return Track Checkpoint (bottom 4 bits)
 */
static inline uint8_t
LumNet_RayHeader_GetTrackCheckpoint(const uint8_t checkpoint) {
  return checkpoint & 0xF;
}

/**
 * @brief  Get the MSB of the first 32-bit word in the header
 * @param  Valid Ray Header
 * @return MSB of the first 32-bit word in the header
 */
static inline uint8_t
LumNet_RayHeader_GetRayHeaderBit(const LumNet_RayHeader *const header) {
  const uint32_t *const headerWords = (uint32_t const *) header;

  return (uint8_t) LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(31, 1,
                                                           headerWords[0]);
}

/**
 * @brief  Get SSI of a Ray
 * @param  Valid Ray Header
 * @return SSI including eye index of the given Ray
 */
static inline uint32_t
LumNet_RayHeader_GetSSI(const LumNet_RayHeader *const header) {
  uint32_t const *const headerWords = (uint32_t const *) header;

  return LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(24, 7, headerWords[0]);
}

/**
 * @brief  Get eye index of a Ray
 * @param  header Valid Ray Header
 * @param  headNumEyes Number of bits used to represent the eye
 * @return What eye sampled the given ray
 */
static inline uint32_t
LumNet_RayHeader_GetEyeIndex(const LumNet_RayHeader *const header,
                             uint8_t                       headNumEyes) {
  uint32_t const *const headerWords = (uint32_t const *) header;

  return LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(24, headNumEyes,
                                                 headerWords[0]);
}

/**
 * @brief  Get microsecond timestamp of a Ray
 * @param  Valid Ray Header
 * @return Microsecond timestamp of the given Ray
 */
static inline uint32_t
LumNet_RayHeader_GetMicroseconds(const LumNet_RayHeader *const header) {
  uint32_t const *const headerWords = (uint32_t const *) header;

  return LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(0, 20, headerWords[0]);
}

/**
 * @brief  Get azimuth angle in signed form of a Ray
 * @param  Valid Ray Header
 * @return Azimuth angle of given Ray
 */
static inline int16_t
LumNet_RayHeader_GetAngleAzimuth(const LumNet_RayHeader *const header) {
  uint32_t const *const headerWords = (uint32_t const *) header;

  return (int16_t) LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(16, 16,
                                                           headerWords[1]);
}

/**
 * @brief  Get elevation angle in signed form of a Ray
 * @param  Valid Ray Header
 * @return Elevation angle of given Ray
 */
static inline int16_t
LumNet_RayHeader_GetAngleElevation(const LumNet_RayHeader *const header) {
  uint32_t const *const headerWords = (uint32_t const *) header;

  return (int16_t) LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(0, 16,
                                                           headerWords[1]);
}

/**
 * @brief  Get the MSB of the 32-bit Ray Return word
 * @param  Valid Ray Return
 * @return MSB of the 32-bit Ray Return word
 */
static inline uint8_t
LumNet_RayReturn_GetReturnBit(const LumNet_RayReturn *const returnWord) {

  return (uint8_t) LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(31, 1, *returnWord);
}

/**
 * @brief  Get fixed point representation of return's range
 *
 * NOTE: can be converted to floating point using the conversion provided in
 * LidarData.h
 *
 * @param  Valid Ray Return
 * @return Range of given Ray Return
 */
static inline uint32_t
LumNet_RayReturn_GetRange(const LumNet_RayReturn *const returnWord) {

  return (uint32_t) LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(12, 19,
                                                            *returnWord);
}

/**
 * @brief  Get raw representation of return's reflectance
 * @param  Valid Ray Return
 * @return Reflectance of given Ray Return
 */
static inline uint32_t
LumNet_RayReturn_GetReflectance(const LumNet_RayReturn *const returnWord) {

  return (uint32_t) LUM_NET_SHIFT_AND_MASK_BIT_RANGE_GETTER(0, 12, *returnWord);
}

  //@}

#endif /* LUM_NET_LIDAR_DATA_H */
