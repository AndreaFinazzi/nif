/**
 * @file   Payloads.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  Top-level file to include all custom payloads or typedefs
 *         available to LumNet's command functionality.
 */

#ifndef LUM_NET_COMMAND_PAYLOADS_H
#define LUM_NET_COMMAND_PAYLOADS_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <stdint.h>
#include <LumNet/Common.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{
#define LUM_NET_SCAN_DEFAULT_MAX_VELOCITY                                      \
  (1146.0f) // degrees per second (used for gaussian and exponential)
#define LUM_NET_SCAN_MININUM_MAX_VELOCITY                                      \
  (10.0f) // degrees per second (used for gaussian and exponential)
#define LUM_NET_SCAN_MAXIMUM_MAX_VELOCITY                                      \
  (2291.0f) // degrees per second (used for gaussian and exponential)

#define LUM_NET_SCAN_MIN_SIGMA (1.0f)    //(used for gaussian and exponential)
#define LUM_NET_SCAN_MIN_EXPONENT (0.5f) //(used for exponential)
#define LUM_NET_SCAN_MAX_EXPONENT (6.0f) //(used for exponential)

#define LUM_NET_SCAN_MIN_DENSITY_MULTIPLIER                                    \
  (0.1f) //(used for horizon focus and trapezoid)
#define LUM_NET_SCAN_MAX_DENSITY_MULTIPLIER                                    \
  (10.f) //(used for horizon focus and trapezoid)
#define MAX_SCAN_PLAYLIST_LENGTH (5)
//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

/**
 * @brief An 8-bit number representing the scan pattern profile identifier.
 *        Currently only Profile identifier A is available at this time.
 */
typedef uint8_t LumNet_ScanProfileID;

/**
 * @brief A list of well named Scan Profile Identifiers available to use to
 *        assign scan profiles.
 */
enum LumNet_ScanProfileIDs {
  LUM_NET_SCAN_PROFILE_ID_A = 1,
};

enum LumNet_ScanProfileTypes {
  LUM_NET_SCAN_PROFILE_TYPE_UNIFORM       = 0,
  LUM_NET_SCAN_PROFILE_TYPE_GAUSSIAN      = 1,
  LUM_NET_SCAN_PROFILE_TYPE_HORIZON_FOCUS = 2,
  LUM_NET_SCAN_PROFILE_TYPE_TRAPEZOID     = 3,
  LUM_NET_SCAN_PROFILE_TYPE_EXPONENTIAL   = 4,
  LUM_NET_SCAN_PROFILE_TYPE_NUM_SCAN_PROFILES
};

typedef uint8_t LumNet_ScanProfileType;

enum LumNet_ScanPatternUniformFields {
  LUM_NET_SCAN_PATTERN_UNIFORM_FIELDS_COUNT = 0,
};

/**
 * @brief Constraints:
 * 1. Sigma must be >= to LUM_NET_SCAN_MIN_SIGMA
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_SIGMA_TOO_SMALL)
 * 2. Max velocity must be >= to LUM_NET_SCAN_MININUM_MAX_VELOCITY and <= to
 * LUM_NET_SCAN_MAXIMUM_MAX_VELOCITY
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_BELOW_MIN or
 * LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_ABOVE_MAX)
 * 3. Scan Frequency * Total FOV * 1.5 must be <= max velocity.
 *    For lower max velocities, consider uniform scan patterns.
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_TOO_SLOW_TO_ACHIEVE_SCAN)
 * 4. (Mean + center must be >= center - (total FOV / 2)
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MEAN_OUTSIDE_FOV)
 * 5. (Mean + center must be <= center + (total FOV / 2)
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MEAN_OUTSIDE_FOV)
 */
enum LumNet_ScanPatternGaussianFields {
  LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MEAN_OFFSET_FROM_CENTER = 0,
  LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_SIGMA,
  LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MAX_VEL,
  LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_COUNT
};

/**
 * @brief Constraints:
 * 1. Width must be positive and <= total FOV
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_WIDTH_NOT_GREATER_THAN_ZERO
 * or
 * LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_WIDTH_LARGER_THAN_FOV)
 * 2. Density must be >= LUM_NET_SCAN_MIN_DENSITY_MULTIPLIER and <=
 * LUM_NET_SCAN_MAX_DENSITY_MULTIPLIER
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_DENSITY_MULTIPLIER_OUT_OF_BOUNDS)
 * 3. Focused area must be completely contained in the FOV
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_MIN_EXCEEDS_MIN_FOV_EXTENTS
 * or
 * LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_MAX_EXCEEDS_MAX_FOV_EXTENTS)
 */
enum LumNet_ScanPatternHorizonFields {
  LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_LOCATION_OFFSET_FROM_CENTER = 0,
  LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_WIDTH,
  LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_DENSITY,
  LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_COUNT
};

/**
 * @brief Constraints:
 * 1. Base min must be <= to top min
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_SHAPE_PARAMETERS_OUT_OF_ORDER)
 * 2. Top min must be <= to top max
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_SHAPE_PARAMETERS_OUT_OF_ORDER)
 * 3. Top max must be <= to base max
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_SHAPE_PARAMETERS_OUT_OF_ORDER)
 * 4. Top density must be >= LUM_NET_SCAN_MIN_DENSITY_MULTIPLIER and <=
 * LUM_NET_SCAN_MAX_DENSITY_MULTIPLIER
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_DENSITY_MULTIPLIER_OUT_OF_BOUNDS)
 * 5. Base min and base max (after offsetting them from center) must be
 * contained in the FOV
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_PARAMETERS_OUTSIDE_FOV)
 */
enum LumNet_ScanPatternTrapezoidFields {
  LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_BASE_MIN_OFFSET_FROM_CENTER = 0,
  LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_BASE_MAX_OFFSET_FROM_CENTER,
  LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_MIN_OFFSET_FROM_CENTER,
  LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_MAX_OFFSET_FROM_CENTER,
  LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_DENSITY,
  LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_COUNT
};

/**
 * @brief Constraints:
 * 1. Sigma must be >= to LUM_NET_SCAN_MIN_SIGMA
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_SIGMA_TOO_SMALL)
 * 2. Max velocity must be >= to LUM_NET_SCAN_MININUM_MAX_VELOCITY and <= to
 * LUM_NET_SCAN_MAXIMUM_MAX_VELOCITY
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_BELOW_MIN or
 * LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_ABOVE_MAX)
 * 3. Exponent must be >= LUM_NET_SCAN_MIN_EXPONENT and <=
 * LUM_NET_SCAN_MAX_EXPONENT
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXPONENT_OUT_OF_BOUNDS)
 * 4. Scan Frequency * Total FOV * 1.5 must be <= max velocity.
 *    For lower max velocities, consider uniform scan patterns.
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_TOO_SLOW_TO_ACHIEVE_SCAN)
 * 5. (Mean + center must be >= center - (total FOV / 2)
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MEAN_OUTSIDE_FOV)
 * 6. (Mean + center must be <= center + (total FOV / 2)
 * (LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MEAN_OUTSIDE_FOV)
 */
enum LumNet_ScanPatternExponentialFields {
  LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_MEAN_OFFSET_FROM_CENTER = 0,
  LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_SIGMA,
  LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_MAX_VEL,
  LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_EXPONENT,
  LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_COUNT
};

/**
* @brief Structure representing settings relevant for defining and activating
         a scan pattern whose rate, min and max extents, and center is
configurable. Used with the
LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_UNIFORM_UNIDIRECTIONAL_SCAN_PROFILE
command.
*/
LUM_NET_PACK(struct LumNet_ScanProfileDeprecatedV4 {
  float frequencyHz;        //!< 1.0 Hz to 30.0 Hz inclusive
  float fieldOfViewDegrees; //!< currently constrained to 0 Deg to 30 Deg
                            //!< inclusive
  float centerDegrees; //!< currently constrained to -15 Deg to 15 Deg inclusive
  LumNet_ScanProfileID scanProfileID; //!< Provide an identifier for the scan
                                      //!< profile. Currently this must be set
                                      //!< to LUM_NET_SCAN_PROFILE_ID_A.
  LumNet_Bool activateProfile; //!< Whether or not scan profile should become
                               //!< the active scan pattern. Currently this must
                               //!< be set to 1.
});

/**
* @brief Structure representing settings relevant for defining and activating
         a scan pattern whose rate, min and max extents, and center is
configurable.  This is the deprecated structure to be used with the deprecated
LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_UNIFORM_SCAN_PROFILE_DEPRECATED
command.
*/
LUM_NET_PACK(struct LumNet_ScanProfileDeprecated {
  uint8_t frequencyHz;        //!< 1 Hz to 30 Hz inclusive
  float   fieldOfViewDegrees; //!< currently constrained to 0 Deg to 30 Deg
                              //!< inclusive
  float centerDegrees; //!< currently constrained to -15 Deg to 15 Deg inclusive
  LumNet_ScanProfileID scanProfileID; //!< Provide an identifier for the scan
                                      //!< profile. Currently this must be set
                                      //!< to LUM_NET_SCAN_PROFILE_ID_A.
  LumNet_Bool activateProfile; //!< Whether or not scan profile should become
                               //!< the active scan pattern. Currently this must
                               //!< be set to 1.
});

/**
* @brief Structure representing settings relevant for defining and activating
         a scan pattern with a gaussian distribution with a configurable mean
and sigma. As with uniform, the rate, min and max extents, and center is also
configurable.
*/
LUM_NET_PACK(struct LumNet_GaussianScanProfileDeprecated {
  struct LumNet_ScanProfileDeprecatedV4 commonScanProfileParams;
  float gaussianMeanOffsetFromCenterDegrees; //!< Mean of gaussian, in degrees
                                             //!< offset from center of scan
  float gaussianSigmaDegrees;                //!< Sigma of gaussian, in degrees
});

LUM_NET_PACK(struct LumNet_CommonScanProfileParams {
  float frequencyHz;        //!< 1.0 Hz to 30.0 Hz inclusive
  float fieldOfViewDegrees; //!< currently constrained to 0 Deg to 30 Deg
                            //!< inclusive
  float centerDegrees; //!< currently constrained to -15 Deg to 15 Deg inclusive
  LumNet_ScanProfileType scanProfileType;
});

LUM_NET_PACK(struct LumNet_GenericScanProfile {
  struct LumNet_CommonScanProfileParams commonScanProfileParams;
  float                                 scanProfileSpecificParams[10];
});

typedef uint8_t LumNet_InterlaceFactor;

enum LumNet_InterlaceFactors {
  LUM_NET_INTERLACE_FACTOR_ONE     = 0,
  LUM_NET_INTERLACE_FACTOR_TWO     = 1,
  LUM_NET_INTERLACE_FACTOR_FOUR    = 2,
  LUM_NET_INTERLACE_FACTOR_EIGHT   = 3,
  LUM_NET_INTERLACE_FACTOR_SIXTEEN = 4,
};

#define MIN_INTERLACE_FACTOR (LUM_NET_INTERLACE_FACTOR_ONE)
#define DEFAULT_INTERLACE_FACTOR (LUM_NET_INTERLACE_FACTOR_ONE)
#define MAX_INTERLACE_FACTOR (LUM_NET_INTERLACE_FACTOR_SIXTEEN)

LUM_NET_PACK(struct LumNet_InterlacingConfiguration {
  // Number of subscans, each with a different vertical offset, which constitute
  // a full vertical scan
  LumNet_InterlaceFactor verticalInterlaceFactor;
  // Used to define two variables:
  // 1. Number of subscans, each with a different horizontal offset, which
  // constitute a full horizontal scan
  // 2. Number of horizontal offsets applied to rays within a line. An offset
  // applies to the whole line. The first line in each Scan has an offset of 0.
  LumNet_InterlaceFactor horizontalInterlaceFactor;
});

LUM_NET_PACK(struct LumNet_ScanPlaylistTrack {
  struct LumNet_GenericScanProfile scanProfile;
  LumNet_Bool                      bottomToTop;
});

LUM_NET_PACK(struct LumNet_ScanPlaylist {
  struct LumNet_ScanPlaylistTrack scanPatterns[MAX_SCAN_PLAYLIST_LENGTH];
  uint8_t                         playlistLength;
});

/**
 * @brief If a failure occurs in a
 * LUM_NET_CMD_ADDR_OFFSET_LIDAR_SCANNER_GENERATE_AND_ASSIGN_SCAN_PLAYLIST
 * command, a payload of LumNet_ScanPlaylistFailureIndex will be sent.
 * This will contain the track of the item in the playlist that
 * triggered the failure (first failure encountered).  The reported error code
 * will be associated with that scan profile. An index of -1 is returned if the
 * failure was not scan profile specific.
 */
#define LUM_NET_SCAN_PLAYLIST_FAILURE_INDEX_NOT_PROFILE_SPECIFIC (-1)
typedef int8_t LumNet_ScanPlaylistFailureIndex;

/**
 * @brief Reported in Celsius
 */
typedef float LumNet_SystemTemperature;
/**
 * @brief Structure representing ptp time synchronization status
 */
LUM_NET_PACK(struct LumNet_PtpStatus {
  LumNet_PtpTypeSigned MasterOffset;      // in nano seconds
  LumNet_PtpType       PathDelay;         // in nano seconds
  LumNet_PtpTypeFloat  FrequencyOffset;   // in parts per billion of a second
  LumNet_PtpTypeDouble NeighborRateRatio; // gPTP specific ratio, value is less
                                          // than or equal to ~1
});

typedef uint8_t LumNet_VerticalSyncMode;

enum LumNet_VerticalSyncModes {
  LUM_NET_VERTICAL_SYNC_MODE_OFF       = 0,
  LUM_NET_VERTICAL_SYNC_MODE_SOFT_SYNC = 1,
  LUM_NET_VERTICAL_SYNC_MODE_HARD_SYNC = 2,
  LUM_NET_VERTICAL_SYNC_MODE_NUM_SYNC_MODES
};

typedef uint64_t LumNet_VerticalSyncOffsetMs;

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

#endif // LUM_NET_COMMAND_PAYLOADS_H
