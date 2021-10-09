#include "CommandUtilities.h"

#include "assert.h"
#include "malloc.h"
#include "math.h"

enum Common_returnCode Command_WrapCommand( struct LumNet_CommandRequest* request,
                                            uint16_t listTransactionID,
                                            uint8_t** request_list_buffer,
                                            enum LegacyCommandProtocolSemVer version )
{
    struct LumNet_CommandRequestListPayload* request_list = Command_CreateCommandRequestList( request,
                                                                                              1,
                                                                                              listTransactionID,
                                                                                              version );

    assert( request_list );

    size_t encoded_list_size = 0;
    enum Common_returnCode encode_status = Command_CopyCommandRequestList( request_list,
                                                                           request_list_buffer,
                                                                           &encoded_list_size );

    assert( *request_list_buffer != NULL );
    assert( encode_status == COMMAND_RETURN_CODE_SUCCESS );

    free( request_list );

    return encode_status;
}

enum LUM_NET_CMD_RETURN_RESULT Command_ValidateScanProfile( struct LumNet_GenericScanProfile profile )
{
    enum LUM_NET_CMD_RETURN_RESULT status = LUM_NET_CMD_RETURN_RESULT_SUCCESS;

    if ( profile.commonScanProfileParams.frequencyHz < 0.1f || profile.commonScanProfileParams.frequencyHz > 30.0f )
    {
        return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_INVALID_FREQUENCY;
    }

    const float fovCenter = fabsf( profile.commonScanProfileParams.centerDegrees );
    const float sum = fabsf( profile.commonScanProfileParams.fieldOfViewDegrees ) + fovCenter;
    const float fovTotal = profile.commonScanProfileParams.fieldOfViewDegrees;

    /*
    * There are the following constraints on fovTotal and fovCenter:
    *
    * 1. The sum of their absolute values must not exceed 30
    * 2. fovCenter must lie between (-15, 15)
    * 3. fovTotal must lie between (0, 30)
    * 4. If fovTotal > 15, fovCenter must lie between (-0.5 * fovTotal, 0.5 fovTotal)
    *
    * If any of these fail to hold, it is not a valid assignment of fovTotal and fovCenter
    *
    */

    if ( fovTotal < 0 )
    {
        return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_NEGATIVE_FOV;
    }

    if ( sum > 30.0f || fovCenter > 15.0f )
    {
        return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXCEEDS_MAX_EXTENTS;
    }

    if ( fovCenter < -15.0f )
    {
        return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXCEEDS_MIN_EXTENTS;
    }

    if ( fovTotal < 0.0f || fovTotal > 30.0f )
    {
        return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_INVALID_FREQUENCY;
    }

    const float half_total = 0.5f * fovTotal;

    if ( fabs( fovCenter + half_total ) > 15.f )
    {
        return ( fovTotal > 0 ? LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXCEEDS_MAX_EXTENTS : LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXCEEDS_MIN_EXTENTS );
    }

    switch ( profile.commonScanProfileParams.scanProfileType )
    {
    case LUM_NET_SCAN_PROFILE_TYPE_UNIFORM:
    {
        // Everything has already been considered above
        break;
    }
    case LUM_NET_SCAN_PROFILE_TYPE_GAUSSIAN:
    {
        float half_total_fov = 0.5f * profile.commonScanProfileParams.fieldOfViewDegrees;
        float mean = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MEAN_OFFSET_FROM_CENTER];
        float mean_plus_center = mean + profile.commonScanProfileParams.centerDegrees;
        float max_vel = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_MAX_VEL];

        if ( max_vel < LUM_NET_SCAN_MININUM_MAX_VELOCITY )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_BELOW_MIN;
            break;
        }

        if ( max_vel > LUM_NET_SCAN_MAXIMUM_MAX_VELOCITY )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_ABOVE_MAX;
            break;
        }

        if ( profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_GAUSSIAN_FIELDS_SIGMA] < LUM_NET_SCAN_MIN_SIGMA )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_SIGMA_TOO_SMALL;
            break;
        }

        if ( profile.commonScanProfileParams.frequencyHz * profile.commonScanProfileParams.fieldOfViewDegrees * 1.5 > max_vel )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_TOO_SLOW_TO_ACHIEVE_SCAN;
            break;
        }

        if ( mean_plus_center < ( profile.commonScanProfileParams.centerDegrees - half_total_fov ) ||
             mean_plus_center > ( profile.commonScanProfileParams.centerDegrees + half_total_fov ) )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MEAN_OUTSIDE_FOV;
            break;
        }

        break;
    }
    case LUM_NET_SCAN_PROFILE_TYPE_TRAPEZOID:
    {
        const float fov_optical_deg = profile.commonScanProfileParams.fieldOfViewDegrees;
        const float fov_center_optical_deg = profile.commonScanProfileParams.centerDegrees;
        const float half_total_fov = 0.5f * profile.commonScanProfileParams.fieldOfViewDegrees;
        const float base_min = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_BASE_MIN_OFFSET_FROM_CENTER];
        const float base_max = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_BASE_MAX_OFFSET_FROM_CENTER];
        const float top_min = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_MIN_OFFSET_FROM_CENTER];
        const float top_max = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_MAX_OFFSET_FROM_CENTER];
        const float top_density_ratio = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_TRAPEZOID_FIELDS_TOP_DENSITY];

        if ( top_density_ratio < LUM_NET_SCAN_MIN_DENSITY_MULTIPLIER || top_density_ratio > LUM_NET_SCAN_MAX_DENSITY_MULTIPLIER )
        {
            return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_DENSITY_MULTIPLIER_OUT_OF_BOUNDS;
        }

        if ( base_min > top_min || top_min > top_max || top_max > base_max )
        {
            return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_SHAPE_PARAMETERS_OUT_OF_ORDER;
        }

        const float fov_optical_min_deg = fov_center_optical_deg - half_total_fov;
        const float fov_optical_max_deg = fov_center_optical_deg + half_total_fov;
        const float base_min_optical_deg = base_min + fov_center_optical_deg;
        const float base_max_optical_deg = base_max + fov_center_optical_deg;
        if ( base_min_optical_deg < fov_optical_min_deg || fov_optical_max_deg < base_max_optical_deg )
        {
            return LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_TRAPEZOID_PARAMETERS_OUTSIDE_FOV;
        }
        break;
    }
    case LUM_NET_SCAN_PROFILE_TYPE_HORIZON_FOCUS:
    {
        float half_total_fov = 0.5f * profile.commonScanProfileParams.fieldOfViewDegrees;
        float location = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_LOCATION_OFFSET_FROM_CENTER];
        float width = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_WIDTH];
        float density = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_HORIZON_FIELDS_DENSITY];
        float half_width = 0.5f * width;

        int width_valid = width <= profile.commonScanProfileParams.fieldOfViewDegrees;
        if ( density < LUM_NET_SCAN_MIN_DENSITY_MULTIPLIER &&
             density > LUM_NET_SCAN_MAX_DENSITY_MULTIPLIER )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_DENSITY_MULTIPLIER_OUT_OF_BOUNDS;
            break;
        }

        if ( width < 0 )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_WIDTH_NOT_GREATER_THAN_ZERO;
            break;
        }

        if ( width > profile.commonScanProfileParams.fieldOfViewDegrees )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_WIDTH_LARGER_THAN_FOV;
            break;
        }

        if ( ( location - half_width ) < ( profile.commonScanProfileParams.centerDegrees - half_total_fov ) )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_MIN_EXCEEDS_MIN_FOV_EXTENTS;
            break;
        }
        if ( ( location + half_width ) > ( profile.commonScanProfileParams.centerDegrees + half_total_fov ) )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_HORIZON_MAX_EXCEEDS_MAX_FOV_EXTENTS;
            break;
        }

        break;
    }
    case LUM_NET_SCAN_PROFILE_TYPE_EXPONENTIAL:
    {
        float half_total_fov = 0.5f * profile.commonScanProfileParams.fieldOfViewDegrees;
        float mean = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_MEAN_OFFSET_FROM_CENTER];
        float mean_plus_center = mean + profile.commonScanProfileParams.centerDegrees;
        float max_vel = profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_MAX_VEL];

        if ( max_vel < LUM_NET_SCAN_MININUM_MAX_VELOCITY )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_BELOW_MIN;
            break;
        }
        if ( max_vel > LUM_NET_SCAN_MAXIMUM_MAX_VELOCITY )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_ABOVE_MAX;
            break;
        }
        if ( profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_SIGMA] < LUM_NET_SCAN_MIN_SIGMA )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_SIGMA_TOO_SMALL;
            break;
        }

        if ( profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_EXPONENT] < LUM_NET_SCAN_MIN_EXPONENT ||
             profile.scanProfileSpecificParams[LUM_NET_SCAN_PATTERN_EXPONENTIAL_FIELDS_EXPONENT] > LUM_NET_SCAN_MAX_EXPONENT )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_EXPONENT_OUT_OF_BOUNDS;
            break;
        }

        if ( profile.commonScanProfileParams.frequencyHz * profile.commonScanProfileParams.fieldOfViewDegrees * 1.5 > max_vel )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MAX_VELOCITY_TOO_SLOW_TO_ACHIEVE_SCAN;
            break;
        }

        if ( mean_plus_center < ( profile.commonScanProfileParams.centerDegrees - half_total_fov ) ||
             mean_plus_center > ( profile.commonScanProfileParams.centerDegrees + half_total_fov ) )
        {
            status = LUM_NET_CMD_RETURN_RESULT_INVALID_SCAN_PATTERN_MEAN_OUTSIDE_FOV;
            break;
        }

        break;
    }
    }

    return status;
}
