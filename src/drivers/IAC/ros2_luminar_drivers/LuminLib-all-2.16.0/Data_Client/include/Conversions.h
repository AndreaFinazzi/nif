/*
* Conversions.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief Conversions
 *         Conversions between spherical and Cartesian coordinates
 *
 */

#pragma once

#include "luminlib_export.h"

#include "LidarReturn.h"
#include "SensorPose.h"

#include "ModelHCommon.h"

#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>

namespace lum
{
#define ONE_EIGHTY_OVER_PI 57.2957795131f
#define PI_OVER_ONE_EIGHTY .01745329251f

#define HALF_EYE_DISTANCE 0.0255f

/*
    * Converts a point from spherical to cartesian coordinates.
    * Implements this: http://mathworld.wolfram.com/SphericalCoordinates.html
    *
    * In keeping with the common simulation conventions, the SDK emits points in a
    * y-up, left handed coordinate system.
    */
LUM_FORCE_INLINE void SphericalToCartesian( LidarReturn& point )
{
    const float azimuth_radians = point.azimuth;
    const float elevation_radians = point.elevation;
    const float range = point.range;

    point.x = range * std::sin( azimuth_radians ) * std::cos( elevation_radians );
    point.y = range * std::sin( elevation_radians );
    point.z = range * std::cos( azimuth_radians ) * std::cos( elevation_radians );
}

/*
    * Converts a point from cartesian to spherical coordinates.
    * Implements this: http://mathworld.wolfram.com/CartesianCoordinates.html
    *
    * It is assumed that the input LiDAR point is in a y-up, left handed coordinate system. The
    * resulting azimuth is the clockwise angle in the x-z plane measured in radians from the positive
    * z-axis (north). The resulting elevation is the upward elevation angle from the x-z plane measured
    * in radians.
    */
LUM_FORCE_INLINE void CartesianToSpherical( LidarReturn& point )
{
    const float x = point.x;
    const float y = point.y;
    const float z = point.z;

    point.azimuth = std::atan( x / z );
    point.range = std::sqrt( x * x + y * y + z * z );
    point.elevation = asin( y / point.range );
}

/*
    * Applies parallax correction to the input Cartesian point
    * Nominal distance between the eyes is 6.2 cm, so offset each eye by half of that
    */
LUM_FORCE_INLINE void ApplyParallaxCorrection( LidarReturn& cartesianPoint )
{
    if ( cartesianPoint.scan_segment_index % 2 )
    {
        cartesianPoint.x += HALF_EYE_DISTANCE;
    }
    else
    {
        cartesianPoint.x -= HALF_EYE_DISTANCE;
    }
}

/*
    * Reverses the parallax correction on the input Cartesian point
    * Apply the inverse of the offset applied to each eye, i.e. half the pupillary distance
    */
LUM_FORCE_INLINE void ReverseParallaxCorrection( LidarReturn& cartesianPoint )
{
    if ( cartesianPoint.scan_segment_index % 2 )
    {
        cartesianPoint.x -= HALF_EYE_DISTANCE;
    }
    else
    {
        cartesianPoint.x += HALF_EYE_DISTANCE;
    }
}

/*
    * Convert the sensor pose to the equivalent transformation matrix.
    */
LUM_FORCE_INLINE const glm::mat4x4 PoseToMatrixTransform( const SensorPose& pose )
{
    glm::mat4x4 transform =
        glm::translate( glm::mat4x4( 1.0f ), glm::vec3( pose.xMeters, pose.yMeters, pose.zMeters ) );

    transform = rotate( transform, pose.yawRadians, glm::vec3( 0.0f, 1.0f, 0.0f ) );
    transform = rotate( transform, pose.pitchRadians, glm::vec3( 1.0f, 0.0f, 0.0f ) );
    transform = rotate( transform, pose.rollRadians, glm::vec3( 0.0f, 0.0f, 1.0f ) );

    return transform;
}

/*
    * Transform a Cartesian LIDAR point with the supplied pose
    */
template <typename Point>
LUM_FORCE_INLINE void ApplyPose( Point& point, const SensorPose& pose )
{
    const glm::vec4 position = glm::vec4( point.x, point.y, point.z, 1.0f );
    const glm::mat4x4 offsetMatrix = PoseToMatrixTransform( pose );
    const glm::vec4 finalPosition = offsetMatrix * position;

    point.x = finalPosition.x;
    point.y = finalPosition.y;
    point.z = finalPosition.z;
}

/*
    * Converts a "sensor-posed" Cartesian LIDAR point back its "non-posed" position/orientation.
    */
template <typename Point>
LUM_FORCE_INLINE void ReversePose( Point& point, const SensorPose& pose )
{
    const glm::vec4 position = glm::vec4( point.x, point.y, point.z, 1.0f );
    const glm::mat4x4 inverseMatrix = glm::inverse( PoseToMatrixTransform( pose ) );
    const glm::vec4 finalPosition = inverseMatrix * position;

    point.x = finalPosition.x;
    point.y = finalPosition.y;
    point.z = finalPosition.z;
}

/*
    * Converts a spherical point from radians to degrees.
    * Implements this: http://mathworld.wolfram.com/SphericalCoordinates.html
    */
LUM_FORCE_INLINE void RadiansToDegrees( LidarReturn& point )
{
    point.azimuth = point.azimuth * ONE_EIGHTY_OVER_PI;
    point.elevation = point.elevation * ONE_EIGHTY_OVER_PI;
}

/*
    * Implements this: http://mathworld.wolfram.com/SphericalCoordinates.html
    * Converts a spherical point from degrees to radians
    */
LUM_FORCE_INLINE void DegreesToRadians( LidarReturn& point )
{
    point.azimuth = point.azimuth * PI_OVER_ONE_EIGHTY;
    point.elevation = point.elevation * PI_OVER_ONE_EIGHTY;
}
}  // namespace lum
