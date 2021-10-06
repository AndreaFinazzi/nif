/*
* SensorPose.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#pragma once

#include <stdint.h>

#include "luminlib_export.h"

namespace lum
{
// Rotation and translation defining the pose of a sensor
struct LUMINLIB_EXPORT SensorPose
{
    float xMeters = 0.0f;
    float yMeters = 0.0f;
    float zMeters = 0.0f;
    float pitchRadians = 0.0f;
    float rollRadians = 0.0f;
    float yawRadians = 0.0f;
};
}  // namespace lum
