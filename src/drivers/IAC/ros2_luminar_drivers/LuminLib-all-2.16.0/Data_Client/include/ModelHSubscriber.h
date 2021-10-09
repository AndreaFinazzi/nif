/*
* ModelHSubscriber.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHSubscriber
 *         Interface for receiving H LiDAR data
 *
 *  The ModelHSubscriber is an interface used to deliver LidarReturns to clients of the SDK when
 *  used in conjunction with the ModelHDistributor. Implement the ReceiveModelHPoints to get
 *  a vector of LidarReturns each frame.
 */

#pragma once

#include <vector>

#include "luminlib_export.h"

#include "LidarReturn.h"
#include "SensorUID.h"

namespace lum
{
enum CoordinateSystem
{
    Cartesian,
    Spherical
};

enum TransmissionFrequency
{
    Error = 0,
    Frame = 1,
    Line = 2,
    SSI = 4
};

struct ReceiveContext
{
    bool includesMissingData = false;
    bool frameComplete = false;
    CoordinateSystem coordinateSystem = Cartesian;
    TransmissionFrequency transmissionFq = Frame;
};

/*
 * The ModelHSubscriber is an interface used to deliver LidarReturns to clients of the SDK when
 * used in conjunction with the ModelHDistributor. Implement the ReceiveModelHPoints to get
 * a vector of LidarReturns each frame.
 */
class LUMINLIB_EXPORT IModelHSubscriber
{
public:
    //Whether this subscriber is paused and should not receive data
    bool mHSubscriptionPaused;

    /*
     * Receive data from the SDK with the requested transformations applied
     */
    virtual void ReceiveModelHPoints( SensorUID sensor, const std::vector<LidarReturn>& points, ReceiveContext receiveContext ) = 0;
};
}  // namespace lum
