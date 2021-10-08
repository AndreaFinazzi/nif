/*
* ModelHSubscriptionManager.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHSubscriptionManager
 *         Manages subscribers to H LiDAR data
 *
 */

#pragma once

#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "luminlib_export.h"

#include "LidarReturn.h"
#include "ModelHSubscriber.h"

namespace lum
{
class LUMINLIB_EXPORT ModelHSubscriptionManager
{
public:
    ModelHSubscriptionManager();

    // Adds the passed subscriber to the subscribers list
    bool AddSubscriber( IModelHSubscriber* newSubscriber );

    // Removes the passed subscriber from the subscribers list
    bool RemoveSubscriber( IModelHSubscriber* currentSubscriber );

    // Send the parsed Model H point data to every subscriber that isn't currently paused
    void PointsReady( SensorUID sensor, const std::vector<LidarReturn>& points, ReceiveContext receiveContext );

    // Create a thread to send points to the subscriber list via SendData
    void StartData();

    // End emitting point data and close the thread created in StartData
    void StopData();

private:
    // Sends Model H points to the subscriber list
    void SendData();

    std::thread mPublisherThread;
    bool mEmitPointData;
    std::mutex mSubscriberMutex;
    std::set<IModelHSubscriber*> mSubscribers;
};
}  // namespace lum
