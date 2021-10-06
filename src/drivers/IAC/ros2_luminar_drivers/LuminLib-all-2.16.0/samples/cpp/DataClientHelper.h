/*
*  DataClientHelper.h
*
*  Copyright (c) 2018, Luminar Technologies, Inc.
*
*  This material contains confidential and trade secret information of Luminar Technologies.
*  Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
*  writing by Luminar Technologies.
*/

#include <map>
#include <mutex>
#include <vector>

#include "LidarReturn.h"
#include "ModelHSubscriber.h"

#pragma once

namespace lum
{
namespace sample
{
/*! \brief DataClientHelper
 *         The DataClientHelper illustrates how to use the ModelHDistributor and how to
 *         subscribe to the stream of ModelH LidarReturns.
 */
class DataClientHelper : public lum::IModelHSubscriber
{
public:
    DataClientHelper();
    ~DataClientHelper();

    // Only try to record the lidar returns from the selected fingerprint
    void SetSensorID( SensorUID sensor );
    // This callback receives a fresh vector of LidarReturns each frame, which is then moved into a local vector
    virtual void ReceiveModelHPoints( SensorUID sensor,
                                      const std::vector<LidarReturn>& points,
                                      ReceiveContext receiveContext ) override final;
    // Display the first 10 elements of the previous frame's point cloud
    void DisplayReturns();
    // Clean up the worker threads to exit gracefully
    void Shutdown();

private:
    std::vector<LidarReturn> mReturns;
    SensorUID mSensorID;
    std::mutex mPointCloudMutex;
};
}  // namespace sample
}  // namespace lum
