/*
* ModelHDistributor.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ModelHDistributor
 *         Forwards parsed LiDAR data to subscribers
 *
 */

#pragma once

#include <assert.h>
#include <map>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <LumNet/Command/Payloads.h>

#include "LidarReturn.h"
#include "ModelHClient.h"
#include "ModelHDataPacketParser.h"
#include "ModelHSubscriptionManager.h"
#include "PagedMemoryPool.h"
#include "SensorPose.h"
#include "SensorUID.h"
#include "luminlib_export.h"

namespace lum
{
static const int EMPTY_RAYS_FILTER_INDEX = 3;
static const int MAX_NUM_RETURNS_PER_SENSOR = 2'000'000;

class LUMINLIB_EXPORT ModelHDistributor
{
public:
    // Gets the current instance of the Model H Distributor
    static ModelHDistributor& get();

    // Initialize and start a Model H Client with localhost and default port
    void InitializeModelHClient( bool initializeDefaultClient = false );

    /*
    * Create a new Model H Client on the specified port and add as listener
    * If active, stops the Model H Client on the specified port before starting a new one
    */
    void AddModelHListener( uint16_t portNumber );

    // If active, stops the Model H Client on the specified port and removes as listener
    void RemoveModelHListener( uint16_t portNumber );

    /*
    * Add the passed subscriber to the subscriber list
    * Returns true on add success
    */
    bool AddModelHDataSubscriber( IModelHSubscriber* subscriber );

    /*
    * Remove the passed subscriber from the subscriber list
    * Returns true on remove success of a valid subscriber
    */
    bool RemoveModelHDataSubscriber( IModelHSubscriber* subscriber );

    /*
    * Set which returns to filter out while processing packet data
    * The Model H Distributor filters no returns by default
    */
    void FilterReturns( bool returnsFilter[4] );

    /*
    * Set which eye to filter out while processing packet data
    * The Model H Distributor does not filter either eye by default
    */
    void ApplyEyeFilter( bool eyeFilter[2] );

    /*
    * Assign an offset, or SensorPose, to each sensor head so that the points can all be
    * relative to one fixed point
    */
    void SetSensorPose( SensorUID sensorID, SensorPose pose );

    /*
    * Sets pose to the current SensorPose for a given fingerprint
    * Returns true if the current fingerprint has a SensorPose and pose was updated
    */
    bool GetSensorPose( SensorUID sensorID, SensorPose& pose );

    /*
    * Resets all stored SensorPoses
    */
    void ResetSensorPoses();

    /*
    * Convert all data incoming through ProcessPacketData from spherical to Cartesian
    * The Model H Distributor converts to Cartesian by default
    */
    void Cartesian( bool cartesian );

    /*
    * If points are being delivered as spherical, convert all data incoming through ProcessPacketData from Radians to Degrees
    * The Model H Distributor delivers Radian spherical coordinates as the normal Cartesian(false) coordinates
    */
    void Degrees( bool returnDegrees );

    /*
    * Dictates how often ReceiveModelHPoints is called.  Currently the options are once every frame (Default),
    * for every SSI, or every line per sensor.  Also supports a frequency of Frame | Line | SSI, publishing any combination therein.
    * Check the ReceiveContext to see which result is being returned to you
    */
    void PointTransmissionFrequency( TransmissionFrequency frequencyFlag );

    // Add new packet to the received packets queue to be processed
    void PacketReceived( uint8_t* pPacketData, size_t packetSize, std::string address );

    /*
    * Turn the culling of empty rays on or off
    * The Model H Distributor culls empty rays by default
    */
    void SetCullEmptyRays( bool cull );

    /*
     * Set empty returns to the specified range. Defaults to 1 meter.
     */
    void SetEmptyRaysRange( float emptyRaysRange );

    /*
     * Get the value that empty returns are being assigned to, defaults to 1 meter
     */
    float GetEmptyRaysRange();

    // Starts the threads for the Model H Clients
    void StartTransmitting();

    // Signals the Model H Clients to stop transmitting and closes all threads
    void StopTransmitting();

    /*
    * Builds and formats the returns from the devices into LidarReturns per frame and preps
    * the points for delivery
    * Returns false when there are no packets in the queue
    */
    bool ProcessPacketData();
    /*
    * Checks mapping of checkpoint to scanprofile and returns true if the passed checkpoint
    * is a snapback checkpoint
    */
    bool ShowCheckpoint( const LumNet_ScanProfileType scanProfile, const uint8_t checkpoint );

    //helper function to set/get show snapback
    void ShowSnapback( bool show );
    bool ShowSnapback();

    void ShowFrame( bool show );
    bool ShowFrame();

    float mTimingSize = 100.0;

private:
    ModelHDistributor();
    // Prevent accidental copy/assignment of this singleton
    ModelHDistributor( const ModelHDistributor& ) = delete;
    ModelHDistributor& operator=( const ModelHDistributor& ) = delete;

    ModelHSubscriptionManager mModelHDataSubscriptions;
    std::multimap<uint16_t, std::unique_ptr<ModelHClient>> mModelHDataClients;
    std::map<SensorUID, SensorPose> mSensorPoses;
    std::map<SensorUID, uint8_t> mSensorScanCounts;
    std::map<SensorUID, std::vector<LidarReturn>> mPointClouds;
    std::mutex mQueueMutex;
    std::mutex mPoseMutex;
    std::map<SensorUID, std::map<uint32_t, std::vector<LidarReturn>>> mLinePoints;
    std::map<SensorUID, std::map<uint8_t, std::vector<LidarReturn>>> mSSIPoints;
    lum::PagedMemoryPool<ModelHDataPacketParser> mPacketPool;
    std::queue<std::pair<std::string, ModelHDataPacketParser*>> mPacketQueue;
    const uint16_t mDefaultPort = 5118;
    bool mConvertToCartesian = true;
    bool mReturnFilters[4] = {false, false, false, true};  // we don't default to showing empty rays
    bool mEyeFilters[2] = {false, false};                  // we default to showing both eyes
    bool mConvertToDegrees = false;
    float mEmptyRaysRange = 1.0f;
    TransmissionFrequency mTransmissionFq = TransmissionFrequency::Frame;

    //boolean to tell distributor whether to send the snapback through the pipeline
    bool mShowSnapback = false;

    //boolean to tell distributor whether to send the frame through the pipeline
    bool mShowFrame = true;
};
}  // namespace lum
