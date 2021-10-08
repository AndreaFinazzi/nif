/*
* ControlClientHelper.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief ControlClientHelper
*         The ControlClientHelper illustrates how to use the ModelHController and how to power on, control,
*         and change scan patterns for multiple connected sensors.  This sample class will exercise
*         the control client methods within LuminLib.  It has a thread constantly checking for
*         DiscoveryResponses, the network packet required to discover and communicate with Sensors.
*         To use, begin by calling InitializeModelH, call DiscoverSensors to find connected sensors,
*         Set the index of the sensor you'd like to control, and then control the selected sensor
*         by calling some of the Set commands. To finish, call StopModelH to cleanly exit.
*/

#pragma once

#include "SensorConnectionSubscriber.h"

#include <DiscoveryResponse.h>

#include <thread>

#include "CommandUtilities.h"
#include "ModelHCommandClientSubscriber.h"
#include "ModelHController.h"
#include "ModelHDistributor.h"

namespace lum
{
namespace sample
{
enum SettingDuration
{
    VOLATILE = 0,
    PERSISTENT
};

class ControlClientHelper : public SensorConnectionSubscriber, public ModelHCommandClientSubscriber
{
public:
    ControlClientHelper() { ; }
    virtual ~ControlClientHelper();
    // Exercise the ModelH Startup logic, start our discovery and response threads
    void InitializeModelH();
    // Stop the local discovery and response threads, and shut down the response server.
    void StopModelH();
    // Discover connected sensor heads
    void DiscoverSensors();
    // Return the number of discovered sensor heads
    size_t GetDiscoveredSensorCount();
    // Return the UID for the given sensor index
    SensorUID GetSensorID();
    // Set the index of the discovered sensor we're communicating with
    void SetSensorIndex( uint8_t sensorIndex );
    //Send the current value of the LumNet_ScanProfile to the selected sensor
    void SendScanPattern( LumNet_GenericScanProfile scanProfile );
    // When we send a command, do expect it to persist through shutdown, or is it volatile?
    void SetDuration( const SettingDuration& duration );
    // Set the field of view for the indexed sensor
    void SetFOV( float fov, float fovCenter );
    // Set the frequency in Hz
    bool SetFrequency( float frequency );
    // set (and send) the scan pattern
    bool SetScanPattern(LumNet_ScanProfileTypes pattern, float a, float b, float c, float d, float e);

    virtual void TransactionStatus( uint16_t fingerprint,
                            uint16_t transactionID,
                            CSCTransactionStatus transactionStatus,
                            const uint8_t* response ) override;

    virtual void GenericScanProfileUpdated( uint16_t fingerprint,
                                    uint16_t transactionID,
                                    struct LumNet_GenericScanProfile scan_profile ) override final;

    virtual void InterlockStatesUpdated(uint16_t fingerprint,  // called when the returned data specifically for Interlock States  returned succeeds
                                    uint16_t transactionID,
                                    struct LumNet_Interlocks interlock_states ) override final;

    void ReadInterlockState();

    // find the index of my sensor's fingerprint
    uint8_t FindIndexOfSensorFingerprint(uint16_t my_fingerprint);
    void AddPcapDataListener(uint16_t my_pcap_port);


private:
    // Test if current profile is valid
    static bool ValidScanProfile( const LumNet_GenericScanProfile profile );
    //Internal blocking thread call that collects the responses.
    void CheckForResponses();
    //Send the current value of the LumNet_ScanProfile to the selected sensor
    void SendRequest();

    // Callback method that is executed when a sensor connection operation completes successfully.
    virtual void SensorConnected( uint16_t fingerprint ) override final;

    std::thread mResponseThread;

    LumNet_GenericScanProfile mScanProfile;

    std::vector<DiscoveryResponse> mDiscoveredSensors;
    uint8_t mSensorHeadIndex = 0;

    bool mDiscoverSensors = false;
    bool mCheckResponses = false;
    float mCenterDegrees = 0.f;
    float mMeanOffsetFromCenter = 1.f;
    float mSigmaDegrees = 5.f;
    SettingDuration mDuration = VOLATILE;

    float mLastCenter = 0.f;
    int mSelectIdx = 0;

};
}  // namespace sample
}  // namespace lum