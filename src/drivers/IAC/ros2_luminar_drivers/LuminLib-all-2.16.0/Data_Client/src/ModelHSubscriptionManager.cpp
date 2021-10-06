/*
* ModelHSubscriptionManager.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ModelHSubscriptionManager.h"

#include "ModelHDistributor.h"

using namespace lum;
using namespace std;

ModelHSubscriptionManager::ModelHSubscriptionManager()
    : mEmitPointData( false )
{
}

bool ModelHSubscriptionManager::AddSubscriber( IModelHSubscriber* newSubscriber )
{
    if ( newSubscriber == nullptr )
    {
        return false;
    }
    const lock_guard<mutex> lock( mSubscriberMutex );
    mSubscribers.insert( newSubscriber );

    return true;
}

bool ModelHSubscriptionManager::RemoveSubscriber( IModelHSubscriber* currentSubscriber )
{
    if ( currentSubscriber == nullptr )
    {
        return false;
    }
    const lock_guard<mutex> lock( mSubscriberMutex );
    const auto found = mSubscribers.find( currentSubscriber );
    if ( found != mSubscribers.end() )
    {
        mSubscribers.erase( found );
        return true;
    }
    return false;
}

void ModelHSubscriptionManager::PointsReady( SensorUID sensor, const vector<LidarReturn>& points, ReceiveContext receiveContext )
{
    const lock_guard<mutex> subscriberLock( mSubscriberMutex );
    for ( const auto& e : mSubscribers )
    {
        if ( !e->mHSubscriptionPaused )
        {
            e->ReceiveModelHPoints( sensor, points, receiveContext );
        }
    }
}

void ModelHSubscriptionManager::StartData()
{
    mEmitPointData = true;
    mPublisherThread = std::thread( &ModelHSubscriptionManager::SendData, this );
}

void ModelHSubscriptionManager::StopData()
{
    mEmitPointData = false;
    if ( mPublisherThread.joinable() )
    {
        mPublisherThread.join();
    }
}

void ModelHSubscriptionManager::SendData()
{
    while ( mEmitPointData )
    {
        while ( ModelHDistributor::get().ProcessPacketData() )
        {
        }
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ) );
    }
}
