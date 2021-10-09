/*
* ModelHReturnIterator.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "ModelHReturnIterator.h"

#include "BitOperations.h"

ModelHReturnIterator::ModelHReturnIterator( uint8_t* ptr, size_t pktLen )
    : mpRayDataStart( ptr )
    , mpRayDataEnd( ptr + pktLen )
    , mpRayPtr( 0 )
    , mpReturnPtr( 0 )
    , mRayIndex( 0 )
{
    Reset();
}

bool ModelHReturnIterator::End() const
{
    return mpReturnPtr >= mpRayDataEnd;
}

void ModelHReturnIterator::NextReturn()
{
    if ( IsRay() )
    {
        mpRayPtr = mpReturnPtr;
        LumNet_Ray* ray = GetCurrentRay();
        mpReturnPtr = reinterpret_cast<uint8_t*>( &ray->rayReturn );
        mRayIndex++;
    }
    else
    {
        mpReturnPtr += sizeof( LumNet_RayReturn );
    }
}

int ModelHReturnIterator::RayIndex() const
{
    return mRayIndex;
}

void ModelHReturnIterator::Reset()
{
    mRayIndex = 0;
    mpRayPtr = mpRayDataStart;
    mpReturnPtr = mpRayPtr + sizeof( LumNet_RayHeader );
}

bool ModelHReturnIterator::ReturnValid() const
{
    return LumNet_RayReturn_GetReturnBit( GetCurrentReturn() ) == 0;
}

float ModelHReturnIterator::Azimuth() const
{
    int16_t azimuthFixed = LumNet_RayHeader_GetAngleAzimuth( &GetCurrentRay()->rayHeader );
    return LumNet_SignedFixedToRadians( azimuthFixed );
}

float ModelHReturnIterator::Elevation() const
{
    int16_t elevationFixed = LumNet_RayHeader_GetAngleElevation( &GetCurrentRay()->rayHeader );
    return LumNet_SignedFixedToRadians( elevationFixed );
}

uint32_t ModelHReturnIterator::MicrosecondTimestamp() const
{
    return LumNet_RayHeader_GetMicroseconds( &GetCurrentRay()->rayHeader );
}

float ModelHReturnIterator::Range() const
{
    uint32_t rangeCount = LumNet_RayReturn_GetRange( GetCurrentReturn() );
    return LumNet_RangeFixedToFloat( rangeCount );
}

float ModelHReturnIterator::Reflectance() const
{
    uint32_t analogReflectance = LumNet_RayReturn_GetReflectance( GetCurrentReturn() );
    return LumNet_ReflectanceFixedToFloat( analogReflectance );
}

uint8_t ModelHReturnIterator::ScanSegmentIndex() const
{
    uint8_t ssi = LumNet_RayHeader_GetSSI( &GetCurrentRay()->rayHeader );
    return ssi;
}

// Get the eye of the current return
uint8_t ModelHReturnIterator::Eye() const
{
    uint8_t eye = LumNet_RayHeader_GetEyeIndex( &GetCurrentRay()->rayHeader, 1 );
    return eye;
}

LumNet_Ray* ModelHReturnIterator::GetCurrentRay() const
{
    return reinterpret_cast<LumNet_Ray*>( mpRayPtr );
}

LumNet_RayReturn* ModelHReturnIterator::GetCurrentReturn() const
{
    return reinterpret_cast<LumNet_RayReturn*>( mpReturnPtr );
}

bool ModelHReturnIterator::IsRay() const
{
    LumNet_RayHeader* testHeader = reinterpret_cast<LumNet_RayHeader*>( mpReturnPtr );
    return LumNet_RayHeader_GetRayHeaderBit( testHeader ) == 1;
}
