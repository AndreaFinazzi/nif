/*
* BinaryHeader.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/
#pragma once

#include <stdint.h>
#include <iostream>
#include "luminlib_export.h"

#include "LumNet/LidarData.h"

namespace lum
{
/** The .lum binary header format */
struct LUMINLIB_EXPORT BinaryHeader
{
    /**
        \enum Compression
        \brief Supported compression types available when reading/writing.
     */
    enum Compression
    {
        none = 0,
        lz4 = 1  // default compression (v1.8.2)
    };
    /**
        Tests system byte ordering
        \returns True if system byte ordering is little endian, false otherwise.
     */
    static inline bool LittleEndian()
    {
        static const int i = 1;
        static const char* const c = reinterpret_cast<const char* const>( &i );
        return ( *c == 1 );
    }
    /** Default constructor */
    BinaryHeader()
        : byte_order( LittleEndian() ? 0 : 1 )
        , cartesian( 1 )
        , compression( (uint8_t)Compression::none )
        , lumver( 1 )
        , semver( LumNet_GetLidarDataProtocolSemVer() )
    {
    }
    /**
            Constructor
            \param  byte_order  The byte order (endianess) of the data in file
            \param  lumver      The .lum binary version
            \param  semver      The semantic version of the data
        */
    BinaryHeader(
        const uint8_t byte_order,
        const Compression compression,
        const uint8_t cartesian,
        const uint8_t lumver,
        const LumNet_Version semver )
        : byte_order( byte_order )
        , compression( ( uint8_t )( compression ) )
        , cartesian( cartesian )
        , lumver( lumver )
        , semver( semver )
    {
    }
    friend std::ostream& operator<<( std::ostream& os, const BinaryHeader& header )
    {
        return os << header.byte_order << header.compression << header.cartesian << header.lumver
                  << header.semver.major << header.semver.minor << header.semver.patch;
    }
    /** The byte order (endianness) of the data, 0 for little endian, 1 for big endian */
    uint8_t byte_order;
    /** The compression formatting used, see compression enum for available formats */
    uint8_t compression;
    /** The flag indicating represented data format 1 for cartesian, 0 for spherical */
    uint8_t cartesian;
    /** The version of the .lum binary file format */
    uint8_t lumver;
    /** The semantic version of the data */
    LumNet_Version semver;
};
}  // namespace lum
