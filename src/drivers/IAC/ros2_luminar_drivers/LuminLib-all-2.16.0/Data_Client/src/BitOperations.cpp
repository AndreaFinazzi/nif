/*
* BitOperations.cpp
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

#include "BitOperations.h"

uint32_t GetBitMask( uint8_t lowbit, uint8_t highbit )
{
    return ~( ~(uint32_t)0 << ( highbit + 1 ) ) & ~(uint32_t)0 << lowbit;
}

uint32_t GetBitRange( uint32_t word, uint8_t lowbit, uint8_t highbit )
{
    return ( word & GetBitMask( lowbit, highbit ) ) >> lowbit;
}

uint32_t GetBit( uint32_t word, uint8_t bit )
{
    return ( word >> bit ) & 0x1;
}

int32_t ConvertToSigned( uint32_t word, uint8_t bitlen )
{
    return ( int32_t )( word << ( 32 - bitlen ) ) / ( 1 << ( 32 - bitlen ) );
}
