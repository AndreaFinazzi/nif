/*
* BitOperations.h
*
* Copyright (c) 2018, Luminar Technologies, Inc.
*
* This material contains confidential and trade secret information of Luminar Technologies.
* Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
* writing by Luminar Technologies.
*/

/*! \brief BitOperations
 *         Bit-field manipulation helper functions
 *
 */

#pragma once

#include <stdint.h>

// Compute a bit mask for a specified range
uint32_t GetBitMask( uint8_t lowbit, uint8_t highbit );

// Retrieve specified range of bits
uint32_t GetBitRange( uint32_t word, uint8_t lowbit, uint8_t highbit );

// Retrieve individual bit
uint32_t GetBit( uint32_t word, uint8_t bit );

/*
* Sign extend and convert to signed a number represented by fewer bits
* than the storage data type. For example, convert a 24-bit signed
* quantity stored in an unsigned 32 bit representation to 32-bit signed.
*/
int32_t ConvertToSigned( uint32_t word, uint8_t bitlen );
