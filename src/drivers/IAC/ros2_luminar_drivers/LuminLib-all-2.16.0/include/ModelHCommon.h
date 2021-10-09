/*
 * ModelHCommon.h
 *
 * Copyright (c) 2018, Luminar Technologies, Inc.
 *
 * This material contains confidential and trade secret information of Luminar Technologies.
 * Reproduction, adaptation, and distribution are prohibited, except to the extent expressly permitted in
 * writing by Luminar Technologies.
 */

/*
 * Defines `LUM_FORCE_INLINE` and `LUM_NEVER_INLINE` macros
 */

#ifndef MODEL_H_DATA_COMMON_H
#define MODEL_H_DATA_COMMON_H

#if defined( _MSC_VER )

#define LUM_FORCE_INLINE __forceinline
#define LUM_NEVER_INLINE __declspec( noinline )

#else  //  defined(_MSC_VER)

#define LUM_FORCE_INLINE inline __attribute__( ( always_inline ) )
#define LUM_NEVER_INLINE __attribute__( ( noinline ) )

#endif  //  !defined(_MSC_VER)

#endif
