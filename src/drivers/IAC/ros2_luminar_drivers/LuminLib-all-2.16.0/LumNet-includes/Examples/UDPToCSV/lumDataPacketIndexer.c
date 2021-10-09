/**
 * @file   lumDataPacketIndexer.c
 * @author Istvan Burbank
 * @date   2018-06-17
 * @brief  In-place indexing of Luminar Data Packets
 */

/*********************************/
/* Local Includes                */
/*********************************/
/* Includes that only need to be see by code here                             */
//@{

#include "lumDataPacketIndexer.h"

#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
#include <stdio.h>
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

//@}

/********************************/
/* LOCAL Macro Definitions      */
/********************************/
/* Naming Examples:                                                           */
/* - #define FORMAT(a)    (a)                                                 */
/* - #define MAGIC_NUMBER (1)                                                 */
//@{

/**
 * @brief Packets are comprised of 32-bit (4-byte) words
 */
#define BYTES_IN_PACKET_WORD (4)

/**
 * @brief The maximum number of returns we expect to see per ray.
 *
 * Used for error checking/debugging.
 */
#define MAX_NUMBER_OF_RETURNS (3)

//@}

/********************************/
/* LOCAL Type(def) Declarations */
/********************************/
/* Don't typedef structs/unions */
/* Naming Examples:                                                           */
/* - struct packetContainer {...};                                            */
/* - typedef unsigned int rowIndex;                                           */
/* - enum itemTypes {ITEM_TYPE_A, ITEM_TYPE_B};                               */
//@{

//@}

/********************************/
/* LOCAL Variable Definitions   */
/********************************/
/* Everything should be static  */
/* Variables shan't be initialized in definition unless variable is a pointer */
/* In this case it will explicitly be set to NULL                             */
/* Variables should then be initialized in a corresponding InitVars function  */
/* Naming Examples:                                                           */
/* - static int errorCounter;                                                 */
/* - static int *configRegister = NULL;                                       */
//@{

//@}

/********************************/
/* LOCAL Function Declarations  */
/********************************/
/* Everything should be static  */
/* Naming Examples:                                                           */
/* - static void MapPins();                                                   */
//@{

static inline bool LumNet_IsWordRayReturn(uint32_t const *const word);

//@}

/********************************/
/* GLOBAL Variable Definitions  */
/********************************/
//@{

//@}

/********************************/
/* GLOBAL Function Definitions  */
/********************************/
//@{

enum LumDataPacketIndexer_returnCode LumDataPacketIndexer_Index(
    char const *const packet, uint32_t const packetNumBytes,
    struct LumNet_LidarDataPreamble const **const    preamble,
    struct LumDataPacketIndexer_rayIndexEntry *const rayIndexPool,
    uint16_t const rayIndexPoolMaxLength, uint16_t *const rayIndexLength) {

  // Ensuring the packet is evenly divisible into 32-bit words
  if (packetNumBytes % BYTES_IN_PACKET_WORD != 0) {
#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
    fprintf(stderr,
            "Error: packetNumBytes is not evenly divisible by 4 (given %u)\n",
            packetNumBytes);
#endif

    return LUM_DATA_PACKET_INDEXER_RETURN_CODE_MALFORMED_PACKET;
  }

  uint32_t packetIndex = 0;

  // Set a default
  *rayIndexLength = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Parse Preamble

  // Ensure there are enough words for the preamble
  if (packetIndex + sizeof(struct LumNet_LidarDataPreamble) > packetNumBytes) {
#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
    fprintf(stderr, "Error: Truncated reading preamble\n");
#endif

    return LUM_DATA_PACKET_INDEXER_RETURN_CODE_TRUNCATED_PACKET;
  }

  // Assign the preamble to point to the preamble in the packet
  *preamble = (struct LumNet_LidarDataPreamble const *) (packet + packetIndex);

  const struct LumNet_Version parserSemver =
      LumNet_GetLidarDataProtocolSemVer();
  if (!LumNet_VersionIsCompatible(parserSemver,
                                  (*preamble)->semVersionedHeader.semver)) {
#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
    fprintf(stderr,
            "Error: Packet and indexer have version mismatch: \n"
            "\tPacket: %u.%u.%u, Parser: %u.%u.%u\n",
            (*preamble)->semVersionedHeader.semver.major,
            (*preamble)->semVersionedHeader.semver.minor,
            (*preamble)->semVersionedHeader.semver.patch, parserSemver.major,
            parserSemver.minor, parserSemver.patch);
#endif

    return LUM_DATA_PACKET_INDEXER_RETURN_CODE_VERSION_MISMATCH;
  }

  // Advance packet pointer by the size of the preamble
  packetIndex += sizeof(struct LumNet_LidarDataPreamble);

  if (packetIndex >= packetNumBytes) {
    if ((*preamble)->num_rays == 0) {
      // There are no rays in this packet so all is as-expected
      return LUM_DATA_PACKET_INDEXER_RETURN_CODE_SUCCESS;
    } else {
      // The preamble promised there are rays, but there aren't enough words for
      // there to be rays!
      return LUM_DATA_PACKET_INDEXER_RETURN_CODE_TRUNCATED_PACKET;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  // Parse Ray Data Structures

  // Each Ray Data Structure gets one rayIndex entry, so ensure:
  //   - We haven't overrun the packet AND
  //   - We haven't exhausted the preamble's num_rays AND
  //   - We haven't filled the rayIndexPool
  //
  // Note that the first and second checks are redundant in a well-formed
  // packet, but both are included for safety.
  while (packetIndex < packetNumBytes &&
         *rayIndexLength < (*preamble)->num_rays &&
         *rayIndexLength < rayIndexPoolMaxLength) {

    ///////////////////
    // Index Ray Header

    // Check to make sure there is at least another Ray Header left in the
    // packet. Because we are still looping, we know there are more rays to be
    // sent according to the packet header so return an error if not.
    if (packetIndex + sizeof(LumNet_RayHeader) > packetNumBytes) {
#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
      fprintf(stderr,
              "Error: truncated parsing ray header\n"
              "\textra words: %u, next word: %x\n",
              (packetNumBytes - packetIndex),
              *((uint32_t const *) (packet + packetIndex)));
#endif
      return LUM_DATA_PACKET_INDEXER_RETURN_CODE_TRUNCATED_PACKET;
    }

    // Put a pointer to the ray into the index. Increment `rayIndexLength` after
    // we parse returns for convenience, so we can use the pointer while parsing
    // rays.
    rayIndexPool[*rayIndexLength].ray =
        (struct LumNet_Ray const *) (packet + packetIndex);

    // Advance the packet pointer by the two words we just parsed
    packetIndex += sizeof(LumNet_RayHeader);

    ////////////////
    // Count Returns

    // Set a default number of returns to 0
    rayIndexPool[*rayIndexLength].num_returns = 0;

    // Loop over returns, if there are any, and count them
    while (packetIndex < packetNumBytes &&
           LumNet_IsWordRayReturn((uint32_t const *) (packet + packetIndex))) {

      packetIndex += sizeof(LumNet_RayReturn);

      rayIndexPool[*rayIndexLength].num_returns++;

#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
      if (rayIndexPool[*rayIndexLength].num_returns > MAX_NUMBER_OF_RETURNS) {
        fprintf(stderr,
                "WARNING: More than %u returns found in a ray (current "
                "return number: %u)!\n",
                MAX_NUMBER_OF_RETURNS,
                rayIndexPool[*rayIndexLength].num_returns);
      }
#endif
    }

    // Increment the index into the rayIndex data structure
    (*rayIndexLength)++;
  }

  // Check that we got the correct number of rays, i.e.:
  //  - Make sure we didn't stop before getting the number of rays the preamble
  //    promised. And if we did, was it because we:
  //      - Exhausted the packet words in the packet or
  //      - We exhausted the rayPool
  //  - We got as many rays as the preamble promised, but there are words left
  //    in the packet when we finished
  if ((*preamble)->num_rays != *rayIndexLength) {
    if (packetIndex >= packetNumBytes) {
#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
      fprintf(stderr,
              "ERROR: Preamble num_rays %u != num parsed rays %u! Ran "
              "out of bytes in the packet to parse.\n",
              (*preamble)->num_rays, *rayIndexLength);
#endif
      return LUM_DATA_PACKET_INDEXER_RETURN_CODE_TRUNCATED_PACKET;
    } else if (*rayIndexLength >= rayIndexPoolMaxLength) {
#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
      fprintf(stderr,
              "ERROR: Preamble num_rays %u != num parsed rays %u! "
              "Filled the Ray Pool.\n",
              (*preamble)->num_rays, *rayIndexLength);
#endif
      return LUM_DATA_PACKET_INDEXER_RETURN_CODE_RAY_POOL_EXHAUSTED;
    }

  } else if (packetIndex < packetNumBytes) {
#if LUM_DATA_PACKET_INDEXER_PRINT_DEBUG == true
    fprintf(stderr,
            "ERROR: Parsed the preamble's num_rays (%u), but there were %u "
            "words left\n",
            (*preamble)->num_rays, packetNumBytes - packetIndex);
#endif

    return LUM_DATA_PACKET_INDEXER_RETURN_CODE_PARTIAL_INDEX;
  }

  return LUM_DATA_PACKET_INDEXER_RETURN_CODE_SUCCESS;
}

//@}

/********************************/
/* LOCAL Function Definitions   */
/********************************/
//@{

static inline bool LumNet_IsWordRayReturn(uint32_t const *const word) {
  return LumNet_RayReturn_GetReturnBit((LumNet_RayReturn const *) word) == 0;
}

//@}
