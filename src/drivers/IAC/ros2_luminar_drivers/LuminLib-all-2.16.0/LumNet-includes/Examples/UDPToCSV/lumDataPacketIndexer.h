/**
 * @file   lumDataPacketIndexer.c
 * @author Istvan Burbank
 * @date   2018-06-17
 * @brief  In-place indexing of Luminar Data Packets
 *
 */

#ifndef LUM_DATA_PACKET_INDEXER_H
#define LUM_DATA_PACKET_INDEXER_H

/*********************************/
/* Includes                      */
/*********************************/
/* Includes that anyone who includes this header needs                        */
//@{

#include <stdint.h>
#include "LumNet/LidarData.h"

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
/* Naming Examples:                                                           */
/* - #define EXAMPLE_TEMPLATE_VERSION_NUMBER (1)                              */
/* - #define EXAMPLE_TEMPLATE_FORMAT(a)      (a)                              */
//@{

/**
 * @brief If defined print errors to stderr
 *
 * If it is not defined dependency on stdio is removed.
 */
#define LUM_DATA_PACKET_INDEXER_PRINT_DEBUG (true)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
/* Don't typedef structs/unions                                               */
/* Naming Examples:                                                           */
/* - union   Template_parameterOverlay {...};                                 */
/* - typedef unsigned int ExampleTemplate_columnIndex;                        */
/* - enum Template_itemType {TEMPLATE_ITEM_TYPE_A, TEMPLATE_ITEM_TYPE_B};     */
//@{

/**
 * Error codes returned by some functions in the LumPointCloudParser
 */
enum LumDataPacketIndexer_returnCode {
  /** The function has returned without error. */
  LUM_DATA_PACKET_INDEXER_RETURN_CODE_SUCCESS = 0,

  /** An error has occurred, no more specific information is available. */
  LUM_DATA_PACKET_INDEXER_RETURN_CODE_ERROR = -1,

  /** Version of the packet was not compatible with the SDK and the function
      aborted. */
  LUM_DATA_PACKET_INDEXER_RETURN_CODE_VERSION_MISMATCH = -2,

  /** The packet is malformed (e.g. the packet length is not a multiple of 4
      bytes), and a more specific error was not determined. */
  LUM_DATA_PACKET_INDEXER_RETURN_CODE_MALFORMED_PACKET = -3,

  /** The preamble's num_rays couldn't be found in the provided packet - we ran
      out of data to index. */
  LUM_DATA_PACKET_INDEXER_RETURN_CODE_TRUNCATED_PACKET = -4,

  /** Indexing some of the data was successful, but there appears to be extra
      data after the expected number of rays. */
  LUM_DATA_PACKET_INDEXER_RETURN_CODE_PARTIAL_INDEX = -5,

  /** Indexing some of the data was successful, but the Ray Pool was filled
      before indexing all of the packet. */
  LUM_DATA_PACKET_INDEXER_RETURN_CODE_RAY_POOL_EXHAUSTED = -6,
};

struct LumDataPacketIndexer_rayIndexEntry {
  struct LumNet_Ray const *ray;
  uint8_t                  num_returns;
};

//@}

/*********************************/
/* GLOBAL Variable Declarations  */
/*********************************/
/* Declare as extern                                                          */
/* Naming Examples:                                                           */
/* - extern int ExampleTemplate_variableName;                                 */
//@{

//@}

/*********************************/
/* GLOBAL Function Declarations  */
/*********************************/
/* Don't declare as extern                                                    */
/* Naming Examples:                                                           */
/* - void ExampleTemplate_Init( void );                                       */
/* - void Template_ReturnZero( void );                                        */
//@{

/**
 * @brief In-place parse of data packet, returning pointers to the preamble/rays
 *
 * @param packet UDP data contents
 * @param packetNumBytes Number of bytes in the packet
 * @param preamble The pointer to the preamble, to be set by the indexer
 * @param rayIndexPool Memory pool pre-allocated to store the index in
 * @param rayIndexPoolMaxLength Maximum number of rayIndexEntries to write
 * @param rayIndexLength Number of rays in the rayIndex array
 *
 * @return Error code if applicable, otherwise success
 */
enum LumDataPacketIndexer_returnCode LumDataPacketIndexer_Index(
    char const *const packet, uint32_t const packetNumBytes,
    struct LumNet_LidarDataPreamble const **const    preamble,
    struct LumDataPacketIndexer_rayIndexEntry *const rayIndexPool,
    uint16_t const rayIndexPoolMaxLength, uint16_t *const rayIndexLength);

//@}

/*********************************/
/* GLOBAL Function Definitions   */
/*********************************/
/* Declare as inline                                                          */
/* These should only be small, performance-critical functions in the header   */
//@{

//@}

#endif // LUM_DATA_PACKET_INDEXER_H
