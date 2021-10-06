/**
 * @file   lum-data-client-demo.c
 * @author Istvan Burbank
 * @date   2017-03-01
 * @brief  Demo of the lum-data-client
 *
 * This `main()` function takes a "dummy", pre-constructed packet and parses it
 * with the lum-data-client. The contents of each field are printed to the
 * console.
 *
 * Build:
 *   $ gcc -o IndexSampleDataPacket -Wall -Wextra -Werror -pedantic -std=gnu99
 *     -I../../ lumDataPacketIndexer.c IndexSampleDataPacket.c
 *
 */

/*********************************/
/* Local Includes                */
/*********************************/
/* Includes that only need to be see by code here                             */
//@{

#include "lumDataPacketIndexer.h"

#include <stdio.h>

//@}

/********************************/
/* LOCAL Macro Definitions      */
/********************************/
/* Naming Examples:                                                           */
/* - #define FORMAT(a)    (a)                                                 */
/* - #define MAGIC_NUMBER (1)                                                 */
//@{

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

#define PACKET_NUM_WORDS (14)
static const uint32_t packet[PACKET_NUM_WORDS] = {
    /*
    31           24 23           16 15            8 7            0
    +---------------+---------------+---------------+--------------+
    | Major         | Minor         | Patch         | Packet Type  |
    +---------------+---------------+---------------+--------------+
    */
    0x02000001,

    /*
     31                           16 15            8 7            0
    +-------------------------------+---------------+--------------+
    | Fingerprint                   | seq number    | num_rays     |
    +-------------------------------+---------------+--------------+
    */
    0xAAAA0002,

    /*
     31                                                           0
    +--------------------------------------------------------------+
    | Unix timestamp according to the sender                       |
    +--------------------------------------------------------------+
    */
    0x1234abcd,

    /*
    31           24 23           16 15           8 7             0
    +---------------+---------------+--------------+---------------+
    | Scan Count    | Checkpoint    | Scan Profile | RESERVED      |
    +---------------+---------------+--------------+---------------+
    */
    0x00010200,

    /*
     31  30      24 23      20 19                          0
    +---+----------+----------+-----------------------------+
    | 1 | SSI      | RESERVED | Timestamp                   |
    +---+----------+----------+-----------------------------+
    */
    0x82202010,
    /*
     31                       16 15                        0
    +---------------------------+---------------------------+
    | Azimuth Angle             | Elevation Angle           |
    +---------------------------+---------------------------+
    */
    0x12345678,

    /*
     31  30                       12 11                    0
    +---+---------------------------+-----------------------+
    | 0 | Range N                   |           Intensity N |
    +---+---------------------------+-----------------------+
    */
    0x00100001,
    0x00200002,
    0x00300003,

    /*
     31  30      24 23      20 19                          0
    +---+----------+----------+-----------------------------+
    | 1 | SSI      | RESERVED | Timestamp                   |
    +---+----------+----------+-----------------------------+
    */
    0x84502011,
    /*
     31                       16 15                        0
    +---------------------------+---------------------------+
    | Azimuth Angle             | Elevation Angle           |
    +---------------------------+---------------------------+
    */
    0x9abcdef0,

    /*
     31  30                       12 11                    0
    +---+---------------------------+-----------------------+
    | 0 | Range N                   |           Intensity N |
    +---+---------------------------+-----------------------+
    */
    0x00400004,
    0x00500005,
    0x00600006,
};

//@}

/********************************/
/* LOCAL Function Declarations  */
/********************************/
/* Everything should be static  */
/* Naming Examples:                                                           */
/* - static void MapPins();                                                   */
//@{

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

int main() {
/*
 * Index the packet
 */
#define POOL_NUM_RAYS (2)
#define BYTES_IN_WORD (4)

  struct LumNet_LidarDataPreamble *         preamble;
  struct LumDataPacketIndexer_rayIndexEntry rayIndex[POOL_NUM_RAYS];
  uint16_t                                  rayIndexLength;

  const enum LumDataPacketIndexer_returnCode parseReturnCode =
      LumDataPacketIndexer_Index((char *) packet,
                                 PACKET_NUM_WORDS * BYTES_IN_WORD, &preamble,
                                 rayIndex, POOL_NUM_RAYS, &rayIndexLength);

  if (parseReturnCode != LUM_DATA_PACKET_INDEXER_RETURN_CODE_SUCCESS) {
    fprintf(stderr, "ERROR: parse was unsuccessful. Return was: %i\n",
            parseReturnCode);
    return parseReturnCode;
  }

  printf("\nPacket Type: \t%x\n", preamble->semVersionedHeader.payloadTypeID);
  printf("Major:\t\t%x\n", preamble->semVersionedHeader.semver.major);
  printf("Minor:\t\t%x\n", preamble->semVersionedHeader.semver.minor);
  printf("Patch:\t\t%x\n", preamble->semVersionedHeader.semver.patch);
  printf("Fingerprint:\t%x\n", preamble->fingerprint);
  printf("Seq Num:\t%x\n", preamble->sequence_number);
  printf("Num Rays:\t%x\n", preamble->num_rays);
  printf("Timestamp:\t%x\n", preamble->timestamp_seconds);
  printf("Scan Count:\t%x\n", preamble->scan_count);
  printf("Checkpoint:\t%x\n", preamble->checkpoint);
  printf("Scan Profile:\t%x\n", preamble->scan_profile);

  for (uint16_t rayIdx = 0; rayIdx < rayIndexLength; ++rayIdx) {
    struct LumNet_Ray const *const ray = rayIndex[rayIdx].ray;

    printf("\nTimestamp us:\t%x\n",
           LumNet_RayHeader_GetMicroseconds(&ray->rayHeader));
    printf("Azimuth:\t%x\n", LumNet_RayHeader_GetAngleAzimuth(&ray->rayHeader));
    printf("Elevation:\t%x\n",
           LumNet_RayHeader_GetAngleElevation(&ray->rayHeader));
    printf("SSI:\t\t%x\n", LumNet_RayHeader_GetSSI(&ray->rayHeader));
    printf("EyeIndex:\t%x\n", LumNet_RayHeader_GetEyeIndex(&ray->rayHeader, 1));

    for (uint8_t returnIdx = 0; returnIdx < rayIndex[rayIdx].num_returns;
         ++returnIdx) {
      LumNet_RayReturn const *const rayReturn =
          (LumNet_RayReturn const *) &(ray->rayReturn[returnIdx]);
      printf("range %i:\t%x\n", returnIdx,
             LumNet_RayReturn_GetRange(rayReturn));
      printf("intensity %i:\t%x\n", returnIdx,
             LumNet_RayReturn_GetIntensity(rayReturn));
    }
  }
  return 0;
}

//@}

/********************************/
/* LOCAL Function Definitions   */
/********************************/
//@{

//@}
