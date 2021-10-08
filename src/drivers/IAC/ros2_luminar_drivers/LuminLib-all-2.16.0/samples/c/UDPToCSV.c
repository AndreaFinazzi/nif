/**
 * @file   udp_server_to_csv.c
 * @author Istvan Burbank
 * @date   2018-03-15
 * @brief  Use the lum-data-client to stream packet data to a csv
 *
 * Build:
 *   $ On Linux:
 *     gcc -o UDPToCSV -Wall -Wextra -Werror -pedantic -std=gnu99 -I../../
 *     lumDataPacketIndexer.c UDPToCSV.c
 *
 *   $ Or:
 *     Run CMake to generate platform specific project for your compiler of choice
 *
 * Usage:
 *   ./UDPToCSV <port> > csv_output.csv
 */

/*********************************/
/* Local Includes                */
/*********************************/
/* Includes that only need to be see by code here                             */
//@{

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

// Windows Headers
#ifdef WIN32
#include <WinSock2.h>
#else  // Linux Headers
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "lumDataPacketIndexer.h"

//@}

/********************************/
/* LOCAL Macro Definitions      */
/********************************/
/* Naming Examples:                                                           */
/* - #define FORMAT(a)    (a)                                                 */
/* - #define MAGIC_NUMBER (1)                                                 */
//@{

#define PACKET_BUFSIZE ( 1500 )

/**
 * We keep a pool allocated to store the Ray Index in. This calculates how many
 * index entries we'd need for a maximum-sized packet, i.e. the total number of
 * bytes in the packet minus the preamble, and assuming we have all ray headers
 * with no returns (the worst case), we divide by the size of the header.
 */
#define RAY_INDEX_POOL_LENGTH                                          \
    ( ( PACKET_BUFSIZE - sizeof( struct LumNet_LidarDataPreamble ) ) / \
      sizeof( LumNet_RayHeader ) )

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

int main( int argc, char **argv )
{
    /*
     * check command line arguments
     */
    if ( argc != 2 )
    {
        fprintf( stderr,
                 "UDP To CSV conversion tool.\n"
                 "---------------------------\n\n"
                 "Usage (prints errors/CSV to console): %s <port>\n"
                 "Usage (prints errors to console, CSV to file): %s <port> > "
                 "/path/to/data.csv\n"
                 "Usage (prints errors to console, discard data): %s <port> > "
                 "/dev/null\n",
                 argv[0], argv[0], argv[0] );
        exit( 1 );
    }
    int portno = atoi( argv[1] );
#define MAX_PORTNO ( 65535 )
    if ( portno < 1 || portno > MAX_PORTNO )
    {
        fprintf( stderr, "ERROR invalid port number\n" );
        exit( EXIT_FAILURE );
    }

    /*
     * socket: create the parent socket
     */
#ifdef WIN32
    LPWSADATA startupInfo;
    int winsockStartup = WSAStartup( 2.2, &startupInfo );
    if ( winsockStartup < 0 )
    {
        fprintf( stderr, "ERROR Winsock failed to initialize\n" );
        exit( EXIT_FAILURE );
    }
    int protocol = IPPROTO_UDP;
#else
    int protocol = 0;
#endif
    int sockfd = socket( AF_INET, SOCK_DGRAM, protocol );
    if ( sockfd < 0 )
    {
        fprintf( stderr, "ERROR opening socket\n" );
        exit( EXIT_FAILURE );
    }

    /* setsockopt: Handy debugging trick that lets
    * us rerun the server immediately after we kill it;
    * otherwise we have to wait about 20 secs.
    * Eliminates "ERROR on binding: Address already in use" error.
    */
    int optval = 1;
    setsockopt( sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval,
                sizeof( optval ) );

    /*
     * build the server's Internet address
     */
    struct sockaddr_in serveraddr = {0};
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl( INADDR_ANY );
    serveraddr.sin_port = htons( (unsigned short)portno );

    /*
     * bind: associate the parent socket with a port
     */
    const int bind_ret = bind( sockfd, (struct sockaddr *)&serveraddr, sizeof( serveraddr ) );
    if ( bind_ret < 0 )
    {
        fprintf( stderr, "ERROR on binding (returned: %i)\n", bind_ret );
        exit( EXIT_FAILURE );
    }

    // print the first row of the csv
    printf( "TIMESTAMP(SEC), TIMESTAMP(USEC), SCAN COUNT, CHECKPOINT, SSI, "
            "EYE INDEX, AZ ANGLE, EL ANGLE, RETURN NUMBER, "
            "RANGE, REFLECTANCE\n" );
    /*
     * main loop: wait for a datagram
     */
    while ( 1 )
    {
        /*
         * recvfrom: receive a UDP datagram from a client
         */
        static char buf[PACKET_BUFSIZE]; /* message buf */

        struct sockaddr_in clientaddr; /* client addr */
        unsigned int clientlen = sizeof( clientaddr );
        int msg_size = recvfrom( sockfd, buf, PACKET_BUFSIZE, 0,
                                 (struct sockaddr *)&clientaddr, &clientlen );
        if ( msg_size < 0 )
        {
            fprintf( stderr, "ERROR in recvfrom (returned %i)\n", msg_size );
            continue;
        }

        /*
         * Index the packet
         */
        struct LumNet_LidarDataPreamble const *preamble;
        static struct LumDataPacketIndexer_rayIndexEntry
            rayIndex[RAY_INDEX_POOL_LENGTH];
        uint16_t rayIndexLength;

        const enum LumDataPacketIndexer_returnCode parseReturnCode =
            LumDataPacketIndexer_Index( buf, msg_size, &preamble, rayIndex,
                                        RAY_INDEX_POOL_LENGTH, &rayIndexLength );

        if ( parseReturnCode != LUM_DATA_PACKET_INDEXER_RETURN_CODE_SUCCESS )
        {
            fprintf( stderr, "ERROR: parse was unsuccessful. Return was: %i\n",
                     parseReturnCode );
            continue;
        }

        static uint8_t last_seq_num = 0;
        if ( preamble->sequence_number != ( ( uint8_t )( last_seq_num + 1 ) ) )
        {
            fprintf( stderr, "bad seq num. Expected: %i, Got: %i\n", last_seq_num + 1,
                     preamble->sequence_number );
        }
        last_seq_num = preamble->sequence_number;

        /*
         * Output packet contents as CSV
         */
        for ( uint16_t rayIdx = 0; rayIdx < rayIndexLength; rayIdx++ )
        {
            for ( uint8_t returnIdx = 0; returnIdx < rayIndex[rayIdx].num_returns;
                  returnIdx++ )
            {
                // Make a convenient name for this ray/return
                struct LumNet_Ray const *const ray = rayIndex[rayIdx].ray;
                LumNet_RayReturn const *const rayReturn =
                    (LumNet_RayReturn const *)&( ray->rayReturn[returnIdx] );

                printf( "%u, "                                                // TIMESTAMP(SEC),
                        "%u, "                                                // TIMESTAMP(USEC),
                        "%u, "                                                // SCAN COUNT,
                        "%u, "                                                // CHECKPOINT,
                        "%u, "                                                // SSI,
                        "%u, "                                                // Eye Index,
                        "%f, "                                                // AZ ANGLE,
                        "%f, "                                                // EL ANGLE,
                        "%u, "                                                // RETURN NUMBER,
                        "%f, "                                                // RANGE,
                        "%u\n",                                               // REFLECTANCE
                        preamble->timestamp_seconds,                          // TIMESTAMP SEC
                        LumNet_RayHeader_GetMicroseconds( &ray->rayHeader ),  // USEC
                        preamble->scan_counters,                              // SCAN COUNT
                        preamble->checkpoint,                                 // CHECKPOINT
                        LumNet_RayHeader_GetSSI( &ray->rayHeader ),           // SSI
                        LumNet_RayHeader_GetEyeIndex( &ray->rayHeader, 1 ),   // EYE
                        LumNet_SignedFixedToDegrees(
                            LumNet_RayHeader_GetAngleAzimuth( &ray->rayHeader ) ),
                        LumNet_SignedFixedToDegrees(
                            LumNet_RayHeader_GetAngleElevation( &ray->rayHeader ) ),
                        returnIdx,
                        LumNet_RangeFixedToFloat( LumNet_RayReturn_GetRange( rayReturn ) ),
                        LumNet_RayReturn_GetReflectance( rayReturn ) );
            }
        }
    }

    return EXIT_SUCCESS;
}
//@}

/********************************/
/* LOCAL Function Definitions   */
/********************************/
//@{

//@}
