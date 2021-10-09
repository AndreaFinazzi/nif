/**
 * @file   StatusPacketParser.c
 * @author Istvan Burbank
 * @date   2020-02-13
 * @brief Enable UDP status stream and print fields
 *
 * Build:
 *   $ mkdir build && cd build && cmake .. && make
 *
 * Usage:
 *   ./StatusPacketParser <sensorIP> <sensor TCP port> <status dest port>
 */

/*********************************/
/* Local Includes                */
/*********************************/
/* Includes that only need to be see by code here                             */
//@{

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <sys/types.h>

#include "LumNet/Common.h"
#include "LumNet/Common/PayloadTypeID.h"
#include "LumNet/Command.h"

// Windows Headers
#ifdef WIN32
#include <WinSock2.h>
#else // Linux Headers
#include <netdb.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>
#endif

#include "jsmn/jsmn.h"

//@}

/********************************/
/* LOCAL Macro Definitions      */
/********************************/
/* Naming Examples:                                                           */
/* - #define FORMAT(a)    (a)                                                 */
/* - #define MAGIC_NUMBER (1)                                                 */
//@{

#define PACKET_BUFSIZE (1500)

#define PREAMBLE_NUM_BYTES (4)

#define MAX_NUM_JSON_TOKENS (128)

#define MAX_PORT (65535)

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

static bool     jsonStrEq(const char *json, jsmntok_t *tok, const char *s);
static uint64_t tokenToL(const char *json, jsmntok_t *tok);
static bool     tokenToBool(const char *json, jsmntok_t *tok);

static void startStatusPackets(char const *sensorIP, int sensor_port);

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

int main(int argc, char **argv) {
  /*
   * check command line arguments
   */
  if (argc != 4) {
    fprintf(stderr,
            "Status Packet Parser.\n"
            "---------------------\n"
            "Usage : %s <sensorIP> <sensor TCP port> <status dest port>\n",
            argv[0]);
    exit(1);
  }

  const char *sensor_ip = argv[1];

  const int sensor_port = atoi(argv[2]);
  if (sensor_port < 1 || sensor_port > MAX_PORT) {
    fprintf(stderr, "ERROR invalid port number\n");
    exit(EXIT_FAILURE);
  }

  // First, configure the sensor to send status updates
  startStatusPackets(sensor_ip, sensor_port);

  const int status_dest_port = atoi(argv[3]);
  if (status_dest_port < 1 || status_dest_port > MAX_PORT) {
    fprintf(stderr, "ERROR invalid port number\n");
    exit(EXIT_FAILURE);
  }

    /*
     * socket: create the parent socket
     */
#ifdef WIN32
  LPWSADATA startupInfo;
  int       winsockStartup = WSAStartup(2.2, &startupInfo);
  if (winsockStartup < 0) {
    fprintf(stderr, "ERROR Winsock failed to initialize\n");
    exit(EXIT_FAILURE);
  }
  int protocol = IPPROTO_UDP;
#else
  int protocol = 0;
#endif
  int sockfd = socket(AF_INET, SOCK_DGRAM, protocol);
  if (sockfd < 0) {
    fprintf(stderr, "ERROR opening socket\n");
    exit(EXIT_FAILURE);
  }

  /* setsockopt: Handy debugging trick that lets
   * us rerun the server immediately after we kill it;
   * otherwise we have to wait about 20 secs.
   * Eliminates "ERROR on binding: Address already in use" error.
   */
  int optval = 1;
  setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *) &optval,
             sizeof(optval));

  /*
   * build the server's Internet address
   */
  struct sockaddr_in serveraddr = {0};
  serveraddr.sin_family         = AF_INET;
  serveraddr.sin_addr.s_addr    = htonl(INADDR_ANY);
  serveraddr.sin_port           = htons((unsigned short) status_dest_port);

  /*
   * bind: associate the parent socket with a port
   */
  const int bind_ret =
      bind(sockfd, (struct sockaddr *) &serveraddr, sizeof(serveraddr));
  if (bind_ret < 0) {
    fprintf(stderr, "ERROR on binding (returned: %i)\n", bind_ret);
    exit(EXIT_FAILURE);
  }

  /*
   * main loop: wait for a datagram
   */
  while (1) {

    /*
     * recvfrom: receive a UDP datagram from a client
     */
    static char buf[PACKET_BUFSIZE]; /* message buf */

    struct sockaddr_in clientaddr; /* client addr */
    unsigned int       clientlen = sizeof(clientaddr);
    int                msg_size  = recvfrom(sockfd, buf, PACKET_BUFSIZE, 0,
                            (struct sockaddr *) &clientaddr, &clientlen);
    if (msg_size < 0) {
      fprintf(stderr, "ERROR in recvfrom (returned %i)\n", msg_size);
      continue;
    }

    /* PREAMBLE BIT LAYOUT:
     *  31           24 23           16 15            8 7            0
     * +---------------+---------------+---------------+--------------+
     * | Major         | Minor         | Patch         | Packet Type  |
     * +---------------+---------------+---------------+--------------+
     */

    if (buf[0] != LUM_NET_PAYLOAD_TYPE_ID_STATUS_PACKET) {
      fprintf(stderr, "ERROR: Wrong packet type.\n");
      continue;
    }

    printf("Version: Major %u: , Minor %u: , Patch %u: \n", buf[3], buf[2],
           buf[1]);

    // The JSON starts after the preamble
    char const *const json = buf + PREAMBLE_NUM_BYTES;

    // Print the packet
    printf("Json: %s\n", json);

    // Parse the packet
    jsmn_parser      parser;
    static jsmntok_t tokens[MAX_NUM_JSON_TOKENS];

    jsmn_init(&parser);

    const int parse_count =
        jsmn_parse(&parser, json, strlen(json), tokens, MAX_NUM_JSON_TOKENS);

    if (parse_count < 0) {
      switch ((enum jsmnerr) parse_count) {
      case JSMN_ERROR_NOMEM: {
        fprintf(stderr, "ERROR: Not enough tokens were provided.\n");
        break;
      }
      case JSMN_ERROR_INVAL: {
        fprintf(stderr, "ERROR:Invalid character inside JSON string.\n");
        break;
      }
      case JSMN_ERROR_PART: {
        fprintf(stderr, "ERROR: The string is not a full JSON packet, more "
                        "bytes expected.\n");
        break;
      }
      default: {
        fprintf(stderr, "ERROR: Unknown JSON parsing error: %i.\n",
                parse_count);
      }
      };

      continue; // don't process this packet further
    }

    // Output information about the packet
    for (int i = 1; i < parse_count; i++) {
      if (jsonStrEq(json, &tokens[i], "timestamp_sec_plus_ns")) {
        printf("Time(s): %lu\n",
               tokenToL(json, &tokens[i + 1]) / (uint64_t) 1E9);
        i++; // skip the index over the value
      } else if (jsonStrEq(json, &tokens[i], "syncd")) {
        jsmntok_t const *const sync_token = &tokens[i + 1];

        if (sync_token->type != JSMN_PRIMITIVE) {
          fprintf(stderr, "ERROR: syncd value is not primitive.\n");
        }

        printf("Sync status: %s\n", tokenToBool(json, &tokens[i + 1])
                                        ? "Synchronized"
                                        : "Not Synchronized");
        i++; // skip the index over the value
      } else {
        printf("Unexpected key: %.*s\n", tokens[i].end - tokens[i].start,
               json + tokens[i].start);
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

static bool jsonStrEq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0)
    return true;
  else
    return false;
}

static uint64_t tokenToL(const char *json, jsmntok_t *tok) {
  if (tok->type == JSMN_PRIMITIVE)
    return atol(json + tok->start);
  return 0;
}

static bool tokenToBool(const char *json, jsmntok_t *tok) {
  if (tok->type == JSMN_PRIMITIVE &&
      strncmp(json + tok->start, "true", tok->end - tok->start) == 0)
    return true;
  else
    return false;
}

static void startStatusPackets(char const *sensor_ip, int sensor_port) {

  const struct LumNet_CommandRequest enable_command = {
      .commandHeader =
          (struct LumNet_CommandRequestHeader){
              .address = LUM_NET_CMD_MAKE_ADDRESS(
                  LUM_NET_CMD_ADDR_BASE_NET, LUM_NET_CMD_ADDR_INDEX_NET_STATUS,
                  LUM_NET_CMD_ADDR_OFFSET_NET_STATUS_UDP_ENABLE),
              .operation     = LUM_NET_MAKE_OP(LUM_NET_CMD_OP_TYPE_WRITE,
                                           LUM_NET_CMD_OP_SUBTYPE_VOLATILE),
              .magicNumber   = LUM_NET_CMD_MAGIC_NUMBER,
              .transactionID = 0,
              .payloadLength = sizeof(LumNet_Bool),
          },
      .payload = {1}};

  const struct LumNet_CommandRequestListPayload commandRequestList = {
      .preamble =
          (struct LumNet_CommandListPreamble){
              .versionPayloadHeader =
                  (struct LumNet_CommonHeader){
                      .sentinel = 0x48325348, //'H' '2' 'S' 'H'
                      .semVersionedHeader =
                          (struct LumNet_SemVersionedPayloadHeader){
                              .payloadTypeID =
                                  LUM_NET_PAYLOAD_TYPE_ID_CMD_REQUEST_LIST,
                              .semver = LumNet_GetCommandProtocolSemVer()}}},
      .commandRequestListHeader =
          (struct LumNet_CommandRequestListHeader){
              .listTransactionID = 0,
              .listPayloadLength =
                  sizeof(enable_command)}, // sizeof everything after this
      .commands = {enable_command},
  };

  int                sockfd;
  struct sockaddr_in servaddr;

  // socket create and varification
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd == -1) {
    printf("socket creation failed...\n");
    exit(0);
  } else
    printf("Socket successfully created..\n");
  servaddr = (struct sockaddr_in){0};

  // assign IP, PORT
  printf("Connecting to sensor at %s:%u\n", sensor_ip, sensor_port);

  servaddr.sin_family      = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(sensor_ip);
  servaddr.sin_port        = htons(sensor_port);

  // connect the client socket to server socket
  if (connect(sockfd, (struct sockaddr *) &servaddr, sizeof(servaddr)) != 0) {
    printf("connection with the server failed...\n");
    exit(0);
  } else
    printf("connected to the server..\n");

  // set high priority
  int flag = 1;
  setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));

  // Writing
  printf("Writing to socket\n");
  const int write_result =
      write(sockfd, &commandRequestList, sizeof(commandRequestList));
  if (write_result < 0)
    printf("error writing to socket\n");
  else
    printf("Wrote to socket\n");

  // close the socket
  close(sockfd);
}

//@}
