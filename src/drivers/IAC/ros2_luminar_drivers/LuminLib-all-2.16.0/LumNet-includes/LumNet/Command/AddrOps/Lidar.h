/**
 * @file   Lidar.h
 * @author Luminar Technologies
 * @date   2018
 * @brief  TODO
 */

#ifndef LUM_NET_COMMAND_ADDR_OPS_LIDAR_H
#define LUM_NET_COMMAND_ADDR_OPS_LIDAR_H

/*********************************/
/* Includes                      */
/*********************************/
//@{

#include <LumNet/Command/AddrOps.h>
#include <LumNet/Command/AddrOps/Lidar/Data.h>
#include <LumNet/Command/AddrOps/Lidar/Detector.h>
#include <LumNet/Command/AddrOps/Lidar/Laser.h>
#include <LumNet/Command/AddrOps/Lidar/Scanner.h>

//@}

/*********************************/
/* GLOBAL Macro Definitions      */
/*********************************/
//@{

#define _LUM_NET_CMD_VERIFY_ADDR_BASE_LIDAR_OPS_ALLOWED(addr, ...)                \
  ((LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER == _LUM_NET_GET_ADDR_INDEX(addr))        \
       ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_LIDAR_SCANNER_OPS_ALLOWED(addr,           \
                                                                  __VA_ARGS__)    \
       : (LUM_NET_CMD_ADDR_INDEX_LIDAR_LASER == _LUM_NET_GET_ADDR_INDEX(addr))    \
             ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_LIDAR_LASER_OPS_ALLOWED(            \
                   addr, __VA_ARGS__)                                             \
             : (LUM_NET_CMD_ADDR_INDEX_LIDAR_DETECTOR ==                          \
                _LUM_NET_GET_ADDR_INDEX(addr))                                    \
                   ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_LIDAR_DETECTOR_OPS_ALLOWED(   \
                         addr, __VA_ARGS__)                                       \
                   : (LUM_NET_CMD_ADDR_INDEX_LIDAR_DATA ==                        \
                      _LUM_NET_GET_ADDR_INDEX(addr))                              \
                         ? _LUM_NET_CMD_VERIFY_ADDR_INDEX_LIDAR_DATA_OPS_ALLOWED( \
                               addr, __VA_ARGS__)                                 \
                         : false)

//@}

/*********************************/
/* GLOBAL Type(def) Declarations */
/*********************************/
//@{

enum LumNet_CmdAddrIndexLidar {
  LUM_NET_CMD_ADDR_INDEX_LIDAR_SCANNER  = 0x01,
  LUM_NET_CMD_ADDR_INDEX_LIDAR_LASER    = 0x02,
  LUM_NET_CMD_ADDR_INDEX_LIDAR_DETECTOR = 0x03,
  LUM_NET_CMD_ADDR_INDEX_LIDAR_DATA     = 0x04,
};

//@}

/*********************************/
/* GLOBAL Variable Declarations  */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Function Declarations  */
/*********************************/
//@{

//@}

/*********************************/
/* GLOBAL Function Definitions   */
/*********************************/
//@{

//@}

#endif // LUM_NET_COMMAND_ADDR_OPS_LIDAR_H