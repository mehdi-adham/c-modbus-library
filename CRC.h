/**
 * @file CRC.h
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library calculate CRC for RTU Modbus Protocol.
 * @version 0.1
 * @date 2022-08-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __CRC_H
#define __CRC_H

#ifdef __cplusplus
extern "C"
{
#endif

unsigned short CRC16(const unsigned char *puchMsg, unsigned short usDataLen);

#ifdef __cplusplus
}
#endif
#endif
