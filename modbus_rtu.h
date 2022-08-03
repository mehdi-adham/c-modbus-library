/**
 * @file modbus_rtu.h
 * @author Mehdi Adham (mehdi.adham@domain.com)
 * @brief This library implements the Modbus RTU protocol.
 * @version 0.1
 * @date 2022-07-30
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#ifdef __cplusplus
    extern "C" {
#endif

#define SLAVE_ADDRESS   3
#define MAX_BUFFER      256

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
bool Receive_byte_to_byte(unsigned char *mbus_frame_buffer , unsigned char (*receive_uart_funt)());

#ifdef __cplusplus
}
#endif
#endif

