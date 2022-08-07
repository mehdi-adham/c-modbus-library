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
extern "C"
{
#endif

#include <stdbool.h>

#define SLAVE_ADDRESS       0x11
#define MAX_BUFFER          256
#define MAX_SLAVE_ADDRESS   247

int request_timeout = 500;
int request_timer = 0; 
int RX_PIN ;
int DIR_PIN;
long long int *timer;

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
void MODBUS_RTU_MONITOR(unsigned char *mbus_frame_buffer, int monitor_fun_timeout,
                        unsigned char (*receive_uart_fun)());
#ifdef __cplusplus
}
#endif
#endif
