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

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
ModbusStatus_t MODBUS_RTU_MONITOR(unsigned char *mbus_frame_buffer,
		int monitor_fun_timeout, volatile uint32_t *Tick);

#ifdef __cplusplus
}
#endif
#endif
